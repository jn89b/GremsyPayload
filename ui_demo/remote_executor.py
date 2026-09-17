#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""
Remote executor service for Gremsy Lynx.

Runs on the machine physically connected to the Gremsy payload/gimbal.

Features
--------
- Click to point camera (EagleEyes)
- Click to track
- Continuous zoom in / out / stop
- Optional ArduPilot MAVLink input
- Forwards ArduPilot GPS into Gremsy
- Receives:
    * vehicle GPS + relative altitude
    * vehicle roll/pitch/yaw
    * Gremsy gimbal roll/pitch/yaw
    * Gremsy tracking pixel
    * Gremsy EO camera FOV
- Computes an approximate target GPS location by intersecting the camera
  line-of-sight ray with a flat ground plane.
- GET_GEO_STATUS returns the live estimator state to the PyQt UI.

IMPORTANT
---------
This is a geometric estimate, NOT a surveyed target position.

Without an LRF or terrain model, target range comes from a flat-ground
intersection. Accuracy is sensitive to:
- camera/gimbal attitude error
- vehicle attitude error
- GPS error
- camera mounting offsets
- ground-height error
- near-horizon geometry

For ground testing, use:
    --camera-height-agl <meters>

Example:
    python3 remote_executor.py \
        --ardupilot-endpoint udpin:0.0.0.0:14555 \
        --camera-height-agl 2.0

For flight over reasonably flat terrain you can omit --camera-height-agl.
The estimator first uses GLOBAL_POSITION_INT.relative_alt and then falls back
to GLOBAL_POSITION_INT.alt - HOME_POSITION.altitude.
"""

import argparse
import json
import math
import os
import sys
import threading
import time
from typing import Dict, List, Optional, Sequence, Tuple

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(__file__),
        "..",
        "libs",
    ),
)

from remote_bridge import BridgeConfig, TcpCommandBridge

try:
    from pymavlink import mavutil
    from payload_define import (
        PAYLOAD_CAMERA_GIMBAL_MODE,
        PAYLOAD_CAMERA_IR_PALETTE,
        PAYLOAD_CAMERA_IR_ZOOM_FACTOR,
        PAYLOAD_CAMERA_RECORD_SRC,
        camera_zoom_value,
        payload_camera_record_src,
    )
    from payload_sdk import (
        PayloadSdkInterface,
        camera_type_t,
        input_mode_t,
        mavlink_global_position_int_t,
        mavlink_gps_raw_int_t,
        payload_param_t,
        payload_status_event_t,
        tracking_mode_t,
    )
    from config import ConnectionConfig
except ImportError as exc:
    print(f"Error importing payload SDK modules: {exc}")
    sys.exit(1)


CMD_PAYLOAD_TOUCH = "PAYLOAD_TOUCH"
CMD_PAYLOAD_TRACK = "PAYLOAD_TRACK"
CMD_PAYLOAD_ZOOM_IN = "PAYLOAD_ZOOM_IN"
CMD_PAYLOAD_ZOOM_OUT = "PAYLOAD_ZOOM_OUT"
CMD_PAYLOAD_ZOOM_STOP = "PAYLOAD_ZOOM_STOP"
CMD_GET_GEO_STATUS = "GET_GEO_STATUS"
CMD_PAYLOAD_CAMERA_PARAM = "PAYLOAD_CAMERA_PARAM"
CMD_PAYLOAD_RECORD = "PAYLOAD_RECORD"  # params: [1 start | 0 stop]
# params: [GB_MODE int]; 0 off, 1 lock, 2 follow, 3 mapping, 4 reset (recenter)
CMD_PAYLOAD_GIMBAL_MODE = "PAYLOAD_GIMBAL_MODE"
# no params; angle command roll=0 keeping current pitch/yaw
CMD_PAYLOAD_GIMBAL_LEVEL_ROLL = "PAYLOAD_GIMBAL_LEVEL_ROLL"
# params: [heading_deg]; yaw the gimbal to an absolute compass heading,
# keeping current pitch, roll=0
CMD_PAYLOAD_GIMBAL_YAW_HEADING = "PAYLOAD_GIMBAL_YAW_HEADING"
# params: [1 on | 0 off]; keep re-commanding the last clicked heading as
# the vehicle yaws
CMD_PAYLOAD_GIMBAL_HEADING_HOLD = "PAYLOAD_GIMBAL_HEADING_HOLD"

# params: [name, int value]; allowlist of settable camera params
CAMERA_PARAMS = {
    "ir_palette": PAYLOAD_CAMERA_IR_PALETTE,
}

GREMSY_FRAME_W = 1920
GREMSY_FRAME_H = 1080
DEFAULT_TRACK_BOX = 128

DEFAULT_ARDUPILOT_ENDPOINT = os.environ.get(
    "ARDUPILOT_ENDPOINT",
    "udpin:0.0.0.0:14555",
)

EARTH_RADIUS_M = 6378137.0


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def ir_px_to_eo_px(
    x: float,
    y: float,
    ir_hfov: float,
    ir_vfov: float,
    eo_hfov: float,
    eo_vfov: float,
) -> Tuple[int, int, bool]:
    """Map a click on the IR image (normalized to the 1920x1080 frame) to the
    EO pixel that looks at the same direction. Returns (x, y, inside) where
    inside is False if the point fell outside the EO FOV before clamping.
    # ponytail: assumes co-boresighted sensors, no lens distortion.
    """

    def remap(px: float, size: int, ir_fov: float, eo_fov: float) -> float:
        u = px / size - 0.5
        tan_ir = math.tan(math.radians(ir_fov) / 2.0)
        tan_eo = math.tan(math.radians(eo_fov) / 2.0)
        return size * (0.5 + u * tan_ir / tan_eo)

    x_eo = remap(x, GREMSY_FRAME_W, ir_hfov, eo_hfov)
    y_eo = remap(y, GREMSY_FRAME_H, ir_vfov, eo_vfov)
    inside = (
        0 <= x_eo < GREMSY_FRAME_W and 0 <= y_eo < GREMSY_FRAME_H
    )
    return (
        int(round(_clamp(x_eo, 0, GREMSY_FRAME_W - 1))),
        int(round(_clamp(y_eo, 0, GREMSY_FRAME_H - 1))),
        inside,
    )


def _wrap_360(angle_deg: float) -> float:
    return angle_deg % 360.0


def _wrap_180(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


# ponytail: Gremsy yaw motor mechanical range, tune per gimbal model.
GIMBAL_YAW_LIMIT_DEG = 170.0


def heading_to_gimbal_yaw(
    heading_deg: float,
    vehicle_yaw_deg: float,
    yaw_offset_deg: float,
    frame: str,
    limit_deg: float = GIMBAL_YAW_LIMIT_DEG,
) -> Tuple[float, float, bool]:
    """Compass heading -> gimbal yaw setpoint, kept inside the motor range.

    Returns (yaw_cmd, body_yaw, clamped). body_yaw is the yaw relative to
    the aircraft nose; that is what the mechanical +-limit applies to, in
    every frame. yaw_cmd is body_yaw for a vehicle-frame gimbal, or
    body_yaw + vehicle heading for an earth-frame one. Inverse of
    _resolve_camera_attitude_ned.
    """
    body = _wrap_180(heading_deg - yaw_offset_deg - vehicle_yaw_deg)
    clamped = abs(body) > limit_deg
    if clamped:
        body = math.copysign(limit_deg, body)
    cmd = body if frame != "earth" else _wrap_180(body + vehicle_yaw_deg)
    return cmd, body, clamped


def _body_to_cmd(body_deg: float, vehicle_yaw_deg: float, frame: str) -> float:
    return body_deg if frame != "earth" else _wrap_180(body_deg + vehicle_yaw_deg)


def yaw_path(cur_body_deg: float, tgt_body_deg: float) -> List[float]:
    """Body-yaw waypoints from cur to tgt that never cross the rear stop.

    The firmware takes the shortest arc. When that arc passes +-180 the
    only way round is a stop at the nose first.
    """
    crosses_rear = (
        cur_body_deg * tgt_body_deg < 0
        and abs(cur_body_deg) + abs(tgt_body_deg) > 180.0
    )
    return [0.0, tgt_body_deg] if crosses_rear else [tgt_body_deg]


def _matmul3(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]):
    return [
        [
            sum(a[i][k] * b[k][j] for k in range(3))
            for j in range(3)
        ]
        for i in range(3)
    ]


def _matvec3(m: Sequence[Sequence[float]], v: Sequence[float]):
    return [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]


def _euler_frd_to_ned(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
):
    """
    Direction-cosine matrix for an FRD frame into NED.

    Aerospace Z-Y-X convention:
        R = Rz(yaw) * Ry(pitch) * Rx(roll)

    x = forward / north at zero attitude
    y = right   / east
    z = down
    """
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)

    rx = [
        [1.0, 0.0, 0.0],
        [0.0, cr, -sr],
        [0.0, sr, cr],
    ]
    ry = [
        [cp, 0.0, sp],
        [0.0, 1.0, 0.0],
        [-sp, 0.0, cp],
    ]
    rz = [
        [cy, -sy, 0.0],
        [sy, cy, 0.0],
        [0.0, 0.0, 1.0],
    ]

    return _matmul3(_matmul3(rz, ry), rx)


class RemoteExecutor:
    """Execute UI commands and maintain target-geolocation telemetry."""

    def __init__(
        self,
        role: str,
        host: str,
        port: int,
        token: str,
        ack_timeout: float,
        retries: int,
        payload_ip: str,
        ardupilot_endpoint: str,
        ardupilot_sysid: int,
        ardupilot_stale_timeout: float,
        position_stale_timeout: float,
        attitude_stale_timeout: float,
        gimbal_stale_timeout: float,
        track_stale_timeout: float,
        track_box: int,
        camera_height_agl: float,
        ground_alt_m: Optional[float],
        min_down_angle_deg: float,
        max_ground_range_m: float,
        fallback_hfov_deg: float,
        fallback_vfov_deg: float,
        ir_hfov_deg: float,
        ir_vfov_deg: float,
        camera_roll_offset_deg: float,
        camera_pitch_offset_deg: float,
        camera_yaw_offset_deg: float,
    ):
        self.bridge = TcpCommandBridge(
            BridgeConfig(
                role=role,
                host=host,
                port=port,
                token=token,
                ack_timeout=ack_timeout,
                retry_count=retries,
            ),
            logger=self._log,
        )

        self.sdk: Optional[PayloadSdkInterface] = None
        self.running = True
        self.is_connected = False
        self.payload_ip = payload_ip

        self.track_on_click = False
        self.track_box = max(
            16,
            min(512, int(track_box)),
        )

        # SDK writes can come from the command thread, MAVLink forwarding
        # thread, and FOV polling thread.
        self._sdk_send_lock = threading.Lock()

        # Last IR palette reported by the camera (None until it answers).
        self.ir_palette: Optional[int] = None
        # Last IR zoom level (C_T_ZOOM index 0..7 = 1x..8x, None until known).
        self.ir_zoom: Optional[int] = None

        # Camera-reported video_status from CAMERA_CAPTURE_STATUS.
        self.recording = False
        self._record_armed = False

        # ------------------------- estimator config ---------------------
        self.camera_height_agl = max(
            0.0,
            float(camera_height_agl),
        )
        self.ground_alt_m = (
            None
            if ground_alt_m is None
            else float(ground_alt_m)
        )
        self.min_down_angle_deg = max(
            0.1,
            float(min_down_angle_deg),
        )
        self.max_ground_range_m = max(
            1.0,
            float(max_ground_range_m),
        )

        self.fallback_hfov_deg = max(
            1.0,
            min(179.0, float(fallback_hfov_deg)),
        )
        self.fallback_vfov_deg = max(
            1.0,
            min(179.0, float(fallback_vfov_deg)),
        )
        # Native (1x) IR FOV; the zoomed FOV is derived from ir_zoom.
        self.ir_hfov_deg = max(
            1.0,
            min(179.0, float(ir_hfov_deg)),
        )
        self.ir_vfov_deg = max(
            1.0,
            min(179.0, float(ir_vfov_deg)),
        )

        self.camera_roll_offset_deg = float(
            camera_roll_offset_deg
        )
        self.camera_pitch_offset_deg = float(
            camera_pitch_offset_deg
        )
        self.camera_yaw_offset_deg = float(
            camera_yaw_offset_deg
        )

        # ------------------------- ArduPilot ----------------------------
        self.ardupilot_endpoint = (
            ardupilot_endpoint or ""
        ).strip()
        self.ardupilot_sysid = int(
            ardupilot_sysid
        )

        self.ardupilot_stale_timeout = max(
            0.5,
            float(ardupilot_stale_timeout),
        )
        self.position_stale_timeout = max(
            0.5,
            float(position_stale_timeout),
        )
        self.attitude_stale_timeout = max(
            0.5,
            float(attitude_stale_timeout),
        )
        self.gimbal_stale_timeout = max(
            0.5,
            float(gimbal_stale_timeout),
        )
        self.track_stale_timeout = max(
            0.5,
            float(track_stale_timeout),
        )

        self._ap_master = None
        self._ap_thread: Optional[
            threading.Thread
        ] = None

        self._ap_target_sysid: Optional[int] = (
            self.ardupilot_sysid
            if self.ardupilot_sysid > 0
            else None
        )

        # ------------------------- Gremsy poll --------------------------
        self._payload_poll_thread: Optional[
            threading.Thread
        ] = None

        # ------------------------- state --------------------------------
        self._state_lock = threading.Lock()

        # ArduPilot health
        self._last_ap_heartbeat_mono = 0.0
        self._last_ap_position_mono = 0.0
        self._last_ap_attitude_mono = 0.0
        self._gps_raw_seen = False
        self._gps_fix_type = 0

        # Vehicle
        self._vehicle_lat: Optional[float] = None
        self._vehicle_lon: Optional[float] = None
        self._vehicle_alt_m: Optional[float] = None
        self._vehicle_relative_alt_m: Optional[
            float
        ] = None

        # Home altitude (AMSL) is used as a flight fallback when
        # GLOBAL_POSITION_INT.relative_alt is unavailable/zero.
        self._home_alt_m: Optional[float] = None
        self._last_home_request_mono = 0.0

        self._vehicle_roll_deg: Optional[
            float
        ] = None
        self._vehicle_pitch_deg: Optional[
            float
        ] = None
        self._vehicle_yaw_deg: Optional[
            float
        ] = None

        # Gimbal attitude returned by PayloadSdk
        self._gimbal_roll_deg: Optional[
            float
        ] = None
        self._gimbal_pitch_deg: Optional[
            float
        ] = None
        self._gimbal_yaw_deg: Optional[
            float
        ] = None
        self._gimbal_mode = ""
        self._gimbal_flags = 0
        self._gimbal_frame = "unknown"
        # Bumped per compass click; a queued via-nose leg checks it.
        self._yaw_seq = 0
        self._hold_on = False
        self._hold_heading_deg: Optional[float] = None
        self._hold_last_cmd: Optional[float] = None
        self._last_gimbal_mono = 0.0

        # Tracker
        self._track_status: Optional[int] = None
        self._track_x: Optional[float] = None
        self._track_y: Optional[float] = None
        self._track_w: Optional[float] = None
        self._track_h: Optional[float] = None
        self._last_track_param_mono = 0.0

        # Last acquisition click is a fallback before Gremsy starts
        # publishing live tracker coordinates.
        self._selected_x: Optional[float] = None
        self._selected_y: Optional[float] = None
        self._last_selected_mono = 0.0

        # Camera FOV
        self._hfov_deg: Optional[float] = None
        self._vfov_deg: Optional[float] = None
        self._last_fov_mono = 0.0

        # Optional native Gremsy target output, retained for comparison.
        self._gremsy_target_lat: Optional[
            float
        ] = None
        self._gremsy_target_lon: Optional[
            float
        ] = None
        self._gremsy_target_alt: Optional[
            float
        ] = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> int:
        ConnectionConfig.UDP_IP_TARGET = (
            self.payload_ip
        )

        self.sdk = PayloadSdkInterface()

        if not self.sdk.sdkInitConnection():
            self._log(
                "payload SDK init failed"
            )
            return 1

        if not self._wait_payload_connection(
            timeout=8.0
        ):
            self._log(
                "payload connection timeout"
            )
            self.stop()
            return 1

        self._configure_payload_telemetry()
        self._start_payload_poll_worker()
        self._start_ardupilot_worker()

        self.bridge.start()

        self._log(
            "executor ready, waiting for commands"
        )

        try:
            self.bridge.serve_commands(
                self._handle_command
            )
        except KeyboardInterrupt:
            self._log(
                "executor interrupted"
            )
        finally:
            self.stop()

        return 0

    def stop(self) -> None:
        if not self.running:
            return

        self.running = False

        if self.sdk and self.is_connected:
            try:
                with self._sdk_send_lock:
                    self.sdk.setCameraZoom(
                        mavutil.mavlink.ZOOM_TYPE_CONTINUOUS,
                        camera_zoom_value.ZOOM_STOP,
                    )
            except Exception:
                pass

        self.bridge.stop()

        master = self._ap_master
        self._ap_master = None

        if master is not None:
            try:
                master.close()
            except Exception:
                pass

        if (
            self._ap_thread
            and self._ap_thread.is_alive()
        ):
            self._ap_thread.join(
                timeout=2.0
            )

        if (
            self._payload_poll_thread
            and self._payload_poll_thread.is_alive()
        ):
            self._payload_poll_thread.join(
                timeout=2.0
            )

        if self.sdk:
            try:
                self.sdk.sdkQuit()
            finally:
                self.sdk = None

        self.is_connected = False

        self._log("executor stopped")

    def _wait_payload_connection(
        self,
        timeout: float,
    ) -> bool:
        start = time.time()

        while (
            self.running
            and time.time() - start < timeout
        ):
            if (
                self.sdk
                and self.sdk.checkPayloadConnection()
            ):
                self.is_connected = True

                self._log(
                    f"payload connected at "
                    f"{self.payload_ip}"
                )
                return True

            time.sleep(0.1)

        return False

    # ------------------------------------------------------------------
    # Gremsy telemetry
    # ------------------------------------------------------------------

    def _configure_payload_telemetry(
        self,
    ) -> None:
        if self.sdk is None:
            return

        # PAYLOAD_PARAMS + CAMERA_FOV_STATUS + MOUNT_ORIENTATION fallback.
        self.sdk.regPayloadStatusChanged(
            self._on_payload_status
        )

        # GIMBAL_DEVICE_ATTITUDE_STATUS.
        self.sdk.regPayloadParamChanged(
            self._on_payload_param_changed
        )

        # Ask once for the current IR palette and zoom so the UI can show them.
        with self._sdk_send_lock:
            self.sdk.getPayloadCameraSettingByID(
                PAYLOAD_CAMERA_IR_PALETTE
            )
            self.sdk.getPayloadCameraSettingByID(
                PAYLOAD_CAMERA_IR_ZOOM_FACTOR
            )

        requested_params = [
            payload_param_t.PARAM_TRACK_POS_X,
            payload_param_t.PARAM_TRACK_POS_Y,
            payload_param_t.PARAM_TRACK_POS_W,
            payload_param_t.PARAM_TRACK_POS_H,
            payload_param_t.PARAM_TRACK_STATUS,
            payload_param_t.PARAM_EO_ZOOM_LEVEL,
            # Native target location is optional and is kept only as a
            # comparison against our own geometry-based estimator.
            payload_param_t.PARAM_TARGET_COOR_LAT,
            payload_param_t.PARAM_TARGET_COOR_LON,
            payload_param_t.PARAM_TARGET_COOR_ALT,
        ]

        try:
            with self._sdk_send_lock:
                for param_index in requested_params:
                    self.sdk.setParamRate(
                        param_index,
                        200,
                    )

                # Request the gimbal-device attitude at ~5 Hz.
                if (
                    self.sdk.master is not None
                ):
                    self.sdk.master.mav.command_long_send(
                        self.sdk.gimbal_system_id,
                        self.sdk.gimbal_component_id,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                        0,
                        mavutil.mavlink.MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS,
                        200000,
                        0,
                        0,
                        0,
                        0,
                        0,
                    )
        except Exception as exc:
            self._log(
                "unable to configure payload telemetry: "
                f"{exc}"
            )

    def _start_payload_poll_worker(
        self,
    ) -> None:
        self._payload_poll_thread = threading.Thread(
            target=self._payload_poll_loop,
            daemon=True,
            name="gremsy-fov-poll",
        )
        self._payload_poll_thread.start()

    def _payload_poll_loop(self) -> None:
        while self.running:
            if (
                self.sdk is not None
                and self.is_connected
            ):
                try:
                    with self._sdk_send_lock:
                        self.sdk.getPayloadCameraFOVStatus(
                            camera_type_t.CAMERA_EO
                        )
                except Exception as exc:
                    self._log(
                        f"FOV request failed: {exc}"
                    )

            time.sleep(1.0)

    def _on_payload_status(
        self,
        event,
        param: List[float],
    ) -> None:
        try:
            event_value = int(event)
            now = time.monotonic()

            if event_value == int(
                payload_status_event_t.PAYLOAD_CAM_CAPTURE_STATUS
            ):
                # param: [image_status, video_status, image_count, rec_ms]
                if len(param) >= 2:
                    self.recording = int(param[1]) != 0
                return

            if (
                event_value
                == int(
                    payload_status_event_t.PAYLOAD_PARAMS
                )
            ):
                if len(param) < 2:
                    return

                index = int(param[0])
                value = float(param[1])

                if not math.isfinite(value):
                    return

                with self._state_lock:
                    if index == int(
                        payload_param_t.PARAM_TRACK_POS_X
                    ):
                        self._track_x = value
                        self._last_track_param_mono = now

                    elif index == int(
                        payload_param_t.PARAM_TRACK_POS_Y
                    ):
                        self._track_y = value
                        self._last_track_param_mono = now

                    elif index == int(
                        payload_param_t.PARAM_TRACK_POS_W
                    ):
                        self._track_w = value

                    elif index == int(
                        payload_param_t.PARAM_TRACK_POS_H
                    ):
                        self._track_h = value

                    elif index == int(
                        payload_param_t.PARAM_TRACK_STATUS
                    ):
                        self._track_status = int(
                            round(value)
                        )
                        self._last_track_param_mono = now

                    elif index == int(
                        payload_param_t.PARAM_TARGET_COOR_LAT
                    ):
                        self._gremsy_target_lat = value

                    elif index == int(
                        payload_param_t.PARAM_TARGET_COOR_LON
                    ):
                        self._gremsy_target_lon = value

                    elif index == int(
                        payload_param_t.PARAM_TARGET_COOR_ALT
                    ):
                        self._gremsy_target_alt = value

                return

            if (
                event_value
                == int(
                    payload_status_event_t.PAYLOAD_PARAM_CAM_FOV_STATUS
                )
            ):
                # PayloadSdk supplies [camera_id, hfov, vfov].
                if len(param) < 3:
                    return

                hfov = float(param[1])
                vfov = float(param[2])

                if (
                    0.1 < hfov < 179.0
                    and 0.1 < vfov < 179.0
                ):
                    with self._state_lock:
                        self._hfov_deg = hfov
                        self._vfov_deg = vfov
                        self._last_fov_mono = now
                return

            # MOUNT_ORIENTATION fallback. PayloadSdk supplies:
            # [pitch, roll, yaw]
            if (
                event_value
                == int(
                    payload_status_event_t.PAYLOAD_GB_ATTITUDE
                )
                and len(param) >= 3
            ):
                pitch = float(param[0])
                roll = float(param[1])
                yaw = float(param[2])

                with self._state_lock:
                    # Do not overwrite a fresh GIMBAL_DEVICE attitude.
                    if (
                        now
                        - self._last_gimbal_mono
                        > 0.75
                    ):
                        self._gimbal_pitch_deg = pitch
                        self._gimbal_roll_deg = roll
                        self._gimbal_yaw_deg = yaw
                        self._gimbal_mode = (
                            "MOUNT_ORIENTATION"
                        )
                        self._gimbal_frame = (
                            "vehicle"
                        )
                        self._gimbal_flags = 0
                        self._last_gimbal_mono = now

        except Exception as exc:
            self._log(
                f"payload status callback error: {exc}"
            )

    def _on_payload_param_changed(
        self,
        event,
        param_mode: str,
        params: List[float],
    ) -> None:
        try:
            if (
                int(event)
                == int(
                    payload_status_event_t.PAYLOAD_CAM_PARAMS
                )
                and str(param_mode).rstrip("\x00")
                == PAYLOAD_CAMERA_IR_PALETTE
                and len(params) >= 2
            ):
                self.ir_palette = int(params[1])
                return

            if (
                int(event)
                == int(
                    payload_status_event_t.PAYLOAD_CAM_PARAMS
                )
                and str(param_mode).rstrip("\x00")
                == PAYLOAD_CAMERA_IR_ZOOM_FACTOR
                and len(params) >= 2
            ):
                self.ir_zoom = int(params[1])
                return

            if (
                int(event)
                != int(
                    payload_status_event_t.PAYLOAD_GB_ATTITUDE
                )
            ):
                return

            if len(params) < 3:
                return

            # PayloadSdk ordering:
            # [pitch_deg, roll_deg, yaw_deg, wx, wy, wz]
            pitch = float(params[0])
            roll = float(params[1])
            yaw = float(params[2])

            flags = 0

            if self.sdk is not None:
                flags = int(
                    getattr(
                        self.sdk,
                        "current_attitude_flags",
                        0,
                    )
                    or 0
                )

            yaw_in_earth = bool(
                flags
                & getattr(
                    mavutil.mavlink,
                    "GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME",
                    64,
                )
            )
            yaw_in_vehicle = bool(
                flags
                & getattr(
                    mavutil.mavlink,
                    "GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME",
                    32,
                )
            )
            yaw_lock = bool(
                flags
                & mavutil.mavlink.GIMBAL_DEVICE_FLAGS_YAW_LOCK
            )

            if yaw_in_earth or (
                not yaw_in_vehicle
                and yaw_lock
            ):
                frame = "earth"
            else:
                frame = "vehicle"

            with self._state_lock:
                self._gimbal_pitch_deg = pitch
                self._gimbal_roll_deg = roll
                self._gimbal_yaw_deg = yaw
                self._gimbal_mode = str(
                    param_mode or ""
                )
                self._gimbal_flags = flags
                self._gimbal_frame = frame
                self._last_gimbal_mono = (
                    time.monotonic()
                )

        except Exception as exc:
            self._log(
                f"gimbal attitude callback error: {exc}"
            )

    # ------------------------------------------------------------------
    # ArduPilot
    # ------------------------------------------------------------------

    def _start_ardupilot_worker(
        self,
    ) -> None:
        if (
            not self.ardupilot_endpoint
            or self.ardupilot_endpoint.lower()
            in {"none", "off", "disabled"}
        ):
            self._log(
                "ArduPilot input disabled"
            )
            return

        self._ap_thread = threading.Thread(
            target=self._ardupilot_loop,
            daemon=True,
            name="ardupilot-telemetry",
        )
        self._ap_thread.start()

    def _ardupilot_loop(self) -> None:
        while self.running:
            master = None

            try:
                self._log(
                    "opening optional ArduPilot MAVLink "
                    f"endpoint {self.ardupilot_endpoint}"
                )

                master = mavutil.mavlink_connection(
                    self.ardupilot_endpoint,
                    source_system=255,
                    source_component=(
                        mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER
                    ),
                    autoreconnect=True,
                )

                self._ap_master = master

                while self.running:
                    msg = master.recv_match(
                        blocking=True,
                        timeout=0.5,
                    )

                    if msg is None:
                        continue

                    msg_type = msg.get_type()

                    if msg_type == "BAD_DATA":
                        continue

                    src_sysid = int(
                        msg.get_srcSystem()
                    )

                    if not self._accept_ardupilot_system(
                        msg,
                        src_sysid,
                    ):
                        continue

                    now = time.monotonic()

                    if msg_type == "HEARTBEAT":
                        with self._state_lock:
                            self._last_ap_heartbeat_mono = now

                        # HOME_POSITION is not guaranteed to be streamed
                        # continuously. Request it periodically until received.
                        self._request_ardupilot_home_position(
                            master,
                            src_sysid,
                            now,
                        )
                        continue

                    if msg_type == "HOME_POSITION":
                        self._handle_ardupilot_home_position(
                            msg,
                            now,
                        )
                        continue

                    if msg_type == "ATTITUDE":
                        self._handle_ardupilot_attitude(
                            msg,
                            now,
                        )
                        continue

                    if msg_type == "GPS_RAW_INT":
                        self._handle_ardupilot_gps_raw(
                            msg,
                            now,
                        )
                        continue

                    if (
                        msg_type
                        == "GLOBAL_POSITION_INT"
                    ):
                        self._handle_ardupilot_global_position(
                            msg,
                            now,
                        )
                        continue

            except Exception as exc:
                if self.running:
                    self._log(
                        f"ArduPilot MAVLink error: {exc}"
                    )

            finally:
                if master is not None:
                    try:
                        master.close()
                    except Exception:
                        pass

                if self._ap_master is master:
                    self._ap_master = None

            if self.running:
                time.sleep(2.0)

    def _accept_ardupilot_system(
        self,
        msg,
        src_sysid: int,
    ) -> bool:
        if self.ardupilot_sysid > 0:
            return (
                src_sysid
                == self.ardupilot_sysid
            )

        if self._ap_target_sysid is not None:
            return (
                src_sysid
                == self._ap_target_sysid
            )

        if msg.get_type() == "HEARTBEAT":
            autopilot = int(
                getattr(
                    msg,
                    "autopilot",
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                )
            )

            component = int(
                msg.get_srcComponent()
            )

            if (
                autopilot
                != mavutil.mavlink.MAV_AUTOPILOT_INVALID
                or component
                == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
            ):
                self._ap_target_sysid = (
                    src_sysid
                )

                self._log(
                    "selected ArduPilot MAVLink "
                    f"sysid={src_sysid}"
                )
                return True

        return False

    def _request_ardupilot_home_position(
        self,
        master,
        src_sysid: int,
        now: float,
    ) -> None:
        """Request HOME_POSITION until a valid home altitude is received."""
        with self._state_lock:
            home_alt_m = self._home_alt_m
            last_request = self._last_home_request_mono

        if home_alt_m is not None:
            return

        # Avoid sending a request on every heartbeat.
        if last_request > 0.0 and now - last_request < 5.0:
            return

        try:
            target_sysid = (
                self._ap_target_sysid
                if self._ap_target_sysid is not None
                else src_sysid
            )
            master.mav.command_long_send(
                int(target_sysid),
                mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                0,
                mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION,
                0,
                0,
                0,
                0,
                0,
                0,
            )

            with self._state_lock:
                self._last_home_request_mono = now

        except Exception as exc:
            self._log(
                f"unable to request HOME_POSITION: {exc}"
            )

    def _handle_ardupilot_home_position(
        self,
        msg,
        now: float,
    ) -> None:
        """Store ArduPilot home altitude in meters AMSL."""
        try:
            home_alt_m = float(msg.altitude) / 1000.0
        except Exception:
            return

        if not math.isfinite(home_alt_m):
            return

        with self._state_lock:
            self._home_alt_m = home_alt_m

        self._log(
            f"ArduPilot HOME_POSITION altitude={home_alt_m:.2f} m AMSL"
        )

    def _handle_ardupilot_attitude(
        self,
        msg,
        now: float,
    ) -> None:
        with self._state_lock:
            self._vehicle_roll_deg = (
                math.degrees(
                    float(msg.roll)
                )
            )
            self._vehicle_pitch_deg = (
                math.degrees(
                    float(msg.pitch)
                )
            )
            self._vehicle_yaw_deg = (
                _wrap_360(
                    math.degrees(
                        float(msg.yaw)
                    )
                )
            )
            self._last_ap_attitude_mono = now
            hold = self._hold_on
            heading = self._hold_heading_deg
        if hold and heading is not None:
            self._yaw_to_heading(heading, only_if_changed=True)

    def _handle_ardupilot_global_position(
        self,
        msg,
        now: float,
    ) -> None:
        lat_deg = (
            float(msg.lat) / 1.0e7
        )
        lon_deg = (
            float(msg.lon) / 1.0e7
        )
        alt_m = (
            float(msg.alt) / 1000.0
        )
        relative_alt_m = (
            float(msg.relative_alt)
            / 1000.0
        )

        with self._state_lock:
            self._last_ap_position_mono = now
            self._vehicle_lat = lat_deg
            self._vehicle_lon = lon_deg
            self._vehicle_alt_m = alt_m
            self._vehicle_relative_alt_m = (
                relative_alt_m
            )

            # If relative_alt is valid, it also gives us a useful home-altitude
            # estimate for fallback/debugging.
            if (
                math.isfinite(relative_alt_m)
                and relative_alt_m > 0.05
            ):
                self._home_alt_m = (
                    alt_m - relative_alt_m
                )

        # Forward the vehicle position to Gremsy as well.
        if (
            self.sdk is None
            or not self.is_connected
        ):
            return

        gps = mavlink_global_position_int_t()
        gps.time_boot_ms = int(
            msg.time_boot_ms
        )
        gps.lat = int(msg.lat)
        gps.lon = int(msg.lon)
        gps.alt = int(msg.alt)
        gps.relative_alt = int(
            msg.relative_alt
        )
        gps.vx = int(msg.vx)
        gps.vy = int(msg.vy)
        gps.vz = int(msg.vz)
        gps.hdg = int(msg.hdg)

        try:
            with self._sdk_send_lock:
                self.sdk.sendPayloadGPSPosition(
                    gps
                )
        except Exception as exc:
            self._log(
                "failed forwarding "
                f"GLOBAL_POSITION_INT: {exc}"
            )

    def _handle_ardupilot_gps_raw(
        self,
        msg,
        now: float,
    ) -> None:
        fix_type = int(
            getattr(
                msg,
                "fix_type",
                0,
            )
        )

        with self._state_lock:
            self._gps_raw_seen = True
            self._gps_fix_type = fix_type

        if (
            self.sdk is None
            or not self.is_connected
        ):
            return

        gps = mavlink_gps_raw_int_t()
        gps.time_usec = int(
            getattr(msg, "time_usec", 0)
        )
        gps.lat = int(
            getattr(msg, "lat", 0)
        )
        gps.lon = int(
            getattr(msg, "lon", 0)
        )
        gps.alt = int(
            getattr(msg, "alt", 0)
        )
        gps.eph = int(
            getattr(msg, "eph", 65535)
        )
        gps.epv = int(
            getattr(msg, "epv", 65535)
        )
        gps.vel = int(
            getattr(msg, "vel", 65535)
        )
        gps.cog = int(
            getattr(msg, "cog", 65535)
        )
        gps.fix_type = fix_type
        gps.satellites_visible = int(
            getattr(
                msg,
                "satellites_visible",
                255,
            )
        )
        gps.alt_ellipsoid = int(
            getattr(
                msg,
                "alt_ellipsoid",
                0,
            )
        )
        gps.h_acc = int(
            getattr(msg, "h_acc", 0)
        )
        gps.v_acc = int(
            getattr(msg, "v_acc", 0)
        )
        gps.vel_acc = int(
            getattr(msg, "vel_acc", 0)
        )
        gps.hdg_acc = int(
            getattr(msg, "hdg_acc", 0)
        )
        gps.yaw = int(
            getattr(msg, "yaw", 0)
        )

        try:
            with self._sdk_send_lock:
                self.sdk.sendPayloadGPSRawInt(
                    gps
                )
        except Exception as exc:
            self._log(
                "failed forwarding "
                f"GPS_RAW_INT: {exc}"
            )

    # ------------------------------------------------------------------
    # Target geolocation
    # ------------------------------------------------------------------

    def _resolve_camera_attitude_ned(
        self,
        vehicle_roll: float,
        vehicle_pitch: float,
        vehicle_yaw: float,
        gimbal_roll: float,
        gimbal_pitch: float,
        gimbal_yaw: float,
        gimbal_flags: int,
        gimbal_frame: str,
    ) -> Tuple[float, float, float, str]:
        """
        Convert the available attitude information into an approximate
        earth/NED camera attitude.

        MAVLink's GIMBAL_DEVICE_ATTITUDE_STATUS can report yaw in either
        the earth or vehicle-heading frame. Roll/pitch lock flags also tell
        whether those axes are horizon-locked.

        This resolves each axis conservatively:
        - locked axis -> treat gimbal angle as earth referenced
        - unlocked axis -> add vehicle attitude
        """
        roll_lock = bool(
            gimbal_flags
            & getattr(
                mavutil.mavlink,
                "GIMBAL_DEVICE_FLAGS_ROLL_LOCK",
                4,
            )
        )
        pitch_lock = bool(
            gimbal_flags
            & getattr(
                mavutil.mavlink,
                "GIMBAL_DEVICE_FLAGS_PITCH_LOCK",
                8,
            )
        )

        if roll_lock:
            roll_abs = gimbal_roll
        else:
            roll_abs = (
                vehicle_roll
                + gimbal_roll
            )

        if pitch_lock:
            pitch_abs = gimbal_pitch
        else:
            pitch_abs = (
                vehicle_pitch
                + gimbal_pitch
            )

        if gimbal_frame == "earth":
            yaw_abs = gimbal_yaw
            frame_note = "gimbal yaw earth"
        else:
            yaw_abs = (
                vehicle_yaw
                + gimbal_yaw
            )
            frame_note = "gimbal yaw vehicle"

        roll_abs += self.camera_roll_offset_deg
        pitch_abs += self.camera_pitch_offset_deg
        yaw_abs += self.camera_yaw_offset_deg

        return (
            roll_abs,
            pitch_abs,
            _wrap_360(yaw_abs),
            frame_note,
        )

    def _get_height_agl(
        self,
        vehicle_alt_m: Optional[float],
        relative_alt_m: Optional[float],
        home_alt_m: Optional[float],
    ) -> Tuple[Optional[float], Optional[float], str]:
        """
        Return:
            height_agl_m,
            estimated_ground_alt_m,
            source
        """
        if (
            self.ground_alt_m is not None
            and vehicle_alt_m is not None
        ):
            height = (
                vehicle_alt_m
                - self.ground_alt_m
            )

            if height > 0.05:
                return (
                    height,
                    self.ground_alt_m,
                    "ground-alt",
                )

        if self.camera_height_agl > 0.05:
            ground_alt = (
                None
                if vehicle_alt_m is None
                else vehicle_alt_m
                - self.camera_height_agl
            )

            return (
                self.camera_height_agl,
                ground_alt,
                "camera-height-agl",
            )

        if (
            relative_alt_m is not None
            and math.isfinite(relative_alt_m)
            and relative_alt_m > 0.05
        ):
            ground_alt = (
                None
                if vehicle_alt_m is None
                else vehicle_alt_m
                - relative_alt_m
            )

            return (
                relative_alt_m,
                ground_alt,
                "relative-alt",
            )

        # Flight fallback: HOME_POSITION altitude is AMSL, so subtract it
        # from GLOBAL_POSITION_INT.alt (also AMSL). This still assumes the
        # target terrain is near the home elevation.
        if (
            vehicle_alt_m is not None
            and home_alt_m is not None
            and math.isfinite(vehicle_alt_m)
            and math.isfinite(home_alt_m)
        ):
            height = vehicle_alt_m - home_alt_m

            if height > 0.05:
                return (
                    height,
                    home_alt_m,
                    "msl-minus-home",
                )

        return (
            None,
            None,
            "none",
        )

    def _choose_tracking_pixel(
        self,
        now: float,
        track_x: Optional[float],
        track_y: Optional[float],
        track_update: float,
        selected_x: Optional[float],
        selected_y: Optional[float],
        selected_update: float,
    ) -> Tuple[Optional[float], Optional[float], str]:
        # Best source: live tracker output.
        if (
            track_x is not None
            and track_y is not None
            and track_update > 0.0
            and now - track_update
            <= self.track_stale_timeout
        ):
            return (
                _clamp(
                    track_x,
                    0.0,
                    GREMSY_FRAME_W - 1.0,
                ),
                _clamp(
                    track_y,
                    0.0,
                    GREMSY_FRAME_H - 1.0,
                ),
                "gremsy-track",
            )

        # Immediately after acquisition, use the selected pixel.
        if (
            selected_x is not None
            and selected_y is not None
            and selected_update > 0.0
            and now - selected_update < 1.5
        ):
            return (
                selected_x,
                selected_y,
                "selected-pixel",
            )

        # If tracking is still enabled, Gremsy's job is to keep the tracked
        # object near image center. This is a more useful fallback than
        # retaining the original off-center click indefinitely.
        if self.track_on_click:
            return (
                GREMSY_FRAME_W / 2.0,
                GREMSY_FRAME_H / 2.0,
                "assumed-image-center",
            )

        return None, None, "none"

    def _estimate_target_location(
        self,
    ) -> Dict:
        now = time.monotonic()

        with self._state_lock:
            hb_time = self._last_ap_heartbeat_mono
            pos_time = self._last_ap_position_mono
            att_time = self._last_ap_attitude_mono

            gps_raw_seen = self._gps_raw_seen
            fix_type = self._gps_fix_type

            vehicle_lat = self._vehicle_lat
            vehicle_lon = self._vehicle_lon
            vehicle_alt_m = self._vehicle_alt_m
            relative_alt_m = (
                self._vehicle_relative_alt_m
            )
            home_alt_m = self._home_alt_m

            vehicle_roll = self._vehicle_roll_deg
            vehicle_pitch = (
                self._vehicle_pitch_deg
            )
            vehicle_yaw = self._vehicle_yaw_deg

            gimbal_roll = self._gimbal_roll_deg
            gimbal_pitch = self._gimbal_pitch_deg
            gimbal_yaw = self._gimbal_yaw_deg
            gimbal_mode = self._gimbal_mode
            gimbal_flags = self._gimbal_flags
            gimbal_frame = self._gimbal_frame
            gimbal_time = self._last_gimbal_mono

            track_status = self._track_status
            track_x = self._track_x
            track_y = self._track_y
            track_w = self._track_w
            track_h = self._track_h
            track_update = (
                self._last_track_param_mono
            )

            selected_x = self._selected_x
            selected_y = self._selected_y
            selected_update = (
                self._last_selected_mono
            )

            hfov = self._hfov_deg
            vfov = self._vfov_deg
            fov_time = self._last_fov_mono

            gremsy_target_lat = (
                self._gremsy_target_lat
            )
            gremsy_target_lon = (
                self._gremsy_target_lon
            )
            gremsy_target_alt = (
                self._gremsy_target_alt
            )

        heartbeat_age = (
            None
            if hb_time <= 0.0
            else max(0.0, now - hb_time)
        )
        position_age = (
            None
            if pos_time <= 0.0
            else max(0.0, now - pos_time)
        )
        attitude_age = (
            None
            if att_time <= 0.0
            else max(0.0, now - att_time)
        )
        gimbal_age = (
            None
            if gimbal_time <= 0.0
            else max(0.0, now - gimbal_time)
        )

        ardupilot_connected = (
            heartbeat_age is not None
            and heartbeat_age
            <= self.ardupilot_stale_timeout
        )

        position_fresh = (
            position_age is not None
            and position_age
            <= self.position_stale_timeout
        )

        attitude_fresh = (
            attitude_age is not None
            and attitude_age
            <= self.attitude_stale_timeout
        )

        gimbal_fresh = (
            gimbal_age is not None
            and gimbal_age
            <= self.gimbal_stale_timeout
        )

        if gps_raw_seen:
            gps_valid = (
                ardupilot_connected
                and position_fresh
                and fix_type >= 3
            )
        else:
            gps_valid = (
                ardupilot_connected
                and position_fresh
                and vehicle_lat is not None
                and vehicle_lon is not None
                and not (
                    abs(vehicle_lat) < 1e-12
                    and abs(vehicle_lon) < 1e-12
                )
            )

        result = {
            "ir_palette": self.ir_palette,
            "recording": self.recording,
            "heading_hold_on": self._hold_on,
            "heading_hold_deg": self._hold_heading_deg,
            "ardupilot_connected": (
                ardupilot_connected
            ),
            "ardupilot_sysid": (
                self._ap_target_sysid
            ),
            "gps_valid": gps_valid,
            "gps_fix_type": (
                fix_type
                if gps_raw_seen
                else None
            ),
            "vehicle_lat": vehicle_lat,
            "vehicle_lon": vehicle_lon,
            "vehicle_alt_m": vehicle_alt_m,
            "vehicle_relative_alt_m": (
                relative_alt_m
            ),
            "home_alt_m": home_alt_m,
            "vehicle_roll_deg": vehicle_roll,
            "vehicle_pitch_deg": vehicle_pitch,
            "vehicle_yaw_deg": vehicle_yaw,
            "gimbal_roll_deg": gimbal_roll,
            "gimbal_pitch_deg": gimbal_pitch,
            "gimbal_yaw_deg": gimbal_yaw,
            "gimbal_mode": gimbal_mode,
            "gimbal_frame": gimbal_frame,
            "gimbal_flags": gimbal_flags,
            "track_enabled": self.track_on_click,
            "track_status": track_status,
            "track_w": track_w,
            "track_h": track_h,
            "hfov_deg": (
                hfov
                if hfov is not None
                else self.fallback_hfov_deg
            ),
            "vfov_deg": (
                vfov
                if vfov is not None
                else self.fallback_vfov_deg
            ),
            "fov_source": (
                "gremsy"
                if (
                    hfov is not None
                    and vfov is not None
                    and fov_time > 0.0
                )
                else "fallback"
            ),
            "estimate_valid": False,
            "estimate_reason": "",
            "estimated_target_lat": None,
            "estimated_target_lon": None,
            "estimated_target_alt_m": None,
            "ground_range_m": None,
            "slant_range_m": None,
            "los_azimuth_deg": None,
            "los_down_deg": None,
            "height_agl_m": None,
            "height_source": "none",
            "track_x": None,
            "track_y": None,
            "track_pixel_source": "none",
            "gremsy_target_lat": gremsy_target_lat,
            "gremsy_target_lon": gremsy_target_lon,
            "gremsy_target_alt": gremsy_target_alt,
            "heartbeat_age_s": heartbeat_age,
            "position_age_s": position_age,
            "attitude_age_s": attitude_age,
            "gimbal_age_s": gimbal_age,
            "ardupilot_endpoint": (
                self.ardupilot_endpoint
            ),
        }

        if not ardupilot_connected:
            result["estimate_reason"] = (
                "Waiting for ArduPilot heartbeat"
            )
            return result

        if not gps_valid:
            result["estimate_reason"] = (
                "Waiting for valid vehicle GPS"
            )
            return result

        if not attitude_fresh:
            result["estimate_reason"] = (
                "Waiting for fresh vehicle ATTITUDE"
            )
            return result

        if not gimbal_fresh:
            result["estimate_reason"] = (
                "Waiting for fresh Gremsy gimbal attitude"
            )
            return result

        if not self.track_on_click:
            result["estimate_reason"] = (
                "Tracking not enabled"
            )
            return result

        # 2 means TRACK_LOST in the PayloadSdk.
        if track_status == 2:
            result["estimate_reason"] = (
                "Gremsy tracker reports LOST"
            )
            return result

        pixel_x, pixel_y, pixel_source = (
            self._choose_tracking_pixel(
                now,
                track_x,
                track_y,
                track_update,
                selected_x,
                selected_y,
                selected_update,
            )
        )

        result["track_x"] = pixel_x
        result["track_y"] = pixel_y
        result["track_pixel_source"] = (
            pixel_source
        )

        if pixel_x is None or pixel_y is None:
            result["estimate_reason"] = (
                "No tracked/selected image point"
            )
            return result

        required_values = [
            vehicle_lat,
            vehicle_lon,
            vehicle_roll,
            vehicle_pitch,
            vehicle_yaw,
            gimbal_roll,
            gimbal_pitch,
            gimbal_yaw,
        ]

        if any(
            value is None
            for value in required_values
        ):
            result["estimate_reason"] = (
                "Incomplete attitude/GPS state"
            )
            return result

        height_agl, estimated_ground_alt, height_source = (
            self._get_height_agl(
                vehicle_alt_m,
                relative_alt_m,
                home_alt_m,
            )
        )

        result["height_agl_m"] = height_agl
        result["height_source"] = (
            height_source
        )

        if height_agl is None:
            rel_text = (
                "none"
                if relative_alt_m is None
                else f"{relative_alt_m:.2f} m"
            )
            home_text = (
                "none"
                if home_alt_m is None
                else f"{home_alt_m:.2f} m"
            )
            alt_text = (
                "none"
                if vehicle_alt_m is None
                else f"{vehicle_alt_m:.2f} m"
            )

            result["estimate_reason"] = (
                "No usable camera AGL "
                f"(relative_alt={rel_text}, "
                f"vehicle_alt={alt_text}, "
                f"home_alt={home_text}). "
                "For bench tests use "
                "--camera-height-agl <meters>."
            )
            return result

        # Resolve the center optical-axis attitude.
        (
            camera_roll,
            camera_pitch,
            camera_yaw,
            frame_note,
        ) = self._resolve_camera_attitude_ned(
            float(vehicle_roll),
            float(vehicle_pitch),
            float(vehicle_yaw),
            float(gimbal_roll),
            float(gimbal_pitch),
            float(gimbal_yaw),
            int(gimbal_flags),
            str(gimbal_frame),
        )

        # FOV supplied by Gremsy is preferred because it changes with zoom.
        hfov_used = (
            float(hfov)
            if hfov is not None
            and 0.1 < hfov < 179.0
            else self.fallback_hfov_deg
        )
        vfov_used = (
            float(vfov)
            if vfov is not None
            and 0.1 < vfov < 179.0
            else self.fallback_vfov_deg
        )

        # Camera frame is FRD:
        #   +X optical axis
        #   +Y image right
        #   +Z image down
        cx = GREMSY_FRAME_W / 2.0
        cy = GREMSY_FRAME_H / 2.0

        fx = (
            GREMSY_FRAME_W / 2.0
        ) / math.tan(
            math.radians(
                hfov_used / 2.0
            )
        )
        fy = (
            GREMSY_FRAME_H / 2.0
        ) / math.tan(
            math.radians(
                vfov_used / 2.0
            )
        )

        ray_camera = [
            1.0,
            (float(pixel_x) - cx) / fx,
            (float(pixel_y) - cy) / fy,
        ]

        norm = math.sqrt(
            sum(
                component * component
                for component in ray_camera
            )
        )
        ray_camera = [
            component / norm
            for component in ray_camera
        ]

        camera_dcm = _euler_frd_to_ned(
            camera_roll,
            camera_pitch,
            camera_yaw,
        )

        ray_ned = _matvec3(
            camera_dcm,
            ray_camera,
        )

        horizontal_component = math.hypot(
            ray_ned[0],
            ray_ned[1],
        )

        los_down_deg = math.degrees(
            math.atan2(
                ray_ned[2],
                horizontal_component,
            )
        )

        los_azimuth_deg = _wrap_360(
            math.degrees(
                math.atan2(
                    ray_ned[1],
                    ray_ned[0],
                )
            )
        )

        result["los_down_deg"] = (
            los_down_deg
        )
        result["los_azimuth_deg"] = (
            los_azimuth_deg
        )
        result["camera_roll_ned_deg"] = (
            camera_roll
        )
        result["camera_pitch_ned_deg"] = (
            camera_pitch
        )
        result["camera_yaw_ned_deg"] = (
            camera_yaw
        )
        result["camera_frame_note"] = (
            frame_note
        )

        if (
            ray_ned[2] <= 0.0
            or los_down_deg
            < self.min_down_angle_deg
        ):
            result["estimate_reason"] = (
                "LOS too close to horizon or points above ground "
                f"(down angle={los_down_deg:.2f} deg)"
            )
            return result

        # Camera is h meters above the ground plane in NED, so its z
        # coordinate is -h. Ground is z=0.
        scale = (
            height_agl / ray_ned[2]
        )

        north_m = scale * ray_ned[0]
        east_m = scale * ray_ned[1]
        ground_range_m = math.hypot(
            north_m,
            east_m,
        )
        slant_range_m = scale

        if (
            ground_range_m
            > self.max_ground_range_m
        ):
            result["estimate_reason"] = (
                "Estimated range exceeds configured limit "
                f"({ground_range_m:.1f} m)"
            )
            return result

        lat_rad = math.radians(
            float(vehicle_lat)
        )

        estimated_lat = (
            float(vehicle_lat)
            + math.degrees(
                north_m / EARTH_RADIUS_M
            )
        )

        cos_lat = math.cos(lat_rad)

        if abs(cos_lat) < 1e-9:
            result["estimate_reason"] = (
                "Longitude conversion invalid near pole"
            )
            return result

        estimated_lon = (
            float(vehicle_lon)
            + math.degrees(
                east_m
                / (
                    EARTH_RADIUS_M
                    * cos_lat
                )
            )
        )

        result.update(
            {
                "estimate_valid": True,
                "estimate_reason": (
                    "Flat-ground LOS intersection"
                ),
                "estimated_target_lat": (
                    estimated_lat
                ),
                "estimated_target_lon": (
                    estimated_lon
                ),
                "estimated_target_alt_m": (
                    estimated_ground_alt
                ),
                "ground_range_m": (
                    ground_range_m
                ),
                "slant_range_m": (
                    slant_range_m
                ),
                "north_offset_m": (
                    north_m
                ),
                "east_offset_m": (
                    east_m
                ),
            }
        )

        return result

    # ------------------------------------------------------------------
    # Remote commands
    # ------------------------------------------------------------------

    def _handle_command(
        self,
        command: str,
        params: List,
    ) -> Tuple[bool, str]:
        # Status must remain queryable even while target solution is invalid.
        if command == CMD_GET_GEO_STATUS:
            return (
                True,
                json.dumps(
                    self._estimate_target_location(),
                    separators=(",", ":"),
                ),
            )

        if (
            not self.sdk
            or not self.is_connected
        ):
            return (
                False,
                "payload not connected",
            )

        try:
            if command == CMD_PAYLOAD_TRACK:
                enable = (
                    bool(int(params[0]))
                    if params
                    else False
                )

                self.track_on_click = enable

                if not enable:
                    with self._sdk_send_lock:
                        self.sdk.setPayloadObjectTrackingMode(
                            tracking_mode_t.TRACK_STOP
                        )

                    with self._state_lock:
                        self._selected_x = None
                        self._selected_y = None
                        self._last_selected_mono = 0.0

                    return (
                        True,
                        "tracking disabled; "
                        "click mode is point camera",
                    )

                return (
                    True,
                    "tracking enabled; "
                    "click a target to acquire",
                )

            if command == CMD_PAYLOAD_TOUCH:
                x = (
                    int(params[0])
                    if len(params) > 0
                    else GREMSY_FRAME_W // 2
                )
                y = (
                    int(params[1])
                    if len(params) > 1
                    else GREMSY_FRAME_H // 2
                )

                # Click came from the IR image: convert to the EO pixel
                # looking the same way, since EagleEyes works in EO frame.
                hint = ""
                if len(params) > 2 and params[2] == "ir":
                    with self._state_lock:
                        eo_h = self._hfov_deg or self.fallback_hfov_deg
                        eo_v = self._vfov_deg or self.fallback_vfov_deg
                    n = (self.ir_zoom or 0) + 1
                    ir_h = 2.0 * math.degrees(math.atan(
                        math.tan(math.radians(self.ir_hfov_deg) / 2.0) / n
                    ))
                    ir_v = 2.0 * math.degrees(math.atan(
                        math.tan(math.radians(self.ir_vfov_deg) / 2.0) / n
                    ))
                    x_ir, y_ir = x, y
                    x, y, inside = ir_px_to_eo_px(
                        x, y, ir_h, ir_v, eo_h, eo_v
                    )
                    self._log(
                        f"ir->eo remap ({x_ir},{y_ir}) -> ({x},{y}) "
                        f"ir_fov={ir_h:.1f}x{ir_v:.1f} "
                        f"eo_fov={eo_h:.1f}x{eo_v:.1f} inside={inside}"
                    )
                    if not inside:
                        hint = (
                            " (outside EO FOV; click again or zoom EO out)"
                        )

                x = max(
                    0,
                    min(
                        GREMSY_FRAME_W - 1,
                        x,
                    ),
                )
                y = max(
                    0,
                    min(
                        GREMSY_FRAME_H - 1,
                        y,
                    ),
                )

                if self.track_on_click:
                    with self._state_lock:
                        self._selected_x = float(x)
                        self._selected_y = float(y)
                        self._last_selected_mono = (
                            time.monotonic()
                        )

                    with self._sdk_send_lock:
                        self.sdk.setPayloadObjectTrackingMode(
                            tracking_mode_t.TRACK_ACTIVE
                        )
                        self.sdk.setPayloadObjectTrackingPosition(
                            x,
                            y,
                            self.track_box,
                            self.track_box,
                        )

                    return (
                        True,
                        f"tracking acquisition sent "
                        f"x={x} y={y} "
                        f"box={self.track_box}x{self.track_box}"
                        f"{hint}",
                    )

                with self._sdk_send_lock:
                    self.sdk.setPayloadObjectTrackingMode(
                        tracking_mode_t.TRACK_EAGLEEYES
                    )
                    self.sdk.setPayloadObjectTrackingPosition(
                        x,
                        y,
                        self.track_box,
                        self.track_box,
                    )

                return (
                    True,
                    f"camera point command sent "
                    f"x={x} y={y}{hint}",
                )

            if (
                command in (CMD_PAYLOAD_ZOOM_IN, CMD_PAYLOAD_ZOOM_OUT)
                and params
                and params[0] == "ir"
            ):
                # IR zoom is a discrete 1x..8x setting; one press = one step.
                step = 1 if command == CMD_PAYLOAD_ZOOM_IN else -1
                level = int(_clamp((self.ir_zoom or 0) + step, 0, 7))
                with self._sdk_send_lock:
                    self.sdk.setPayloadCameraParam(
                        PAYLOAD_CAMERA_IR_ZOOM_FACTOR,
                        level,
                        mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
                    )
                self.ir_zoom = level
                return True, f"ir zoom {level + 1}x"

            if command == CMD_PAYLOAD_ZOOM_IN:
                with self._sdk_send_lock:
                    self.sdk.setCameraZoom(
                        mavutil.mavlink.ZOOM_TYPE_CONTINUOUS,
                        camera_zoom_value.ZOOM_IN,
                    )

                return True, "zoom in started"

            if command == CMD_PAYLOAD_ZOOM_OUT:
                with self._sdk_send_lock:
                    self.sdk.setCameraZoom(
                        mavutil.mavlink.ZOOM_TYPE_CONTINUOUS,
                        camera_zoom_value.ZOOM_OUT,
                    )

                return True, "zoom out started"

            if command == CMD_PAYLOAD_ZOOM_STOP:
                with self._sdk_send_lock:
                    self.sdk.setCameraZoom(
                        mavutil.mavlink.ZOOM_TYPE_CONTINUOUS,
                        camera_zoom_value.ZOOM_STOP,
                    )

                return True, "zoom stopped"

            if command == CMD_PAYLOAD_CAMERA_PARAM:
                name = str(params[0])
                value = int(params[1])
                param_id = CAMERA_PARAMS[name]

                with self._sdk_send_lock:
                    self.sdk.setPayloadCameraParam(
                        param_id,
                        value,
                        mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
                    )

                if name == "ir_palette":
                    self.ir_palette = value

                return True, f"{name}={value}"

            if command == CMD_PAYLOAD_RECORD:
                start = int(params[0]) != 0
                with self._sdk_send_lock:
                    if start and not self._record_armed:
                        # ponytail: video mode + EO/IR-to-card set once, on
                        # first record, so a mode switch never hits the
                        # stream mid-session. Stream source (C_SOURCE)
                        # is untouched.
                        self.sdk.setPayloadCameraMode(
                            mavutil.mavlink.CAMERA_MODE_VIDEO
                        )
                        self.sdk.setPayloadCameraParam(
                            PAYLOAD_CAMERA_RECORD_SRC,
                            payload_camera_record_src.PAYLOAD_CAMERA_RECORD_BOTH,
                            mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
                        )
                        self._record_armed = True
                    if start:
                        self.sdk.setPayloadCameraRecordVideoStart()
                    else:
                        self.sdk.setPayloadCameraRecordVideoStop()
                    # Camera answers with CAMERA_CAPTURE_STATUS -> self.recording
                    self.sdk.getPayloadCaptureStatus()

                self.recording = start
                return True, "recording started" if start else "recording stopped"

            if command == CMD_PAYLOAD_GIMBAL_MODE:
                mode = int(params[0])
                with self._state_lock:
                    self._hold_on = False
                with self._sdk_send_lock:
                    self.sdk.setPayloadCameraParam(
                        PAYLOAD_CAMERA_GIMBAL_MODE,
                        mode,
                        mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
                    )
                return True, f"gimbal mode {mode}"

            if command == CMD_PAYLOAD_GIMBAL_LEVEL_ROLL:
                with self._state_lock:
                    pitch = self._gimbal_pitch_deg
                    yaw = self._gimbal_yaw_deg
                if pitch is None or yaw is None:
                    return False, "no gimbal attitude yet"
                # ponytail: only helps when the gimbal *reports* the roll
                # (commanded/mechanical). IMU horizon drift reads ~0 and
                # needs gyro calib / vehicle attitude feed instead.
                with self._sdk_send_lock:
                    self.sdk.setGimbalSpeed(
                        pitch, 0.0, yaw, input_mode_t.INPUT_ANGLE
                    )
                return True, f"roll->0 (pitch={pitch:.1f} yaw={yaw:.1f})"

            if command == CMD_PAYLOAD_GIMBAL_YAW_HEADING:
                heading = float(params[0])
                with self._state_lock:
                    self._hold_heading_deg = heading
                return self._yaw_to_heading(heading)

            if command == CMD_PAYLOAD_GIMBAL_HEADING_HOLD:
                on = bool(int(params[0]))
                with self._state_lock:
                    self._hold_on = on
                    self._hold_last_cmd = None
                    heading = self._hold_heading_deg
                if not on:
                    return True, "heading hold off"
                # Follow mode: gimbal yaw is nose-relative, so the hold loop
                # (driven by autopilot heading) is the thing keeping it on
                # heading. In lock mode the gimbal's own IMU north wins and
                # the executor cannot correct it.
                with self._sdk_send_lock:
                    self.sdk.setPayloadCameraParam(
                        PAYLOAD_CAMERA_GIMBAL_MODE,
                        2,
                        mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
                    )
                if heading is None:
                    return True, "heading hold on, click a heading"
                return self._yaw_to_heading(heading)

            return (
                False,
                f"unsupported command: {command}",
            )

        except Exception as exc:
            return (
                False,
                f"execution error: {exc}",
            )

    def _yaw_to_heading(
        self, heading: float, only_if_changed: bool = False
    ) -> Tuple[bool, str]:
        """Yaw the gimbal to a compass heading, keeping pitch, roll=0.

        only_if_changed: skip the send unless the setpoint moved >2 deg
        since the last one (the heading-hold loop calls this on every
        ATTITUDE message).
        """
        with self._state_lock:
            pitch = self._gimbal_pitch_deg
            cur_yaw = self._gimbal_yaw_deg
            frame = self._gimbal_frame
            vehicle_yaw = self._vehicle_yaw_deg
            last_cmd = self._hold_last_cmd
        if pitch is None or cur_yaw is None:
            return False, "no gimbal attitude yet"
        # Vehicle yaw is needed in every frame: the +-limit is on
        # the yaw relative to the nose.
        if vehicle_yaw is None:
            return False, "no vehicle yaw yet"
        yaw, body, clamped = heading_to_gimbal_yaw(
            heading, vehicle_yaw, self.camera_yaw_offset_deg, frame
        )
        if (
            only_if_changed
            and last_cmd is not None
            and abs(_wrap_180(yaw - last_cmd)) < 2.0
        ):
            return True, "unchanged"
        with self._state_lock:
            self._hold_last_cmd = yaw
            self._yaw_seq += 1
            seq = self._yaw_seq
        cur_body = _wrap_180(
            cur_yaw - (vehicle_yaw if frame == "earth" else 0.0)
        )
        path = yaw_path(cur_body, body)
        first = _body_to_cmd(path[0], vehicle_yaw, frame)
        with self._sdk_send_lock:
            self.sdk.setGimbalSpeed(
                pitch, 0.0, first, input_mode_t.INPUT_ANGLE
            )
        if len(path) > 1:
            threading.Thread(
                target=self._finish_yaw_via_nose,
                args=(seq, pitch, body),
                daemon=True,
            ).start()
        note = (
            f", clamped to {body:+.0f} of nose" if clamped else ""
        ) + (", via nose" if len(path) > 1 else "")
        return True, f"heading {heading:.0f} -> yaw {yaw:.1f} ({frame}{note})"

    def _finish_yaw_via_nose(
        self, seq: int, pitch: float, tgt_body: float
    ) -> None:
        """Second leg of a compass click that had to go round the front."""
        # ponytail: poll reported yaw; 6 s covers 170 deg at slow gimbal rates.
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            time.sleep(0.1)
            with self._state_lock:
                if seq != self._yaw_seq:
                    return  # newer click took over
                gy = self._gimbal_yaw_deg
                vy = self._vehicle_yaw_deg
                frame = self._gimbal_frame
            if gy is None or vy is None:
                continue
            cur_body = _wrap_180(gy - (vy if frame == "earth" else 0.0))
            if abs(cur_body) < 15.0:
                break
        else:
            self._log("yaw via nose: timed out waiting at nose, sending anyway")
        with self._state_lock:
            if seq != self._yaw_seq:
                return
            vy = self._vehicle_yaw_deg
            frame = self._gimbal_frame
        cmd = _body_to_cmd(tgt_body, vy or 0.0, frame)
        with self._sdk_send_lock:
            self.sdk.setGimbalSpeed(pitch, 0.0, cmd, input_mode_t.INPUT_ANGLE)
        self._log(f"yaw via nose: final body {tgt_body:+.0f} -> cmd {cmd:.1f}")

    @staticmethod
    def _log(message: str) -> None:
        print(
            f"[REMOTE_EXECUTOR] {message}"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Gremsy Payload SDK Remote Executor "
            "with target geolocation estimator"
        )
    )

    parser.add_argument(
        "--role",
        choices=["connect", "listen"],
        default="listen",
    )
    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="Bind/connect host for bridge",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=5000,
        help="Bind/connect port for bridge",
    )
    parser.add_argument(
        "--token",
        default="",
        help="Optional shared token",
    )
    parser.add_argument(
        "--ack-timeout",
        type=float,
        default=1.5,
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=2,
    )
    parser.add_argument(
        "--payload-ip",
        default=ConnectionConfig.UDP_IP_TARGET,
    )

    parser.add_argument(
        "--ardupilot-endpoint",
        default=DEFAULT_ARDUPILOT_ENDPOINT,
        help=(
            "pymavlink endpoint used for vehicle GPS/ATTITUDE. "
            "Use 'none' to disable."
        ),
    )
    parser.add_argument(
        "--ardupilot-sysid",
        type=int,
        default=0,
        help=(
            "Expected ArduPilot SYSID; "
            "0 auto-detects the first autopilot heartbeat"
        ),
    )
    parser.add_argument(
        "--ardupilot-stale-timeout",
        type=float,
        default=3.0,
    )
    parser.add_argument(
        "--position-stale-timeout",
        type=float,
        default=2.0,
    )
    parser.add_argument(
        "--attitude-stale-timeout",
        type=float,
        default=1.0,
    )
    parser.add_argument(
        "--gimbal-stale-timeout",
        type=float,
        default=1.0,
    )
    parser.add_argument(
        "--track-stale-timeout",
        type=float,
        default=1.0,
    )

    parser.add_argument(
        "--track-box",
        type=int,
        default=DEFAULT_TRACK_BOX,
    )

    parser.add_argument(
        "--camera-height-agl",
        type=float,
        default=0.0,
        help=(
            "Override camera height above target ground plane in meters. "
            "Recommended for ground tests. 0 = use relative_alt, then MSL-home fallback."
        ),
    )
    parser.add_argument(
        "--ground-alt-m",
        type=float,
        default=None,
        help=(
            "Optional fixed ground altitude in meters AMSL. "
            "If supplied, height AGL = vehicle AMSL - this value."
        ),
    )
    parser.add_argument(
        "--min-down-angle-deg",
        type=float,
        default=5.0,
        help=(
            "Reject LOS solutions closer to the horizon than this."
        ),
    )
    parser.add_argument(
        "--max-ground-range-m",
        type=float,
        default=5000.0,
    )

    parser.add_argument(
        "--fallback-hfov-deg",
        type=float,
        default=60.4,
        help=(
            "Used only until Lynx CAMERA_FOV_STATUS is received."
        ),
    )
    parser.add_argument(
        "--fallback-vfov-deg",
        type=float,
        default=36.255,
        help=(
            "Used only until Lynx CAMERA_FOV_STATUS is received."
        ),
    )
    # ponytail: placeholder IR FOV, calibrate on hardware with
    # examples/payload_get_fov_status.py at IR 1x and update these defaults.
    parser.add_argument(
        "--ir-hfov-deg",
        type=float,
        default=32.0,
        help="Native (1x) IR horizontal FOV; used to remap IR clicks.",
    )
    parser.add_argument(
        "--ir-vfov-deg",
        type=float,
        default=26.0,
        help="Native (1x) IR vertical FOV; used to remap IR clicks.",
    )

    parser.add_argument(
        "--camera-roll-offset-deg",
        type=float,
        default=0.0,
        help="Calibration offset applied to LOS roll.",
    )
    parser.add_argument(
        "--camera-pitch-offset-deg",
        type=float,
        default=0.0,
        help="Calibration offset applied to LOS pitch.",
    )
    parser.add_argument(
        "--camera-yaw-offset-deg",
        type=float,
        default=0.0,
        help="Calibration offset applied to LOS yaw.",
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    app = RemoteExecutor(
        role=args.role,
        host=args.host,
        port=args.port,
        token=args.token,
        ack_timeout=args.ack_timeout,
        retries=args.retries,
        payload_ip=args.payload_ip,
        ardupilot_endpoint=args.ardupilot_endpoint,
        ardupilot_sysid=args.ardupilot_sysid,
        ardupilot_stale_timeout=(
            args.ardupilot_stale_timeout
        ),
        position_stale_timeout=(
            args.position_stale_timeout
        ),
        attitude_stale_timeout=(
            args.attitude_stale_timeout
        ),
        gimbal_stale_timeout=(
            args.gimbal_stale_timeout
        ),
        track_stale_timeout=(
            args.track_stale_timeout
        ),
        track_box=args.track_box,
        camera_height_agl=(
            args.camera_height_agl
        ),
        ground_alt_m=args.ground_alt_m,
        min_down_angle_deg=(
            args.min_down_angle_deg
        ),
        max_ground_range_m=(
            args.max_ground_range_m
        ),
        fallback_hfov_deg=(
            args.fallback_hfov_deg
        ),
        fallback_vfov_deg=(
            args.fallback_vfov_deg
        ),
        ir_hfov_deg=args.ir_hfov_deg,
        ir_vfov_deg=args.ir_vfov_deg,
        camera_roll_offset_deg=(
            args.camera_roll_offset_deg
        ),
        camera_pitch_offset_deg=(
            args.camera_pitch_offset_deg
        ),
        camera_yaw_offset_deg=(
            args.camera_yaw_offset_deg
        ),
    )

    return app.start()


if __name__ == "__main__":
    raise SystemExit(main())
