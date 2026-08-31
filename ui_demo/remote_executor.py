#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""
Remote executor service.

Runs on the machine physically connected to the Gremsy payload/gimbal.
Receives remote UI commands over TCP and executes a safe command subset.

Optional ArduPilot geolocation path
-----------------------------------
- A background pymavlink connection listens for ArduPilot telemetry.
- GLOBAL_POSITION_INT and GPS_RAW_INT are forwarded into the Gremsy payload.
- Gremsy TARGET_LAT/TARGET_LON/TARGET_ALT parameters are monitored.
- The UI polls GET_GEO_STATUS; lack of ArduPilot never blocks gimbal control.

Click behavior
--------------
- PAYLOAD_TRACK [0] -> point-camera mode.
- PAYLOAD_TRACK [1] -> click-to-track mode.
- PAYLOAD_TOUCH [x, y]
    * point mode: TRACK_EAGLEEYES (move only)
    * track mode: TRACK_ACTIVE + acquire a fixed box around x/y

Zoom behavior
-------------
- PAYLOAD_ZOOM_IN -> begin continuous zoom in
- PAYLOAD_ZOOM_OUT -> begin continuous zoom out
- PAYLOAD_ZOOM_STOP -> stop continuous zoom
"""

import argparse
import json
import math
import os
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

# Add libs path.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "libs"))

from remote_bridge import BridgeConfig, TcpCommandBridge

try:
    from pymavlink import mavutil
    from payload_sdk import (
        PayloadSdkInterface,
        mavlink_global_position_int_t,
        mavlink_gps_raw_int_t,
        payload_param_t,
        payload_status_event_t,
        tracking_mode_t,
    )
    from config import ConnectionConfig
    from payload_define import camera_zoom_value
except ImportError as exc:
    print(f"Error importing payload SDK modules: {exc}")
    sys.exit(1)


CMD_PAYLOAD_TOUCH = "PAYLOAD_TOUCH"
CMD_PAYLOAD_TRACK = "PAYLOAD_TRACK"
CMD_GET_GEO_STATUS = "GET_GEO_STATUS"
CMD_PAYLOAD_ZOOM_IN = "PAYLOAD_ZOOM_IN"
CMD_PAYLOAD_ZOOM_OUT = "PAYLOAD_ZOOM_OUT"
CMD_PAYLOAD_ZOOM_STOP = "PAYLOAD_ZOOM_STOP"

GREMSY_FRAME_W = 1920
GREMSY_FRAME_H = 1080
DEFAULT_TRACK_BOX = 128

DEFAULT_ARDUPILOT_ENDPOINT = os.environ.get(
    "ARDUPILOT_ENDPOINT",
    "udpin:0.0.0.0:14555",
)


class RemoteExecutor:
    """Execute remote UI commands on the local Gremsy payload SDK."""

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
        track_box: int,
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

        # Operator mode: False = point only, True = click-to-track.
        self.track_on_click = False
        self.track_box = max(16, min(512, int(track_box)))

        # Serialize outgoing calls into the Gremsy SDK. The SDK has its own
        # receive thread, but our command thread and ArduPilot forwarding
        # thread can otherwise transmit at the same time.
        self._sdk_send_lock = threading.Lock()

        # ------------------------- ArduPilot state ----------------------
        self.ardupilot_endpoint = (ardupilot_endpoint or "").strip()
        self.ardupilot_sysid = int(ardupilot_sysid)
        self.ardupilot_stale_timeout = max(0.5, float(ardupilot_stale_timeout))
        self.position_stale_timeout = max(0.5, float(position_stale_timeout))

        self._ap_master = None
        self._ap_thread: Optional[threading.Thread] = None
        self._ap_target_sysid: Optional[int] = (
            self.ardupilot_sysid if self.ardupilot_sysid > 0 else None
        )

        self._state_lock = threading.Lock()
        self._last_ap_heartbeat_mono = 0.0
        self._last_ap_position_mono = 0.0
        self._last_ap_message_mono = 0.0
        self._gps_raw_seen = False
        self._gps_fix_type = 0
        self._last_vehicle_lat: Optional[float] = None
        self._last_vehicle_lon: Optional[float] = None
        self._last_vehicle_alt_m: Optional[float] = None

        # Gremsy's target geolocation estimate.
        self._target_lat: Optional[float] = None
        self._target_lon: Optional[float] = None
        self._target_alt: Optional[float] = None
        self._target_update_mono = 0.0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> int:
        """Start payload SDK, optional ArduPilot listener, and TCP bridge."""
        ConnectionConfig.UDP_IP_TARGET = self.payload_ip
        self.sdk = PayloadSdkInterface()

        if not self.sdk.sdkInitConnection():
            self._log("payload SDK init failed")
            return 1

        if not self._wait_payload_connection(timeout=8.0):
            self._log("payload connection timeout")
            self.stop()
            return 1

        self._configure_payload_geolocation()
        self._start_ardupilot_worker()

        self.bridge.start()
        self._log("executor ready, waiting for commands")

        try:
            self.bridge.serve_commands(self._handle_command)
        except KeyboardInterrupt:
            self._log("executor interrupted")
        finally:
            self.stop()

        return 0

    def stop(self) -> None:
        """Stop executor and all background connections."""
        if not self.running:
            return

        self.running = False
        self.bridge.stop()

        ap_master = self._ap_master
        self._ap_master = None
        if ap_master is not None:
            try:
                ap_master.close()
            except Exception:
                pass

        if self._ap_thread and self._ap_thread.is_alive():
            self._ap_thread.join(timeout=2.0)
        self._ap_thread = None

        if self.sdk:
            try:
                if self.is_connected:
                    try:
                        with self._sdk_send_lock:
                            self.sdk.setCameraZoom(
                                mavutil.mavlink.ZOOM_TYPE_CONTINUOUS,
                                camera_zoom_value.ZOOM_STOP,
                            )
                    except Exception as exc:
                        self._log(
                            f"failed stopping zoom during shutdown: {exc}"
                        )

                self.sdk.sdkQuit()
            finally:
                self.sdk = None

        self._log("executor stopped")

    def _wait_payload_connection(self, timeout: float) -> bool:
        start = time.time()
        while self.running and (time.time() - start) < timeout:
            if self.sdk and self.sdk.checkPayloadConnection():
                self.is_connected = True
                self._log(f"payload connected at {self.payload_ip}")
                return True
            time.sleep(0.1)
        return False

    # ------------------------------------------------------------------
    # Gremsy geolocation output
    # ------------------------------------------------------------------

    def _configure_payload_geolocation(self) -> None:
        """Subscribe to Gremsy's estimated target coordinates."""
        if self.sdk is None:
            return

        self.sdk.regPayloadStatusChanged(self._on_payload_status)

        # Gremsy API uses milliseconds for these requested parameter rates.
        # 500 ms = 2 Hz, which is plenty for the UI display.
        try:
            with self._sdk_send_lock:
                self.sdk.setParamRate(payload_param_t.PARAM_TARGET_COOR_LAT, 500)
                self.sdk.setParamRate(payload_param_t.PARAM_TARGET_COOR_LON, 500)
                self.sdk.setParamRate(payload_param_t.PARAM_TARGET_COOR_ALT, 500)
        except Exception as exc:
            self._log(f"unable to request target geolocation stream: {exc}")

    def _on_payload_status(self, event, param: List[float]) -> None:
        """Receive TARGET_LAT/LON/ALT updates from the Gremsy payload."""
        try:
            if int(event) != int(payload_status_event_t.PAYLOAD_PARAMS):
                return
            if len(param) < 2:
                return

            index = int(param[0])
            value = float(param[1])
            if not math.isfinite(value):
                return

            now = time.monotonic()

            with self._state_lock:
                if index == int(payload_param_t.PARAM_TARGET_COOR_LAT):
                    self._target_lat = value
                    self._target_update_mono = now
                elif index == int(payload_param_t.PARAM_TARGET_COOR_LON):
                    self._target_lon = value
                    self._target_update_mono = now
                elif index == int(payload_param_t.PARAM_TARGET_COOR_ALT):
                    self._target_alt = value
                    self._target_update_mono = now
        except Exception as exc:
            self._log(f"payload status callback error: {exc}")

    # ------------------------------------------------------------------
    # ArduPilot MAVLink input
    # ------------------------------------------------------------------

    def _start_ardupilot_worker(self) -> None:
        if not self.ardupilot_endpoint or self.ardupilot_endpoint.lower() in {
            "none",
            "off",
            "disabled",
        }:
            self._log("ArduPilot geolocation input disabled")
            return

        self._ap_thread = threading.Thread(
            target=self._ardupilot_loop,
            daemon=True,
            name="ardupilot-geolocation",
        )
        self._ap_thread.start()

    def _ardupilot_loop(self) -> None:
        """
        Continuously listen for ArduPilot MAVLink.

        Failure to connect is intentionally non-fatal. The gimbal executor and
        remote bridge remain fully operational and this loop keeps retrying so
        geolocation can recover later.
        """
        while self.running:
            master = None
            try:
                self._log(
                    f"opening optional ArduPilot MAVLink endpoint "
                    f"{self.ardupilot_endpoint}"
                )

                master = mavutil.mavlink_connection(
                    self.ardupilot_endpoint,
                    source_system=255,
                    source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
                    autoreconnect=True,
                )
                self._ap_master = master

                while self.running:
                    msg = master.recv_match(blocking=True, timeout=0.5)
                    if msg is None:
                        continue

                    msg_type = msg.get_type()
                    if msg_type == "BAD_DATA":
                        continue

                    src_sysid = int(msg.get_srcSystem())
                    if not self._accept_ardupilot_system(msg, src_sysid):
                        continue

                    now = time.monotonic()
                    with self._state_lock:
                        self._last_ap_message_mono = now

                    if msg_type == "HEARTBEAT":
                        with self._state_lock:
                            self._last_ap_heartbeat_mono = now
                        continue

                    if msg_type == "GPS_RAW_INT":
                        self._handle_ardupilot_gps_raw(msg, now)
                        continue

                    if msg_type == "GLOBAL_POSITION_INT":
                        self._handle_ardupilot_global_position(msg, now)
                        continue

            except Exception as exc:
                if self.running:
                    self._log(f"ArduPilot MAVLink error: {exc}")
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

    def _accept_ardupilot_system(self, msg, src_sysid: int) -> bool:
        """Select one vehicle system and ignore unrelated MAVLink systems."""
        if self.ardupilot_sysid > 0:
            return src_sysid == self.ardupilot_sysid

        if self._ap_target_sysid is not None:
            return src_sysid == self._ap_target_sysid

        if msg.get_type() == "HEARTBEAT":
            autopilot = int(
                getattr(msg, "autopilot", mavutil.mavlink.MAV_AUTOPILOT_INVALID)
            )
            src_component = int(msg.get_srcComponent())

            if (
                autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID
                or src_component == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
            ):
                self._ap_target_sysid = src_sysid
                self._log(f"selected ArduPilot MAVLink sysid={src_sysid}")
                return True

        # GLOBAL_POSITION_INT can be useful even if the heartbeat was lost.
        # Wait for a heartbeat before locking onto an unknown system so that
        # another MAVLink component is not accidentally selected.
        return False

    def _handle_ardupilot_global_position(self, msg, now: float) -> None:
        lat_deg = float(msg.lat) / 1.0e7
        lon_deg = float(msg.lon) / 1.0e7
        alt_m = float(msg.alt) / 1000.0

        with self._state_lock:
            self._last_ap_position_mono = now
            self._last_vehicle_lat = lat_deg
            self._last_vehicle_lon = lon_deg
            self._last_vehicle_alt_m = alt_m

        if self.sdk is None or not self.is_connected:
            return

        gps = mavlink_global_position_int_t()
        gps.time_boot_ms = int(msg.time_boot_ms)
        gps.lat = int(msg.lat)
        gps.lon = int(msg.lon)
        gps.alt = int(msg.alt)
        gps.relative_alt = int(msg.relative_alt)
        gps.vx = int(msg.vx)
        gps.vy = int(msg.vy)
        gps.vz = int(msg.vz)
        gps.hdg = int(msg.hdg)

        try:
            with self._sdk_send_lock:
                self.sdk.sendPayloadGPSPosition(gps)
        except Exception as exc:
            self._log(f"failed forwarding GLOBAL_POSITION_INT to payload: {exc}")

    def _handle_ardupilot_gps_raw(self, msg, now: float) -> None:
        fix_type = int(getattr(msg, "fix_type", 0))

        with self._state_lock:
            self._gps_raw_seen = True
            self._gps_fix_type = fix_type

        if self.sdk is None or not self.is_connected:
            return

        gps = mavlink_gps_raw_int_t()
        gps.time_usec = int(getattr(msg, "time_usec", 0))
        gps.lat = int(getattr(msg, "lat", 0))
        gps.lon = int(getattr(msg, "lon", 0))
        gps.alt = int(getattr(msg, "alt", 0))
        gps.eph = int(getattr(msg, "eph", 65535))
        gps.epv = int(getattr(msg, "epv", 65535))
        gps.vel = int(getattr(msg, "vel", 65535))
        gps.cog = int(getattr(msg, "cog", 65535))
        gps.fix_type = fix_type
        gps.satellites_visible = int(getattr(msg, "satellites_visible", 255))
        gps.alt_ellipsoid = int(getattr(msg, "alt_ellipsoid", 0))
        gps.h_acc = int(getattr(msg, "h_acc", 0))
        gps.v_acc = int(getattr(msg, "v_acc", 0))
        gps.vel_acc = int(getattr(msg, "vel_acc", 0))
        gps.hdg_acc = int(getattr(msg, "hdg_acc", 0))
        gps.yaw = int(getattr(msg, "yaw", 0))

        try:
            with self._sdk_send_lock:
                self.sdk.sendPayloadGPSRawInt(gps)
        except Exception as exc:
            self._log(f"failed forwarding GPS_RAW_INT to payload: {exc}")

    # ------------------------------------------------------------------
    # Status snapshot requested by UI
    # ------------------------------------------------------------------

    def _geo_status(self) -> Dict:
        now = time.monotonic()

        with self._state_lock:
            hb_time = self._last_ap_heartbeat_mono
            pos_time = self._last_ap_position_mono
            gps_raw_seen = self._gps_raw_seen
            fix_type = self._gps_fix_type

            vehicle_lat = self._last_vehicle_lat
            vehicle_lon = self._last_vehicle_lon
            vehicle_alt = self._last_vehicle_alt_m

            target_lat = self._target_lat
            target_lon = self._target_lon
            target_alt = self._target_alt
            target_update = self._target_update_mono

        heartbeat_age = None if hb_time <= 0.0 else max(0.0, now - hb_time)
        position_age = None if pos_time <= 0.0 else max(0.0, now - pos_time)
        target_age = None if target_update <= 0.0 else max(0.0, now - target_update)

        ardupilot_connected = (
            heartbeat_age is not None
            and heartbeat_age <= self.ardupilot_stale_timeout
        )

        position_fresh = (
            position_age is not None
            and position_age <= self.position_stale_timeout
        )

        # If GPS_RAW_INT is present, use its actual fix type. If the stream
        # doesn't include GPS_RAW_INT, a fresh non-zero GLOBAL_POSITION_INT is
        # accepted as the fallback indication that vehicle position is usable.
        if gps_raw_seen:
            gps_valid = ardupilot_connected and position_fresh and fix_type >= 3
        else:
            gps_valid = (
                ardupilot_connected
                and position_fresh
                and vehicle_lat is not None
                and vehicle_lon is not None
                and not (abs(vehicle_lat) < 1e-12 and abs(vehicle_lon) < 1e-12)
            )

        geolocation_available = ardupilot_connected and gps_valid

        target_valid = (
            geolocation_available
            and target_lat is not None
            and target_lon is not None
            and math.isfinite(target_lat)
            and math.isfinite(target_lon)
            and not (abs(target_lat) < 1e-12 and abs(target_lon) < 1e-12)
        )

        if not self.ardupilot_endpoint or self.ardupilot_endpoint.lower() in {
            "none",
            "off",
            "disabled",
        }:
            detail = "ArduPilot input disabled"
        elif not ardupilot_connected:
            detail = f"Waiting for ArduPilot on {self.ardupilot_endpoint}"
        elif not gps_valid:
            detail = "ArduPilot connected; waiting for valid GPS position"
        elif not target_valid:
            detail = "Vehicle GPS available; waiting for Gremsy target location"
        else:
            detail = "Target geolocation available"

        return {
            "ardupilot_connected": ardupilot_connected,
            "ardupilot_sysid": self._ap_target_sysid,
            "gps_valid": gps_valid,
            "gps_fix_type": fix_type if gps_raw_seen else None,
            "geolocation_available": geolocation_available,
            "target_valid": target_valid,
            "vehicle_lat": vehicle_lat,
            "vehicle_lon": vehicle_lon,
            "vehicle_alt": vehicle_alt,
            "target_lat": target_lat if target_valid else None,
            "target_lon": target_lon if target_valid else None,
            "target_alt": target_alt if target_valid else None,
            "heartbeat_age_s": heartbeat_age,
            "position_age_s": position_age,
            "target_age_s": target_age,
            "ardupilot_endpoint": self.ardupilot_endpoint,
            "detail": detail,
        }

    # ------------------------------------------------------------------
    # Remote UI commands
    # ------------------------------------------------------------------

    def _handle_command(self, command: str, params: List) -> Tuple[bool, str]:
        # Status must remain queryable independently of tracking state.
        if command == CMD_GET_GEO_STATUS:
            return True, json.dumps(self._geo_status(), separators=(",", ":"))

        if not self.sdk or not self.is_connected:
            return False, "payload not connected"

        try:
            if command == CMD_PAYLOAD_TRACK:
                enable = bool(int(params[0])) if params else False
                self.track_on_click = enable

                # Turning tracking off immediately stops an existing track.
                # Turning it on only arms the UI mode; TRACK_ACTIVE is sent on
                # the next click so the selected image point is the trigger.
                if not enable:
                    with self._sdk_send_lock:
                        self.sdk.setPayloadObjectTrackingMode(
                            tracking_mode_t.TRACK_STOP
                        )
                    return True, "tracking disabled; click mode is point camera"

                return True, "tracking enabled; click a target to acquire"

            if command == CMD_PAYLOAD_TOUCH:
                x = int(params[0]) if len(params) > 0 else GREMSY_FRAME_W // 2
                y = int(params[1]) if len(params) > 1 else GREMSY_FRAME_H // 2

                x = max(0, min(GREMSY_FRAME_W - 1, x))
                y = max(0, min(GREMSY_FRAME_H - 1, y))

                if self.track_on_click:
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

                    return True, (
                        f"tracking acquisition sent x={x} y={y} "
                        f"box={self.track_box}x{self.track_box}"
                    )

                # EagleEyes: move gimbal to the selected image point without
                # triggering object tracking.
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

                return True, f"camera point command sent x={x} y={y}"

            if command == CMD_PAYLOAD_ZOOM_IN:
                with self._sdk_send_lock:
                    self.sdk.setCameraZoom(
                        mavutil.mavlink.ZOOM_TYPE_CONTINUOUS,
                        camera_zoom_value.ZOOM_IN,
                    )

                return True, "continuous zoom in started"

            if command == CMD_PAYLOAD_ZOOM_OUT:
                with self._sdk_send_lock:
                    self.sdk.setCameraZoom(
                        mavutil.mavlink.ZOOM_TYPE_CONTINUOUS,
                        camera_zoom_value.ZOOM_OUT,
                    )

                return True, "continuous zoom out started"

            if command == CMD_PAYLOAD_ZOOM_STOP:
                with self._sdk_send_lock:
                    self.sdk.setCameraZoom(
                        mavutil.mavlink.ZOOM_TYPE_CONTINUOUS,
                        camera_zoom_value.ZOOM_STOP,
                    )

                return True, "zoom stopped"

            return False, f"unsupported command: {command}"

        except Exception as exc:
            return False, f"execution error: {exc}"

    @staticmethod
    def _log(message: str) -> None:
        print(f"[REMOTE_EXECUTOR] {message}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Payload SDK Remote Executor"
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
        help="ACK timeout seconds",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=2,
        help="Retry count",
    )
    parser.add_argument(
        "--payload-ip",
        default=ConnectionConfig.UDP_IP_TARGET,
        help="Payload/gimbal IP",
    )
    parser.add_argument(
        "--ardupilot-endpoint",
        default=DEFAULT_ARDUPILOT_ENDPOINT,
        help=(
            "Optional pymavlink endpoint. Default: "
            f"{DEFAULT_ARDUPILOT_ENDPOINT}. Use 'none' to disable. "
            "A dedicated mavlink-router UDP output is recommended."
        ),
    )
    parser.add_argument(
        "--ardupilot-sysid",
        type=int,
        default=0,
        help="Expected ArduPilot SYSID; 0 = detect first autopilot heartbeat",
    )
    parser.add_argument(
        "--ardupilot-stale-timeout",
        type=float,
        default=3.0,
        help="Seconds without heartbeat before ArduPilot is considered offline",
    )
    parser.add_argument(
        "--position-stale-timeout",
        type=float,
        default=2.0,
        help="Seconds before GLOBAL_POSITION_INT is considered stale",
    )
    parser.add_argument(
        "--track-box",
        type=int,
        default=DEFAULT_TRACK_BOX,
        help="Gremsy acquisition box size centered on clicked pixel",
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
        ardupilot_stale_timeout=args.ardupilot_stale_timeout,
        position_stale_timeout=args.position_stale_timeout,
        track_box=args.track_box,
    )
    return app.start()


if __name__ == "__main__":
    raise SystemExit(main())
