#!/usr/bin/env python3
"""night_track_test.py: does the payload's tracker drive the gimbal in IR view?

Bare reproduction outside remote_executor.py, same calls as Gremsy's
examples/payload_do_object_tracking.py but with a chosen view source and
gimbal mode. Stop gremsy.service first. Run from the repo root on the Pi:

    python3 useful_scripts/night_track_test.py ir 1            # IR view, LOCK
    python3 useful_scripts/night_track_test.py ir 2            # IR view, FOLLOW
    python3 useful_scripts/night_track_test.py eo 1            # control: should move
    python3 useful_scripts/night_track_test.py ir 1 1200 540   # pixel to track
    python3 useful_scripts/night_track_test.py ir 2 drive      # steer the gimbal from here

`drive`: the firmware tracks in IR view but does not steer the gimbal
(2026-09-19), so close the loop here: tracker box offset from frame centre ->
angle (IR geometry) -> gimbal rate. Success = pos settles near (896,476),
i.e. box centre at (960,540), and status stays 1.

Prints tracker status/position and gimbal attitude for 10 s, then the total
gimbal movement, releases the tracker and leaves the view source as set.
"""
import math
import os
import sys
import time

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "libs"))

from pymavlink import mavutil  # noqa: E402
from payload_sdk import (  # noqa: E402
    PayloadSdkInterface,
    input_mode_t,
    payload_param_t,
    payload_status_event_t,
    tracking_mode_t,
)

VIEW = {"eo": 1, "ir": 2}
U32 = mavutil.mavlink.MAV_PARAM_TYPE_UINT32
TRACK_PARAMS = {
    int(payload_param_t.PARAM_TRACK_POS_X): "x",
    int(payload_param_t.PARAM_TRACK_POS_Y): "y",
    int(payload_param_t.PARAM_TRACK_STATUS): "status",
}

# Pixels per tan(angle) of the 1080-high frame in IR view at IR 1x (VFOV 18.2).
IR_F_PX = 540 / math.tan(math.radians(18.24) / 2)
DRIVE_GAIN = 2.0       # deg/s per deg of error; feedback is slow, keep it low
DRIVE_MAX_DPS = 30.0
DRIVE_MIN_DPS = 3.0    # below this the motor does not move
DRIVE_DEADBAND_DEG = 0.5
YAW_RATE_SCALE = 6.0   # same per-axis calibration as remote_executor.py
PITCH_RATE_SCALE = 1.0

track = {}
att = {}


def drive_rate(err_deg: float) -> float:
    if abs(err_deg) < DRIVE_DEADBAND_DEG:
        return 0.0
    return math.copysign(
        min(DRIVE_MAX_DPS, max(DRIVE_MIN_DPS, DRIVE_GAIN * abs(err_deg))), err_deg
    )


def drive_step(sdk) -> None:
    yaw_dps = pitch_dps = 0.0
    if (track.get("status") or 0) & 0xFF == 1 and "x" in track and "y" in track:
        ex = track["x"] + 64 - 960   # pos is the box's top-left corner
        ey = track["y"] + 64 - 540
        yaw_dps = drive_rate(math.degrees(math.atan(ex / IR_F_PX)))
        pitch_dps = drive_rate(-math.degrees(math.atan(ey / IR_F_PX)))
    sdk.setGimbalSpeed(
        PITCH_RATE_SCALE * pitch_dps, 0.0, YAW_RATE_SCALE * yaw_dps,
        input_mode_t.INPUT_SPEED,
    )


def on_status(event, params):
    # Attitude arrives here (MOUNT_ORIENTATION) or in on_param
    # (GIMBAL_DEVICE_ATTITUDE_STATUS), depending on what the payload streams.
    if int(event) == int(payload_status_event_t.PAYLOAD_GB_ATTITUDE) and len(params) >= 3:
        att["pitch"], att["yaw"] = float(params[0]), float(params[2])
    elif int(event) == int(payload_status_event_t.PAYLOAD_PARAMS):
        name = TRACK_PARAMS.get(int(params[0]))
        if name:
            track[name] = int(params[1])


def on_param(event, param_id, params):
    if int(event) == int(payload_status_event_t.PAYLOAD_GB_ATTITUDE) and len(params) >= 3:
        att["pitch"], att["yaw"] = float(params[0]), float(params[2])


def main() -> None:
    drive = "drive" in sys.argv
    if drive:
        sys.argv.remove("drive")
    if len(sys.argv) not in (3, 5) or sys.argv[1] not in VIEW:
        sys.exit(__doc__)
    view, gb_mode = VIEW[sys.argv[1]], int(sys.argv[2])
    x, y = (int(sys.argv[3]), int(sys.argv[4])) if len(sys.argv) == 5 else (1200, 540)

    sdk = PayloadSdkInterface()
    sdk.sdkInitConnection()
    sdk.checkPayloadConnection()
    sdk.regPayloadStatusChanged(on_status)
    sdk.regPayloadParamChanged(on_param)

    for param_id, value in (("C_SOURCE", view), ("TRACK_MODE", 0), ("GB_MODE", gb_mode)):
        sdk.setPayloadCameraParam(param_id, value, U32)
        time.sleep(0.5)
    for index in TRACK_PARAMS:
        sdk.setParamRate(index, 100 if drive else 200)
    time.sleep(3.0)  # let the view switch settle and attitude start flowing

    start = dict(att)
    print(f"view={sys.argv[1]} GB_MODE={gb_mode} track pixel=({x},{y}) start={start}")
    sdk.setPayloadObjectTrackingMode(tracking_mode_t.TRACK_ACTIVE)
    sdk.setPayloadObjectTrackingPosition(x, y, 128, 128)
    for _ in range(10):
        for _ in range(10):
            time.sleep(0.1)
            if drive:
                drive_step(sdk)
        status = track.get("status")
        print(
            f"status={None if status is None else status & 0xFF} (raw {status}) "
            f"pos=({track.get('x')},{track.get('y')}) "
            f"yaw={att.get('yaw')} pitch={att.get('pitch')}"
        )
    if start and att:
        print(
            f"MOVED yaw={att['yaw'] - start['yaw']:+.2f} "
            f"pitch={att['pitch'] - start['pitch']:+.2f}"
        )
    if drive:
        sdk.setGimbalSpeed(0.0, 0.0, 0.0, input_mode_t.INPUT_SPEED)
    sdk.setPayloadObjectTrackingMode(tracking_mode_t.TRACK_STOP)
    time.sleep(0.5)
    sdk.sdkQuit()


if __name__ == "__main__":
    main()
