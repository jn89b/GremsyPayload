#!/usr/bin/env python3
"""IR overlay: /ir_raw -> telemetry text + north arrow -> /ir.

    python3 ir_overlay.py                    # run the pipeline (systemd: ir-overlay.service)
    python3 ir_overlay.py --selftest out.png # check the math, render one sample frame

ffmpeg decodes to raw BGR on a pipe, this script draws, a second ffmpeg encodes
libx264 and publishes to MediaMTX. Telemetry comes from the JSON snapshot that
remote_executor.py rewrites every 0.2 s.

ponytail: no reconnect loop; any pipe EOF exits non-zero and systemd restarts us.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time
from datetime import datetime
from zoneinfo import ZoneInfo

import cv2
import numpy as np

try:
    import mgrs as _mgrs_lib

    _MGRS = _mgrs_lib.MGRS()
except ImportError:  # keep streaming with lat/lon until `pip install mgrs`
    _MGRS = None

FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.45
LINE_H = 18
MARGIN = 8
STALE_S = 2.0  # older than this, a heading is worse than no heading
FT_PER_M = 3.28084
ARROW_R = 24
CROSS_R = 4  # centre "+" half-length in pixels
ARROW_LABEL = "N"  # rides the arrow tip, sensor-ball style; "" for a bare arrow
CLOCK_TZ = ZoneInfo("America/Los_Angeles")  # explicit: the Pi's own zone is not Pacific


def north_arrow_vec(camera_az_deg):
    """Unit vector (dx, dy) in image pixels pointing at north; image up = camera azimuth."""
    a = math.radians(camera_az_deg)
    return -math.sin(a), -math.cos(a)


def clock_text(now):
    """'2026-09-18 15:53:43 PDT'; PST/PDT follows the date."""
    return datetime.fromtimestamp(now, CLOCK_TZ).strftime("%Y-%m-%d %H:%M:%S %Z")


def _num(value, fmt, missing="--"):
    return missing if value is None else format(value, fmt)


def mgrs_10m(lat, lon):
    """'15R TN 1234 5678' (10 m MGRS); '--' without a position."""
    if lat is None or lon is None:
        return "--"
    if _MGRS is None:
        return f"{lat:.5f} {lon:.5f}"
    g = _MGRS.toMGRS(lat, lon, MGRSPrecision=4)
    # Sliced from the end: the zone is one or two digits.
    return f"{g[:-10]} {g[-10:-8]} {g[-8:-4]} {g[-4:]}"


def telemetry_lines(status):
    """(left_lines, right_lines, camera_az or None) from an executor status dict."""
    if status is None:
        return ["TELEM STALE"], [], None

    # The executor's estimate follows the tracked pixel while tracking, image centre otherwise.
    tgt = status.get("estimate_valid")
    left = ["TGT " + mgrs_10m(status.get("estimated_target_lat") if tgt else None,
                              status.get("estimated_target_lon") if tgt else None)]

    gps = status.get("gps_valid")
    alt_m = status.get("vehicle_alt_m")
    right = [
        "ACFT " + mgrs_10m(status.get("vehicle_lat") if gps else None,
                           status.get("vehicle_lon") if gps else None),
        f"ALT {_num(None if alt_m is None else alt_m * FT_PER_M, '.0f')}ft MSL",
    ]
    return left, right, status.get("camera_yaw_ned_deg")


def _text(frame, text, x, y):
    # Black outline under white text stays readable on any thermal palette. Offsets, not a
    # thicker stroke: OpenCV 5 widens glyphs with thickness and the outline drifts off the text.
    for ox, oy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        cv2.putText(frame, text, (x + ox, y + oy), FONT, FONT_SCALE, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(frame, text, (x, y), FONT, FONT_SCALE, (255, 255, 255), 1, cv2.LINE_AA)


def draw(frame, status, now):
    h, w = frame.shape[:2]
    left, right, az = telemetry_lines(status)

    # Compare against `TZ=America/Los_Angeles date` on the viewer to read latency and drift.
    _text(frame, clock_text(now), MARGIN, MARGIN + 12)
    for i, line in enumerate(reversed(left)):
        _text(frame, line, MARGIN, h - MARGIN - i * LINE_H)
    for i, line in enumerate(reversed(right)):
        tw = cv2.getTextSize(line, FONT, FONT_SCALE, 1)[0][0]
        _text(frame, line, w - MARGIN - tw, h - MARGIN - i * LINE_H)

    # Boresight "+": the pixel the TGT grid refers to when nothing is being tracked.
    mx, my = w // 2, h // 2
    for color, thickness in (((0, 0, 0), 3), ((255, 255, 255), 1)):
        cv2.line(frame, (mx - CROSS_R, my), (mx + CROSS_R, my), color, thickness)
        cv2.line(frame, (mx, my - CROSS_R), (mx, my + CROSS_R), color, thickness)

    if az is None:  # no arrow beats a wrong arrow
        return
    # Sensor-ball style: thin monochrome stick arrow, open chevron head, letter at the tip.
    reach = ARROW_R + 20  # room for the letter when north points at the frame corner
    cx, cy = w - MARGIN - reach, MARGIN + reach
    dx, dy = north_arrow_vec(az)

    def at(along, across=0.0):
        return round(cx + dx * along - dy * across), round(cy + dy * along + dx * across)

    tip = at(ARROW_R)
    segments = [(at(-ARROW_R), tip), (tip, at(ARROW_R - 10, 6)), (tip, at(ARROW_R - 10, -6))]
    for color, thickness in (((0, 0, 0), 3), ((255, 255, 255), 1)):
        for a, b in segments:
            cv2.line(frame, a, b, color, thickness, cv2.LINE_AA)
    if ARROW_LABEL:
        (tw, th), _ = cv2.getTextSize(ARROW_LABEL, FONT, FONT_SCALE, 1)
        lx, ly = at(ARROW_R + 11)
        _text(frame, ARROW_LABEL, lx - tw // 2, ly + th // 2)


def read_status(path, now):
    """Status dict, or None when the file is missing, torn, or stale."""
    try:
        if now - os.path.getmtime(path) > STALE_S:
            return None
        with open(path) as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return None


def run(args):
    w, h = (int(v) for v in args.size.lower().split("x"))
    dec = subprocess.Popen(
        ["ffmpeg", "-loglevel", "error", "-nostdin", "-rtsp_transport", "tcp",
         "-fflags", "nobuffer", "-flags", "low_delay", "-i", args.src,
         "-an", "-vf", f"scale={w}:{h}", "-f", "rawvideo", "-pix_fmt", "bgr24", "-"],
        stdout=subprocess.PIPE,
    )
    enc = subprocess.Popen(
        ["ffmpeg", "-loglevel", "error", "-nostdin",
         "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{w}x{h}", "-r", args.fps, "-i", "-",
         "-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency",
         "-pix_fmt", "yuv420p", "-g", "30", "-b:v", args.bitrate,
         "-f", "rtsp", "-rtsp_transport", "tcp", args.dst],
        stdin=subprocess.PIPE,
    )
    nbytes = w * h * 3
    status, n = None, 0
    try:
        while True:
            data = dec.stdout.read(nbytes)
            if len(data) < nbytes:
                print("decoder ended", file=sys.stderr)
                return 1
            now = time.time()
            if n % 6 == 0:  # ~5 Hz, matches the executor's write rate
                status = read_status(args.status_file, now)
            n += 1
            frame = np.frombuffer(data, np.uint8).reshape(h, w, 3).copy()
            draw(frame, status, now)
            enc.stdin.write(frame.tobytes())
    except BrokenPipeError:
        print("encoder ended", file=sys.stderr)
        return 1
    finally:
        dec.kill()
        enc.kill()


def selftest(png):
    for az, want in ((0, (0, -1)), (90, (-1, 0)), (180, (0, 1)), (270, (1, 0))):
        got = north_arrow_vec(az)
        assert math.isclose(got[0], want[0], abs_tol=1e-9), (az, got)
        assert math.isclose(got[1], want[1], abs_tol=1e-9), (az, got)

    assert telemetry_lines(None) == (["TELEM STALE"], [], None)
    assert clock_text(0) == "1969-12-31 16:00:00 PST", clock_text(0)
    assert clock_text(1789772023) == "2026-09-18 15:53:43 PDT", clock_text(1789772023)

    status = {"gps_valid": True, "vehicle_lat": 29.5, "vehicle_lon": -95.1, "vehicle_alt_m": 120.4,
              "camera_yaw_ned_deg": 45.0, "estimate_valid": True,
              "estimated_target_lat": 29.503, "estimated_target_lon": -95.097}
    left, right, az = telemetry_lines(status)
    assert right[1] == "ALT 395ft MSL" and az == 45.0  # 120.4 m
    if _MGRS is not None:
        # White House (published 1 m grid 18S UJ 23394 07395; MGRS truncates, never rounds),
        # then a single-digit zone, which the library zero-pads.
        assert mgrs_10m(38.8977, -77.0365) == "18S UJ 2339 0739", mgrs_10m(38.8977, -77.0365)
        assert mgrs_10m(21.3, -157.85) == "04Q FJ 1928 5578", mgrs_10m(21.3, -157.85)
        assert right[0].startswith("ACFT 15R ") and left[0].startswith("TGT 15R ")
        assert [len(part) for part in right[0].split()[1:]] == [3, 2, 4, 4], right[0]

    # No GPS / no estimate: positions blank out, the arrow keeps working off attitude alone.
    left, right, az = telemetry_lines({"gps_valid": False, "vehicle_lat": 0.0, "camera_yaw_ned_deg": 10.0,
                                       "estimate_valid": False, "estimated_target_lat": 1.0})
    assert left == ["TGT --"] and right[0] == "ACFT --" and right[1] == "ALT --ft MSL" and az == 10.0

    if png:
        frame = np.full((512, 640, 3), 90, np.uint8)
        cv2.rectangle(frame, (200, 150), (440, 360), (230, 230, 230), -1)
        draw(frame, status, time.time())
        cv2.imwrite(png, frame)
    print("ok")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default="rtsp://127.0.0.1:8554/ir_raw")
    ap.add_argument("--dst", default="rtsp://127.0.0.1:8554/ir")
    ap.add_argument("--size", default="640x512")
    ap.add_argument("--fps", default="30000/1001")
    ap.add_argument("--bitrate", default="1500k")
    ap.add_argument("--status-file", default=os.environ.get("GREMSY_STATUS_FILE", "/dev/shm/gremsy_status.json"))
    ap.add_argument("--selftest", nargs="?", const="", metavar="PNG")
    args = ap.parse_args()
    if args.selftest is not None:
        selftest(args.selftest)
    else:
        sys.exit(run(args))
