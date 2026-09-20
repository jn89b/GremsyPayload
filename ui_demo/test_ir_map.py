"""Smallest check that an IR click lands on the matching EO pixel.

Run: python3 ui_demo/test_ir_map.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from remote_executor import (  # noqa: E402
    ir_fov_deg,
    ir_px_to_eo_px,
    night_track_rates,
)


def main() -> None:
    # Center stays center whatever the FOVs are.
    assert ir_px_to_eo_px(960, 540, 20.0, 16.0, 60.0, 45.0) == (960, 540, True)

    # Equal FOVs: identity.
    assert ir_px_to_eo_px(100, 100, 60.0, 45.0, 60.0, 45.0) == (100, 100, True)
    assert ir_px_to_eo_px(1900, 1000, 60.0, 45.0, 60.0, 45.0) == (1900, 1000, True)

    # IR narrower than EO: the point moves toward the center, stays inside.
    x, y, inside = ir_px_to_eo_px(100, 100, 20.0, 16.0, 60.0, 45.0)
    assert inside
    assert 100 < x < 960 and 100 < y < 540, (x, y)

    # IR wider than EO (EO zoomed in): corner falls outside, gets clamped.
    # Outside EO: pulled in along the click's own direction to just inside
    # the border (the payload ignores clicks on the border itself).
    x, y, inside = ir_px_to_eo_px(0, 0, 60.0, 45.0, 10.0, 8.0)
    assert not inside and 0 < x < 960 and 0 < y < 540 and 8 in (x, y), (x, y)
    x, y, inside = ir_px_to_eo_px(960, 1079, 60.0, 45.0, 10.0, 8.0)
    assert (x, y, inside) == (960, 1072, False), (x, y)  # straight down stays straight down

    # Night mode: the 16:9 frame pillarboxes the 5:4 IR picture at full
    # height, so x squeezes by (5/4)/(16/9) about centre and y is untouched.
    ir_h, ir_v = ir_fov_deg(22.7, 0, 640 / 512)
    frame_h = 2 * math.degrees(math.atan(math.tan(math.radians(ir_v) / 2) * 16 / 9))
    assert ir_px_to_eo_px(1920, 800, ir_h, ir_v, frame_h, ir_v)[:2] == (960 + 675, 800)
    assert ir_px_to_eo_px(0, 300, ir_h, ir_v, frame_h, ir_v)[:2] == (960 - 675, 300)

    # Night tracking: steer toward the tracker box (reported by its top-left
    # corner). Values from the 2026-09-19 hardware run at IR 1x, f = 3364 px.
    assert night_track_rates(896, 476, 128, 128, 3364) == (0.0, 0.0)  # centred
    pitch, yaw = night_track_rates(1136, 476, 128, 128, 3364)  # 240 px right
    assert pitch == 0.0 and 8.0 < yaw < 8.4, (pitch, yaw)
    pitch, yaw = night_track_rates(896, 176, 128, 128, 3364)  # above centre
    assert yaw == 0.0 and pitch > 0.0, (pitch, yaw)
    assert night_track_rates(0, 476, 128, 128, 3364)[1] < 0.0  # left -> yaw left
    assert abs(night_track_rates(1792, 476, 128, 128, 800)[1]) == 30.0  # capped


if __name__ == "__main__":
    main()
    print("ok")
