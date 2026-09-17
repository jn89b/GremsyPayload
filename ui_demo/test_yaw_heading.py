"""Smallest check that a compass click stays inside the yaw motor range.

Run: python3 ui_demo/test_yaw_heading.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from remote_executor import heading_to_gimbal_yaw, yaw_path  # noqa: E402


def main() -> None:
    # Nose at 090, click 090: dead ahead in both frames.
    assert heading_to_gimbal_yaw(90, 90, 0, "vehicle") == (0.0, 0.0, False)
    assert heading_to_gimbal_yaw(90, 90, 0, "earth") == (90.0, 0.0, False)

    # Nose at 000, click 270: -90 of nose; earth frame sends the heading.
    assert heading_to_gimbal_yaw(270, 0, 0, "vehicle") == (-90.0, -90.0, False)
    assert heading_to_gimbal_yaw(270, 0, 0, "earth") == (-90.0, -90.0, False)

    # Straight behind is unreachable: clamped to the limit, same sign.
    cmd, body, clamped = heading_to_gimbal_yaw(180, 0, 0, "vehicle")
    assert clamped and abs(body) == 170.0 and cmd == body
    cmd, body, clamped = heading_to_gimbal_yaw(175, 0, 0, "earth")
    assert clamped and body == 170.0 and cmd == 170.0
    cmd, body, clamped = heading_to_gimbal_yaw(185, 0, 0, "earth")
    assert clamped and body == -170.0 and cmd == -170.0

    # Wraps across 360: nose 350, click 010 is +20, not -340.
    assert heading_to_gimbal_yaw(10, 350, 0, "vehicle") == (20.0, 20.0, False)

    # Yaw offset calibration is undone before the limit check.
    assert heading_to_gimbal_yaw(100, 0, 10, "vehicle") == (90.0, 90.0, False)

    # Gimbal at E (+90), click SW (-135): shortest arc crosses the rear,
    # so go to the nose first.
    assert yaw_path(90, -135) == [0.0, -135]
    assert yaw_path(-135, 90) == [0.0, 90]
    # Same side, or short hop across the nose: direct.
    assert yaw_path(90, 170) == [170]
    assert yaw_path(90, -80) == [-80]
    assert yaw_path(-170, 170) == [0.0, 170]
    assert yaw_path(0, -135) == [-135]


if __name__ == "__main__":
    main()
    print("ok")
