"""Smallest check that an IR click lands on the matching EO pixel.

Run: python3 ui_demo/test_ir_map.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from remote_executor import ir_px_to_eo_px  # noqa: E402


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
    assert ir_px_to_eo_px(0, 0, 60.0, 45.0, 10.0, 8.0) == (0, 0, False)
    assert ir_px_to_eo_px(1919, 1079, 60.0, 45.0, 10.0, 8.0) == (1919, 1079, False)


if __name__ == "__main__":
    main()
    print("ok")
