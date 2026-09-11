#!/usr/bin/env python3
"""Set the Gremsy payload's stream view source over MAVLink.

Run on the Pi from the root of the GremsyPayload checkout (uses its libs/config.py):

    python3 set_view_source.py eo        # EO camera only
    python3 set_view_source.py ir        # IR camera only
    python3 set_view_source.py eoir      # EO with IR picture-in-picture (0)
    python3 set_view_source.py ireo      # IR with EO picture-in-picture (3)
    python3 set_view_source.py 6         # any raw value the payload accepts

Stop anything else that holds a MAVLink session to the payload (remote_executor.py)
before running this, then start it again afterwards.
"""

import os
import sys
import time

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "libs"))

from pymavlink import mavutil  # noqa: E402
from payload_define import PAYLOAD_CAMERA_VIEW_SRC  # noqa: E402
from payload_sdk import PayloadSdkInterface  # noqa: E402

VIEW_SOURCES = {"eoir": 0, "eo": 1, "ir": 2, "ireo": 3}


def main() -> None:
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    arg = sys.argv[1].lower()
    value = VIEW_SOURCES.get(arg)
    if value is None:
        try:
            value = int(arg)
        except ValueError:
            sys.exit(f"unknown view source {arg!r}; use one of {', '.join(VIEW_SOURCES)} or an integer")

    payload = PayloadSdkInterface()
    payload.sdkInitConnection()
    payload.checkPayloadConnection()
    print(f"setting {PAYLOAD_CAMERA_VIEW_SRC} = {value} ({arg})")
    payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, value, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)
    time.sleep(1.0)  # give the payload a moment to apply before the session closes
    payload.sdkQuit()
    print("done")


if __name__ == "__main__":
    main()
