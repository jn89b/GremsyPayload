#!/usr/bin/env python3
"""gremsy_settings.py: read and change Gremsy payload camera settings over MAVLink.

Run on the Pi from the root of the GremsyPayload checkout (it uses libs/config.py
for the payload's address and type). Stop gremsy.service first: the payload
should have one MAVLink client at a time.

    python3 gremsy_settings.py show                 # every parameter the payload reports
    python3 gremsy_settings.py get C_SOURCE OSD_MODE
    python3 gremsy_settings.py view eo              # eo | ir | eoir | ireo      (C_SOURCE)
    python3 gremsy_settings.py osd off              # off | debug | status      (OSD_MODE)
    python3 gremsy_settings.py clean                # view eo + osd off: bare EO picture for /eo
    python3 gremsy_settings.py set OSD_MODE 0       # any parameter id, raw integer

Every change is read back from the payload afterwards so you can see whether
it actually took. Parameter ids are Gremsy's 16-character strings (C_SOURCE,
OSD_MODE, C_V_FLIP, ...); `show` lists the ones this payload has.
"""

from __future__ import annotations

import os
import sys
import threading
import time

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "libs"))

from pymavlink import mavutil  # noqa: E402
from payload_sdk import PayloadSdkInterface, payload_status_event_t  # noqa: E402

VIEW_SOURCE = {"eoir": 0, "eo": 1, "ir": 2, "ireo": 3}      # C_SOURCE
OSD_MODE = {"off": 0, "debug": 1, "status": 2}               # OSD_MODE
PRESETS = {"clean": [("C_SOURCE", VIEW_SOURCE["eo"]), ("OSD_MODE", OSD_MODE["off"])]}

SETTLE_S = 0.5      # after a set, before reading back
LIST_WAIT_S = 3.0   # how long to collect PARAM_EXT_VALUE replies for `show`


class Payload:
    """Thin wrapper: connects, collects parameter values, sets, reads back."""

    def __init__(self) -> None:
        self.values: dict[str, int] = {}
        self._lock = threading.Lock()
        self._event = threading.Event()
        self.sdk = PayloadSdkInterface()
        self.sdk.regPayloadParamChanged(self._on_param)
        self.sdk.sdkInitConnection()
        self.sdk.checkPayloadConnection()

    def _on_param(self, event, param_id: str, params) -> None:
        if event == payload_status_event_t.PAYLOAD_CAM_PARAMS:
            with self._lock:
                self.values[param_id.rstrip("\0")] = int(params[1])
            self._event.set()

    def close(self) -> None:
        self.sdk.sdkQuit()

    def read_all(self) -> dict[str, int]:
        self.sdk.getPayloadCameraSettingList()
        deadline = time.monotonic() + LIST_WAIT_S
        while time.monotonic() < deadline:       # keep collecting until replies go quiet
            self._event.clear()
            if not self._event.wait(0.6):
                break
        with self._lock:
            return dict(self.values)

    def read(self, param_id: str, timeout: float = 2.0):
        with self._lock:
            self.values.pop(param_id, None)
        self._event.clear()
        self.sdk.getPayloadCameraSettingByID(param_id)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._event.wait(deadline - time.monotonic())
            with self._lock:
                if param_id in self.values:
                    return self.values[param_id]
            self._event.clear()
        return None

    def set(self, param_id: str, value: int) -> None:
        self.sdk.setPayloadCameraParam(param_id, value, mavutil.mavlink.MAV_PARAM_TYPE_UINT32)


def apply(payload: Payload, changes: list[tuple[str, int]]) -> bool:
    ok = True
    for param_id, value in changes:
        print(f"set  {param_id} = {value}")
        payload.set(param_id, value)
        time.sleep(SETTLE_S)
        got = payload.read(param_id)
        if got == value:
            print(f"     {param_id} confirmed = {got}")
        else:
            ok = False
            print(f"     {param_id} reads back as {got!r}, NOT {value}: the payload did not apply it")
    return ok


def parse_command(argv: list[str]) -> tuple[str, list]:
    if not argv:
        sys.exit(__doc__)
    cmd, rest = argv[0].lower(), argv[1:]
    if cmd == "show" and not rest:
        return "show", []
    if cmd == "get" and rest:
        return "get", rest
    if cmd == "view" and len(rest) == 1 and rest[0].lower() in VIEW_SOURCE:
        return "set", [("C_SOURCE", VIEW_SOURCE[rest[0].lower()])]
    if cmd == "osd" and len(rest) == 1 and rest[0].lower() in OSD_MODE:
        return "set", [("OSD_MODE", OSD_MODE[rest[0].lower()])]
    if cmd in PRESETS and not rest:
        return "set", PRESETS[cmd]
    if cmd == "set" and len(rest) == 2:
        try:
            return "set", [(rest[0], int(rest[1]))]
        except ValueError:
            sys.exit(f"value must be an integer, got {rest[1]!r}")
    sys.exit(__doc__)


def main() -> None:
    action, payload_args = parse_command(sys.argv[1:])
    payload = Payload()
    try:
        if action == "show":
            values = payload.read_all()
            if not values:
                print("no parameters received (is the payload's camera component answering?)")
            for param_id in sorted(values):
                print(f"{param_id:<16} = {values[param_id]}")
        elif action == "get":
            for param_id in payload_args:
                print(f"{param_id:<16} = {payload.read(param_id)}")
        else:
            time.sleep(SETTLE_S)
            if not apply(payload, payload_args):
                sys.exit(1)
    finally:
        payload.close()


if __name__ == "__main__":
    main()
