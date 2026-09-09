#!/usr/bin/env python3

import os

# MUST be set before importing pymavlink
os.environ["MAVLINK20"] = "1"

from pymavlink import mavutil
import time

GREMSY_IP = "192.168.1.240"
GREMSY_PORT = 14566

TARGET_SYSTEM = 1
TARGET_COMPONENT = 101  # verify this on the Lynx

gimbal = mavutil.mavlink_connection(
    f"udpout:{GREMSY_IP}:{GREMSY_PORT}",
    source_system=255,
    source_component=190,
)

print("MAVLink version:", mavutil.mavlink.WIRE_PROTOCOL_VERSION)
print(f"Connecting to Gremsy at {GREMSY_IP}:{GREMSY_PORT}")

time.sleep(1)

print("Setting camera view to EO only...")

gimbal.mav.param_ext_set_send(
    TARGET_SYSTEM,
    TARGET_COMPONENT,
    b"C_SOURCE",
    b"1",  # 1 = Only EO
    mavutil.mavlink.MAV_PARAM_EXT_TYPE_UINT32,
)

print("C_SOURCE=1 sent")