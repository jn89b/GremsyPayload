# Setting camera to view visual only
- To do so use the `set_camera.py` script to set the camera for visual only on the `eo` rtsp streampoint by executing the following
```bash
uv run set_camera.py
```
- Prior to doing that though make sure the GREMSY_IP, GREMSY_PORT, and TARGET_COMPONENT_ID matches to is set up
```pymavlink

GREMSY_IP = "192.168.1.240"
GREMSY_PORT = 14566

TARGET_SYSTEM = 1
TARGET_COMPONENT = 101  # verify this on the Lynx check the website 

```