# Payload UI Demo - PyQt MVP

This folder provides a PyQt alternative UI focused on remote click-to-track control.

## Scope

- Embedded RTSP preview inside the Qt window (OpenCV-based).
- Remote bridge connect/listen support over TCP.
- Click-to-track command send (`PAYLOAD_TOUCH`).
- Track toggle command send (`PAYLOAD_TRACK`).
- Reuses existing backend modules:
  - `ui_demo/remote_bridge.py`
  - `libs/config.py`

## Install

From repository root:

```bash
pip install -r ui_demo_pyqt/requirements.txt
```

## Run

From repository root:

```bash
python ui_demo_pyqt/app.py --remote-mode connect --remote-host 100.89.182.15 --remote-port 5000
```

Or run as listener:

```bash
python ui_demo_pyqt/app.py --remote-mode listen --remote-host 0.0.0.0 --remote-port 5000
```

## Expected flow

1. Start `ui_demo/remote_executor.py` on the gimbal-attached machine.
2. Start this PyQt app on the operator machine.
3. Connect bridge in the app.
4. Enter RTSP URL (example: `rtsp://100.89.182.15:8554/eo`) and click Play.
5. Enable Track if needed and click on the video to send touch coordinates.

## Notes

- Coordinate mapping converts displayed frame clicks to 1920x1080 payload tracking coordinates.
- This is an MVP frontend. Existing GTK app remains the full-feature UI.
