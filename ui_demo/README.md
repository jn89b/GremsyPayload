# Payload SDK UI Demo - Python

Graphical User Interface for Gremsy Payload SDK, ported from C++ GTK implementation.

## System Requirements

### System Dependencies (Ubuntu/Debian)

```bash
# GTK+ 3.0 and GStreamer
sudo apt-get install python3-gi python3-gi-cairo gir1.2-gtk-3.0
sudo apt-get install gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good
sudo apt-get install gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav
sudo apt-get install gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0
```

### Python Dependencies

```bash
pip install PyGObject
```

## Quick Setup (Ubuntu 24.04)

If you see `gst_parse_error: no element "rtspsrc"`, install the full GStreamer runtime stack below.

1. Install system packages:

```bash
sudo apt-get update
sudo apt-get install -y \
	python3-gi python3-gi-cairo gir1.2-gtk-3.0 \
	gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0 \
	gstreamer1.0-tools gstreamer1.0-plugins-base \
	gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
	gstreamer1.0-plugins-ugly gstreamer1.0-libav
```

2. Install Python dependencies:

```bash
pip install -r requirements.txt
```

3. Verify RTSP support:

```bash
gst-inspect-1.0 rtspsrc
```

Expected result: plugin information is printed (not "No such element").

## Native Windows Setup

The UI now includes native Windows video-sink handling for embedded playback in the GTK panel.

Recommended setup path is MSYS2 (UCRT64) because it provides GTK + PyGObject + GStreamer together.

1. Install MSYS2 from msys2.org.
2. Open MSYS2 UCRT64 shell and install packages:

```bash
pacman -Syu
pacman -S --needed \
	mingw-w64-ucrt-x86_64-python \
	mingw-w64-ucrt-x86_64-python-pip \
	mingw-w64-ucrt-x86_64-python-gobject \
	mingw-w64-ucrt-x86_64-gtk3 \
	mingw-w64-ucrt-x86_64-gstreamer \
	mingw-w64-ucrt-x86_64-gst-plugins-base \
	mingw-w64-ucrt-x86_64-gst-plugins-good \
	mingw-w64-ucrt-x86_64-gst-plugins-bad \
	mingw-w64-ucrt-x86_64-gst-libav
```

3. In the same UCRT64 shell, install Python requirements:

```bash
cd /c/path/to/PayloadSdk
pip install -r requirements.txt
```

4. Verify RTSP plugin support:

```bash
gst-inspect-1.0 rtspsrc
```

5. Run the UI:

```bash
cd /c/path/to/PayloadSdk/ui_demo
python ui_demo.py
```

For remote mode:

```bash
python ui_demo.py --remote-mode connect --remote-host <gimbal_machine_vpn_ip> --remote-port 5000
```

Notes:
- On Windows, the UI prefers `d3d11videosink`, then `d3dvideosink`, then `glimagesink` to keep video inside the GUI.
- Use RTSP URL path `/eo` if your camera stream is `rtsp://<ip>:8554/eo`.

## Usage

### Standard Payload Mode

```bash
cd PayloadSdk
python ui_demo/ui_demo.py
```

### MB1 Payload Mode

```bash
cd PayloadSdk
python ui_demo/ui_demo.py --mb1
```

## Remote Command Bridge (MVP)

This repository now includes an MVP remote-control split for click-to-track:

- UI machine: runs GTK UI and sends tracking commands over TCP.
- Gimbal machine: runs executor service, receives commands, then executes SDK calls locally.
- Video stream: still reaches UI machine directly via RTSP over VPN.

### Scope in this MVP

- Supported remotely:
	- `PAYLOAD_TOUCH` (click pixel to track)
	- `PAYLOAD_TRACK` (start/stop tracking)
- Other UI commands are intentionally blocked in remote mode.

### 1) Start executor on gimbal machine

```bash
cd PayloadSdk/ui_demo
python remote_executor.py --role listen --host 0.0.0.0 --port 5000 --payload-ip 192.168.55.1
```

Optional token:

```bash
python remote_executor.py --role listen --host 0.0.0.0 --port 5000 --token my-shared-token
```

### 2) Start UI on operator machine in remote mode

```bash
cd PayloadSdk
python ui_demo/ui_demo.py \
	--remote-mode connect \
	--remote-host <gimbal_machine_vpn_ip> \
	--remote-port 5000
```

Optional token:

```bash
python ui_demo/ui_demo.py --remote-mode connect --remote-host <gimbal_machine_vpn_ip> --remote-port 5000 --remote-token my-shared-token
```

Then in the UI:

1. Put the payload/camera RTSP-reachable IP in the IP field.
2. Press Connect (this now connects command bridge in remote mode).
3. Click video pixels to send track-position commands to the remote executor.

### Listener/connector role flexibility

Both programs support `--role connect|listen` / `--remote-mode connect|listen`.
Use whichever direction best fits your VPN routing rules.

### Reliability behavior

- Per-command ACK is required.
- Timeout and retry are enabled (default retry count: 2).
- Duplicate message IDs are acknowledged without re-execution.

### New files

- `remote_bridge.py`: shared TCP protocol and ACK/retry transport.
- `remote_executor.py`: headless command-consumer service for gimbal machine.

## Features

### Connection
- Payload IP address configuration
- Connect/Disconnect button
- Connection status display

### Video Streaming
- RTSP video playback using GStreamer
- Play/Stop/Fullscreen controls
- Touch-to-track on video area

### Payload Settings
- **Camera View & Record**: View mode (EO/IR/EO+IR/IR+EO), Record source
- **Capture/Record**: Capture button, Record button, SD card status

### Camera Settings
- **Zoom Controls**: Continuous zoom, Step zoom, Range zoom, Speed slider
- **Focus Controls**: Continuous focus, Auto focus, Speed slider
- **Exposure**: AE mode, Shutter, Iris, Gain
- **White Balance**: Mode selection, WB trigger
- **IR Camera**: Palette selection, FFC mode, FFC trigger
- **LRF**: Frequency mode
- **OSD**: Disable/Debug/Status
- **Image Flip**: Flip image on/off

### Gimbal Settings
- **Gimbal Mode**: Off/Lock/Follow/Mapping
- **Speed Control**: Speed slider, Direction buttons (Up/Down/Left/Right/Home)
- **Angle Control**: Pitch/Roll/Yaw sliders

### Payload Info Display
- Gimbal Mode, Pitch, Roll, Yaw
- View Mode, Record Source
- EO/IR Zoom Levels
- IR Type, Palette, FFC Mode, Temperatures
- LRF Offset X/Y, Range
- Target GPS Coordinates
- Payload GPS Coordinates

### MB1 Additional Features (when running with --mb1)
- **Setting Target**: Select target device (EO Camera/IR Camera/Gimbal)
- **RC Mode**: Select RC mode (Gremsy/Standard)
- **Storage Type**: Select storage (Internal/SD Card)
- **EO Advanced**: Scene Mode, AE Compensation, White Balance, ISO, Sharpness
- **IR Advanced**: Gain Mode, Contrast Mode, AGC Mode, AGC Linear Percent
- **IR SpotMeter**: Mode, Units, Size
- **IR Isotherm**: Mode, Units, Threshold
- **Object Detection**: Enable/Disable
- **Gimbal Forward Flag**: Overwrite/Forward

## File Structure

```
ui_demo/
├── README.md                  # This file
├── ui_demo.py                 # Main entry point, handles connection and callbacks
├── main_window.py             # Main window class
└── payload_settings_tab.py    # Payload settings tab with all controls
```

## Architecture

The UI follows the same callback-based event-driven architecture as the C++ version:

1. **UI to SDK Communication**: UI commands are sent through callbacks
2. **SDK to UI Communication**: Status updates via registered callbacks
3. **Thread Safety**: GLib.idle_add() used to update UI from background threads

### Data Flow

```
+-------------+     Callback      +-------------+     MAVLink      +----------+
|   UI Demo   | <---------------- | Payload SDK | <-------------> | Payload  |
|  (GTK UI)   | ---------------> |  (Python)   |                 | (Camera) |
+-------------+   UI Commands     +-------------+                 +----------+
```

### Main Callbacks

- `regPayloadStatusChanged`: Receives capture status, storage info, gimbal attitude
- `regPayloadParamChanged`: Receives camera and gimbal parameters
- `regPayloadStreamChanged`: Receives streaming URL

## Comparison with C++ Version

| Feature | C++ Version | Python Version |
|---------|-------------|----------------|
| UI Framework | gtkmm-3.0 | PyGObject (GTK+ 3.0) |
| Video Streaming | GStreamer C API | GStreamer Python bindings |
| Threading | pthread | Python threading |
| Build System | CMake | None (interpreted) |
| Dependency | Compile libraries | pip install |

## Troubleshooting

### Video not displaying
- Check if RTSP URL is correct
- Verify GStreamer plugins are installed
- Check network connection to payload

### Video pops out into separate window
- Linux: run under X11 and ensure `ximagesink` or `glimagesink` is available.
- Windows: run from MSYS2 UCRT64 shell so `d3d11videosink`/`d3dvideosink` plugins are available.

### Cannot connect to payload
- Verify IP address is correct
- Check if payload is powered on
- Check firewall is not blocking UDP port

### Remote mode cannot connect
- Verify VPN connectivity between machines and TCP port reachability.
- Ensure UI `--remote-host/--remote-port` matches executor endpoint.
- Ensure tokens match on both sides when token is set.
- In listener mode, make sure firewall allows inbound TCP on bridge port.

### Not receiving parameters
- Check if payload supports PARAM_EXT
- Verify connection is successful
- Check terminal logs for debugging
