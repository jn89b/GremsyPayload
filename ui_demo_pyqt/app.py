#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""
PyQt remote Gremsy payload UI.

Interaction model
-----------------
- Track unchecked:
    click video -> move gimbal to clicked image pixel only.

- Track checked:
    click video -> begin Gremsy object tracking at clicked image pixel.

Geolocation is optional:
- After the remote bridge connects, the UI waits --geo-timeout seconds for
  GEO_STATUS events from the backend.
- If ArduPilot/GPS is not available before the timeout, point-camera and
  tracking still work normally.
- The UI shows a persistent warning that geolocation is unavailable.
- If geolocation later becomes available, the warning clears automatically.

Expected asynchronous backend event
-----------------------------------
The remote executor should periodically publish:

    bridge.send_event(
        "GEO_STATUS",
        {
            "ardupilot_connected": True,
            "gps_valid": True,
            "geolocation_available": True,
            "target_lat": 38.1234567,   # optional until known
            "target_lon": -94.1234567,  # optional until known
            "target_alt": 250.0,        # optional until known
            "detail": "GPS valid",
        },
    )
"""

import argparse
import math
import os
import sys
import time
from typing import List, Optional, Tuple

from PySide6 import QtCore, QtGui, QtWidgets

# Reuse shared bridge + config from existing project modules.
sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(__file__),
        "..",
        "libs",
    ),
)
sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(__file__),
        "..",
        "ui_demo",
    ),
)

from config import ConnectionConfig, RemoteBridgeConfig
from remote_bridge import BridgeConfig, TcpCommandBridge
from widgets.video_widget import RtspVideoWidget


CMD_PAYLOAD_TOUCH = "PAYLOAD_TOUCH"
CMD_PAYLOAD_TRACK = "PAYLOAD_TRACK"

EVENT_GEO_STATUS = "GEO_STATUS"

GREMSY_FRAME_W = 1920
GREMSY_FRAME_H = 1080

DEFAULT_GEO_TIMEOUT_S = 8.0


class BridgeSignals(QtCore.QObject):
    """
    Thread-safe handoff from TcpCommandBridge worker threads into Qt.
    """

    event_received = QtCore.Signal(str, object)
    connection_changed = QtCore.Signal(bool)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, args: argparse.Namespace):
        super().__init__()

        self.setWindowTitle(
            "Payload UI Demo - Click / Track"
        )
        self.resize(1400, 900)

        self.bridge: Optional[TcpCommandBridge] = None
        self.bridge_connected = False
        self.tracking_enabled = False

        # --------------------------------------------------------------
        # Optional geolocation state
        # --------------------------------------------------------------
        self.geo_timeout_s = max(
            0.0,
            float(args.geo_timeout),
        )

        self.geo_wait_started: Optional[float] = None
        self.geo_timeout_notified = False

        self.ardupilot_connected = False
        self.gps_valid = False
        self.geolocation_available = False
        self.last_geo_event_time: Optional[float] = None

        self.target_lat: Optional[float] = None
        self.target_lon: Optional[float] = None
        self.target_alt: Optional[float] = None

        self.bridge_signals = BridgeSignals()
        self.bridge_signals.event_received.connect(
            self._on_bridge_event
        )
        self.bridge_signals.connection_changed.connect(
            self._on_bridge_connection_changed
        )

        self._build_ui()
        self._apply_defaults(args)

        # Periodically checks whether the optional geolocation grace
        # period has expired.
        self.geo_timer = QtCore.QTimer(self)
        self.geo_timer.setInterval(250)
        self.geo_timer.timeout.connect(
            self._update_geo_timeout_state
        )
        self.geo_timer.start()

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        root = QtWidgets.QWidget(self)
        self.setCentralWidget(root)

        layout = QtWidgets.QVBoxLayout(root)
        layout.setContentsMargins(
            12,
            12,
            12,
            12,
        )
        layout.setSpacing(10)

        # --------------------------------------------------------------
        # Remote bridge
        # --------------------------------------------------------------
        conn_group = QtWidgets.QGroupBox(
            "Remote Bridge"
        )
        conn_grid = QtWidgets.QGridLayout(
            conn_group
        )

        self.role_combo = QtWidgets.QComboBox()
        self.role_combo.addItems(
            ["connect", "listen"]
        )

        self.host_edit = QtWidgets.QLineEdit()

        self.port_spin = QtWidgets.QSpinBox()
        self.port_spin.setRange(1, 65535)

        self.token_edit = QtWidgets.QLineEdit()
        self.token_edit.setEchoMode(
            QtWidgets.QLineEdit.EchoMode.Password
        )

        self.connect_button = QtWidgets.QPushButton(
            "Connect Bridge"
        )
        self.connect_button.clicked.connect(
            self._toggle_bridge
        )

        self.bridge_status = QtWidgets.QLabel(
            "Disconnected"
        )
        self.bridge_status.setStyleSheet(
            "color: #e11d48; font-weight: 600;"
        )

        conn_grid.addWidget(
            QtWidgets.QLabel("Role"),
            0,
            0,
        )
        conn_grid.addWidget(
            self.role_combo,
            0,
            1,
        )
        conn_grid.addWidget(
            QtWidgets.QLabel("Host"),
            0,
            2,
        )
        conn_grid.addWidget(
            self.host_edit,
            0,
            3,
        )
        conn_grid.addWidget(
            QtWidgets.QLabel("Port"),
            0,
            4,
        )
        conn_grid.addWidget(
            self.port_spin,
            0,
            5,
        )
        conn_grid.addWidget(
            QtWidgets.QLabel("Token"),
            0,
            6,
        )
        conn_grid.addWidget(
            self.token_edit,
            0,
            7,
        )
        conn_grid.addWidget(
            self.connect_button,
            0,
            8,
        )
        conn_grid.addWidget(
            self.bridge_status,
            0,
            9,
        )

        # --------------------------------------------------------------
        # Optional geolocation status
        # --------------------------------------------------------------
        geo_group = QtWidgets.QGroupBox(
            "Target Geolocation (Optional)"
        )
        geo_grid = QtWidgets.QGridLayout(
            geo_group
        )

        self.geo_status_label = QtWidgets.QLabel()
        self.geo_status_label.setWordWrap(True)
        self._set_geo_status_neutral(
            "Geolocation: waiting for remote bridge"
        )

        self.ardupilot_status_label = QtWidgets.QLabel(
            "ArduPilot: --"
        )
        self.gps_status_label = QtWidgets.QLabel(
            "GPS: --"
        )

        self.target_position_label = QtWidgets.QLabel(
            "Target: --"
        )
        self.target_position_label.setTextInteractionFlags(
            QtCore.Qt.TextInteractionFlag.TextSelectableByMouse
        )

        geo_grid.addWidget(
            self.geo_status_label,
            0,
            0,
            1,
            4,
        )
        geo_grid.addWidget(
            self.ardupilot_status_label,
            1,
            0,
        )
        geo_grid.addWidget(
            self.gps_status_label,
            1,
            1,
        )
        geo_grid.addWidget(
            self.target_position_label,
            1,
            2,
            1,
            2,
        )

        # --------------------------------------------------------------
        # RTSP stream controls
        # --------------------------------------------------------------
        stream_group = QtWidgets.QGroupBox(
            "RTSP Stream"
        )
        stream_layout = QtWidgets.QHBoxLayout(
            stream_group
        )

        self.rtsp_url_edit = QtWidgets.QLineEdit()
        self.rtsp_url_edit.setPlaceholderText(
            "rtsp://ip:8554/eo"
        )

        self.play_button = QtWidgets.QPushButton(
            "Play"
        )
        self.play_button.clicked.connect(
            self._start_stream
        )

        self.stop_button = QtWidgets.QPushButton(
            "Stop"
        )
        self.stop_button.clicked.connect(
            self._stop_stream
        )

        stream_layout.addWidget(
            self.rtsp_url_edit,
            stretch=1,
        )
        stream_layout.addWidget(
            self.play_button
        )
        stream_layout.addWidget(
            self.stop_button
        )

        # --------------------------------------------------------------
        # Video
        # --------------------------------------------------------------
        self.video_widget = RtspVideoWidget()
        self.video_widget.clicked.connect(
            self._on_video_clicked
        )

        # --------------------------------------------------------------
        # Video controls
        # --------------------------------------------------------------
        controls_row = QtWidgets.QHBoxLayout()

        self.track_checkbox = QtWidgets.QCheckBox(
            "Track"
        )
        self.track_checkbox.setToolTip(
            "Unchecked: click moves camera. "
            "Checked: click begins object tracking."
        )
        self.track_checkbox.stateChanged.connect(
            self._on_track_toggled
        )

        self.stop_tracking_button = QtWidgets.QPushButton(
            "Stop Tracking"
        )
        self.stop_tracking_button.clicked.connect(
            self._stop_tracking
        )

        self.mode_label = QtWidgets.QLabel(
            "Click mode: Point Camera"
        )
        self.mode_label.setStyleSheet(
            "font-weight: 600;"
        )

        self.status_label = QtWidgets.QLabel(
            "Ready"
        )
        self.status_label.setStyleSheet(
            "color: #38bdf8;"
        )

        controls_row.addWidget(
            self.track_checkbox
        )
        controls_row.addWidget(
            self.stop_tracking_button
        )
        controls_row.addWidget(
            self.mode_label
        )
        controls_row.addStretch(1)
        controls_row.addWidget(
            self.status_label
        )

        layout.addWidget(conn_group)
        layout.addWidget(geo_group)
        layout.addWidget(stream_group)
        layout.addWidget(
            self.video_widget,
            stretch=1,
        )
        layout.addLayout(controls_row)

    def _apply_defaults(
        self,
        args: argparse.Namespace,
    ) -> None:
        self.role_combo.setCurrentText(
            args.remote_mode
        )
        self.host_edit.setText(
            args.remote_host
        )
        self.port_spin.setValue(
            args.remote_port
        )
        self.token_edit.setText(
            args.remote_token
        )

        default_rtsp_url = (
            f"rtsp://{ConnectionConfig.UDP_IP_TARGET}:"
            f"{ConnectionConfig.RTSP_PORT_TARGET}/"
            f"{ConnectionConfig.RTSP_PATH_TARGET}"
        )
        self.rtsp_url_edit.setText(
            default_rtsp_url
        )

    # ------------------------------------------------------------------
    # Bridge
    # ------------------------------------------------------------------

    def _toggle_bridge(self) -> None:
        if self.bridge is not None:
            self._disconnect_bridge()
            return

        cfg = BridgeConfig(
            role=self.role_combo.currentText(),
            host=self.host_edit.text().strip(),
            port=self.port_spin.value(),
            token=self.token_edit.text().strip(),
            connect_timeout=RemoteBridgeConfig.CONNECT_TIMEOUT,
            ack_timeout=RemoteBridgeConfig.ACK_TIMEOUT,
            retry_count=RemoteBridgeConfig.RETRY_COUNT,
            reconnect_interval=RemoteBridgeConfig.RECONNECT_INTERVAL,
        )

        self.bridge = TcpCommandBridge(
            cfg,
            logger=self._bridge_log,
        )

        # Worker-thread callbacks -> Qt signals -> GUI thread.
        self.bridge.set_event_handler(
            lambda event_name, data:
                self.bridge_signals.event_received.emit(
                    event_name,
                    data,
                )
        )

        self.bridge.set_connection_handler(
            lambda connected:
                self.bridge_signals.connection_changed.emit(
                    connected
                )
        )

        self.bridge.start()

        self.connect_button.setText(
            "Disconnect Bridge"
        )

        if cfg.role == "connect":
            connected = self.bridge.wait_until_connected(
                RemoteBridgeConfig.CONNECT_TIMEOUT
            )

            if not connected:
                self.status_label.setText(
                    "Bridge connect timeout"
                )
                self._disconnect_bridge()
                return

        if self.bridge.is_connected():
            self._mark_bridge_connected()
        else:
            self.bridge_status.setText(
                "Waiting for peer"
            )
            self.bridge_status.setStyleSheet(
                "color: #f59e0b; font-weight: 600;"
            )
            self.status_label.setText(
                "Bridge listening - waiting for executor"
            )

    @QtCore.Slot(bool)
    def _on_bridge_connection_changed(
        self,
        connected: bool,
    ) -> None:
        if connected:
            self._mark_bridge_connected()
        else:
            self._mark_bridge_disconnected()

    def _mark_bridge_connected(self) -> None:
        was_connected = self.bridge_connected

        self.bridge_connected = True

        self.connect_button.setText(
            "Disconnect Bridge"
        )
        self.bridge_status.setText(
            "Connected"
        )
        self.bridge_status.setStyleSheet(
            "color: #16a34a; font-weight: 600;"
        )
        self.status_label.setText(
            "Bridge ready"
        )

        if not was_connected:
            self._begin_geo_wait()

            # Avoid calling a blocking ACK request directly inside a
            # connection callback.
            QtCore.QTimer.singleShot(
                0,
                self._sync_tracking_mode,
            )

    def _mark_bridge_disconnected(self) -> None:
        self.bridge_connected = False

        self.bridge_status.setText(
            "Reconnecting..."
        )
        self.bridge_status.setStyleSheet(
            "color: #f59e0b; font-weight: 600;"
        )
        self.status_label.setText(
            "Remote bridge disconnected"
        )

        self._reset_geo_state(
            "Geolocation: bridge disconnected"
        )

    def _sync_tracking_mode(self) -> None:
        if not self.bridge_connected:
            return

        self._send_remote_command(
            CMD_PAYLOAD_TRACK,
            [
                1
                if self.track_checkbox.isChecked()
                else 0
            ],
        )

    def _disconnect_bridge(self) -> None:
        bridge = self.bridge
        self.bridge = None

        if bridge is not None:
            bridge.set_event_handler(None)
            bridge.set_connection_handler(None)
            bridge.stop()

        self.bridge_connected = False
        self.tracking_enabled = False

        self.connect_button.setText(
            "Connect Bridge"
        )
        self.bridge_status.setText(
            "Disconnected"
        )
        self.bridge_status.setStyleSheet(
            "color: #e11d48; font-weight: 600;"
        )

        self._reset_geo_state(
            "Geolocation: waiting for remote bridge"
        )

    def _send_remote_command(
        self,
        command: str,
        params: List,
    ) -> Tuple[bool, str]:
        if (
            not self.bridge_connected
            or self.bridge is None
            or not self.bridge.is_connected()
        ):
            detail = (
                f"Bridge offline, skipped {command}"
            )
            self.status_label.setText(detail)
            return False, detail

        ok, detail = self.bridge.send_command(
            command=command,
            params=params,
            ack_required=True,
        )

        if ok:
            self.status_label.setText(
                f"ACK {command}: {detail}"
            )
        else:
            self.status_label.setText(
                f"NACK {command}: {detail}"
            )

        return ok, detail

    def _bridge_log(
        self,
        msg: str,
    ) -> None:
        print(msg)

    # ------------------------------------------------------------------
    # Optional geolocation
    # ------------------------------------------------------------------

    def _begin_geo_wait(self) -> None:
        self.geo_wait_started = time.monotonic()
        self.geo_timeout_notified = False

        self.ardupilot_connected = False
        self.gps_valid = False
        self.geolocation_available = False
        self.last_geo_event_time = None

        self.target_lat = None
        self.target_lon = None
        self.target_alt = None

        self.ardupilot_status_label.setText(
            "ArduPilot: waiting"
        )
        self.gps_status_label.setText(
            "GPS: waiting"
        )
        self.target_position_label.setText(
            "Target: --"
        )

        self._set_geo_status_waiting(
            f"Waiting up to {self.geo_timeout_s:.1f}s "
            "for ArduPilot/GPS. "
            "Pointing and tracking remain available."
        )

    def _reset_geo_state(
        self,
        message: str,
    ) -> None:
        self.geo_wait_started = None
        self.geo_timeout_notified = False

        self.ardupilot_connected = False
        self.gps_valid = False
        self.geolocation_available = False
        self.last_geo_event_time = None

        self.target_lat = None
        self.target_lon = None
        self.target_alt = None

        self.ardupilot_status_label.setText(
            "ArduPilot: --"
        )
        self.gps_status_label.setText(
            "GPS: --"
        )
        self.target_position_label.setText(
            "Target: --"
        )

        self._set_geo_status_neutral(
            message
        )

    @QtCore.Slot(str, object)
    def _on_bridge_event(
        self,
        event_name: str,
        data_obj: object,
    ) -> None:
        if event_name != EVENT_GEO_STATUS:
            return

        data = (
            data_obj
            if isinstance(data_obj, dict)
            else {}
        )

        self.last_geo_event_time = time.monotonic()

        self.ardupilot_connected = bool(
            data.get(
                "ardupilot_connected",
                False,
            )
        )

        self.gps_valid = bool(
            data.get(
                "gps_valid",
                False,
            )
        )

        if "geolocation_available" in data:
            self.geolocation_available = bool(
                data.get(
                    "geolocation_available"
                )
            )
        else:
            self.geolocation_available = (
                self.ardupilot_connected
                and self.gps_valid
            )

        self.ardupilot_status_label.setText(
            "ArduPilot: Connected"
            if self.ardupilot_connected
            else "ArduPilot: Not connected"
        )

        self.gps_status_label.setText(
            "GPS: Valid"
            if self.gps_valid
            else "GPS: Unavailable"
        )

        target_lat = self._optional_finite_float(
            data.get("target_lat")
        )
        target_lon = self._optional_finite_float(
            data.get("target_lon")
        )
        target_alt = self._optional_finite_float(
            data.get("target_alt")
        )

        if target_lat is not None:
            self.target_lat = target_lat

        if target_lon is not None:
            self.target_lon = target_lon

        if target_alt is not None:
            self.target_alt = target_alt

        self._refresh_target_position_label()

        detail = str(
            data.get("detail", "")
        ).strip()

        if self.geolocation_available:
            self.geo_timeout_notified = False

            self._set_geo_status_ok(
                detail
                or "Geolocation available"
            )
            return

        if not self._geo_deadline_expired():
            if self.ardupilot_connected:
                self._set_geo_status_waiting(
                    detail
                    or (
                        "ArduPilot connected; "
                        "waiting for valid GPS/geolocation."
                    )
                )
            else:
                self._set_geo_status_waiting(
                    detail
                    or (
                        f"Waiting for ArduPilot "
                        f"({self.geo_timeout_s:.1f}s grace period)."
                    )
                )
            return

        self._notify_geo_unavailable_once(
            detail
        )

    def _update_geo_timeout_state(self) -> None:
        if not self.bridge_connected:
            return

        if self.geolocation_available:
            return

        if not self._geo_deadline_expired():
            return

        self._notify_geo_unavailable_once("")

    def _geo_deadline_expired(self) -> bool:
        if self.geo_wait_started is None:
            return False

        return (
            time.monotonic()
            - self.geo_wait_started
            >= self.geo_timeout_s
        )

    def _notify_geo_unavailable_once(
        self,
        detail: str,
    ) -> None:
        if self.ardupilot_connected:
            message = (
                "Geolocation unavailable: ArduPilot is connected, "
                "but valid GPS/target geolocation was not received. "
                "Pointing and tracking are still available."
            )
        else:
            message = (
                "Geolocation unavailable: no ArduPilot "
                f"connection/status after {self.geo_timeout_s:.1f}s. "
                "Pointing and tracking are still available."
            )

        if detail:
            message = f"{message} {detail}"

        self._set_geo_status_warning(
            message
        )

        # Avoid spamming the same warning every timer tick.
        if not self.geo_timeout_notified:
            self.geo_timeout_notified = True
            print(
                f"[GEO] {message}"
            )

    def _refresh_target_position_label(
        self,
    ) -> None:
        if (
            self.target_lat is None
            or self.target_lon is None
        ):
            if self.geolocation_available:
                self.target_position_label.setText(
                    "Target: waiting for target coordinate"
                )
            else:
                self.target_position_label.setText(
                    "Target: --"
                )
            return

        text = (
            f"Target: {self.target_lat:.7f}, "
            f"{self.target_lon:.7f}"
        )

        if self.target_alt is not None:
            text += (
                f", alt={self.target_alt:.1f} m"
            )

        self.target_position_label.setText(
            text
        )

    @staticmethod
    def _optional_finite_float(
        value,
    ) -> Optional[float]:
        if value is None:
            return None

        try:
            number = float(value)
        except (TypeError, ValueError):
            return None

        if not math.isfinite(number):
            return None

        return number

    def _set_geo_status_ok(
        self,
        message: str,
    ) -> None:
        self.geo_status_label.setText(
            f"Geolocation: {message}"
        )
        self.geo_status_label.setStyleSheet(
            "color: #16a34a; font-weight: 600;"
        )

    def _set_geo_status_waiting(
        self,
        message: str,
    ) -> None:
        self.geo_status_label.setText(
            f"Geolocation: {message}"
        )
        self.geo_status_label.setStyleSheet(
            "color: #f59e0b; font-weight: 600;"
        )

    def _set_geo_status_warning(
        self,
        message: str,
    ) -> None:
        self.geo_status_label.setText(
            f"WARNING: {message}"
        )
        self.geo_status_label.setStyleSheet(
            "color: #f59e0b; font-weight: 700;"
        )

    def _set_geo_status_neutral(
        self,
        message: str,
    ) -> None:
        self.geo_status_label.setText(
            message
        )
        self.geo_status_label.setStyleSheet(
            "color: #94a3b8; font-weight: 600;"
        )

    # ------------------------------------------------------------------
    # RTSP
    # ------------------------------------------------------------------

    def _start_stream(self) -> None:
        url = self.rtsp_url_edit.text().strip()

        if not url:
            self.status_label.setText(
                "RTSP URL is empty"
            )
            return

        ok = self.video_widget.start_stream(
            url
        )

        self.status_label.setText(
            "Stream playing"
            if ok
            else "Failed to open stream"
        )

    def _stop_stream(self) -> None:
        self.video_widget.stop_stream()
        self.status_label.setText(
            "Stream stopped"
        )

    # ------------------------------------------------------------------
    # Tracking mode
    # ------------------------------------------------------------------

    def _on_track_toggled(
        self,
        state: int,
    ) -> None:
        enabled = (
            state
            == QtCore.Qt.CheckState.Checked.value
        )

        self.tracking_enabled = enabled

        if enabled:
            self.mode_label.setText(
                "Click mode: Track Target"
            )
            self.status_label.setText(
                "Track enabled - click a target"
            )
        else:
            self.mode_label.setText(
                "Click mode: Point Camera"
            )
            self.status_label.setText(
                "Track disabled - click to point camera"
            )

        if self.bridge_connected:
            self._send_remote_command(
                CMD_PAYLOAD_TRACK,
                [1 if enabled else 0],
            )

    def _stop_tracking(self) -> None:
        if self.bridge_connected:
            ok, _ = self._send_remote_command(
                CMD_PAYLOAD_TRACK,
                [0],
            )
        else:
            ok = False

        self.tracking_enabled = False

        self.track_checkbox.blockSignals(
            True
        )
        self.track_checkbox.setChecked(
            False
        )
        self.track_checkbox.blockSignals(
            False
        )

        self.mode_label.setText(
            "Click mode: Point Camera"
        )

        if ok:
            self.status_label.setText(
                "Tracking stopped"
            )

    # ------------------------------------------------------------------
    # Coordinate conversion
    # ------------------------------------------------------------------

    def _widget_point_to_payload(
        self,
        x_widget: float,
        y_widget: float,
        frame_w: int,
        frame_h: int,
    ) -> Optional[Tuple[int, int]]:
        if frame_w <= 0 or frame_h <= 0:
            return None

        widget_w = self.video_widget.width()
        widget_h = self.video_widget.height()

        if widget_w <= 0 or widget_h <= 0:
            return None

        # Qt KeepAspectRatio scaling used by the video widget.
        scale = min(
            widget_w / frame_w,
            widget_h / frame_h,
        )

        displayed_w = frame_w * scale
        displayed_h = frame_h * scale

        offset_x = (
            widget_w - displayed_w
        ) / 2.0

        offset_y = (
            widget_h - displayed_h
        ) / 2.0

        # Ignore clicks in letterbox/pillarbox space.
        if (
            x_widget < offset_x
            or x_widget >= offset_x + displayed_w
            or y_widget < offset_y
            or y_widget >= offset_y + displayed_h
        ):
            return None

        # Widget -> actual RTSP source frame.
        x_src = (
            x_widget - offset_x
        ) / scale

        y_src = (
            y_widget - offset_y
        ) / scale

        # Source frame -> Gremsy's fixed 1920 x 1080 coordinate space.
        x_payload = round(
            (x_src / frame_w)
            * GREMSY_FRAME_W
        )

        y_payload = round(
            (y_src / frame_h)
            * GREMSY_FRAME_H
        )

        x_payload = max(
            0,
            min(
                GREMSY_FRAME_W - 1,
                x_payload,
            ),
        )

        y_payload = max(
            0,
            min(
                GREMSY_FRAME_H - 1,
                y_payload,
            ),
        )

        return (
            x_payload,
            y_payload,
        )

    # ------------------------------------------------------------------
    # Click handler
    # ------------------------------------------------------------------

    def _on_video_clicked(
        self,
        x_widget: float,
        y_widget: float,
        frame_w: int,
        frame_h: int,
    ) -> None:
        point = self._widget_point_to_payload(
            x_widget,
            y_widget,
            frame_w,
            frame_h,
        )

        if point is None:
            self.status_label.setText(
                "Click inside the actual video image"
            )
            return

        x_payload, y_payload = point

        action = (
            "TRACK"
            if self.tracking_enabled
            else "POINT"
        )

        print(
            f"[{action}] "
            f"widget=({x_widget:.1f}, {y_widget:.1f}) "
            f"Gremsy=({x_payload}, {y_payload})"
        )

        # Backend decides whether this means move-only or track,
        # based on PAYLOAD_TRACK mode previously sent.
        ok, _ = self._send_remote_command(
            CMD_PAYLOAD_TOUCH,
            [
                x_payload,
                y_payload,
            ],
        )

        if not ok:
            return

        if self.tracking_enabled:
            suffix = (
                " | geolocation enabled"
                if self.geolocation_available
                else " | geolocation unavailable"
            )

            self.status_label.setText(
                f"Tracking target at "
                f"({x_payload}, {y_payload})"
                f"{suffix}"
            )
        else:
            self.status_label.setText(
                f"Camera moving to "
                f"({x_payload}, {y_payload})"
            )

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def closeEvent(
        self,
        event: QtGui.QCloseEvent,
    ) -> None:
        if self.bridge_connected:
            try:
                self._send_remote_command(
                    CMD_PAYLOAD_TRACK,
                    [0],
                )
            except Exception:
                pass

        self.geo_timer.stop()
        self.video_widget.stop_stream()
        self._disconnect_bridge()

        super().closeEvent(event)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Payload PyQt UI - click to point or click to track"
        )
    )

    parser.add_argument(
        "--remote-mode",
        choices=[
            "connect",
            "listen",
        ],
        default="connect",
        help="Remote bridge role for this UI process",
    )

    parser.add_argument(
        "--remote-host",
        default=RemoteBridgeConfig.HOST,
    )

    parser.add_argument(
        "--remote-port",
        type=int,
        default=RemoteBridgeConfig.PORT,
    )

    parser.add_argument(
        "--remote-token",
        default=RemoteBridgeConfig.TOKEN,
    )

    parser.add_argument(
        "--geo-timeout",
        type=float,
        default=DEFAULT_GEO_TIMEOUT_S,
        help=(
            "Seconds to wait for ArduPilot/GPS status after the "
            "remote bridge connects. Geolocation is optional and "
            "pointing/tracking continue after timeout."
        ),
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    app = QtWidgets.QApplication(
        sys.argv
    )

    window = MainWindow(args)
    window.show()

    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
