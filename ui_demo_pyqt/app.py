#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""
PyQt remote Gremsy Lynx UI.

Capabilities
------------
- Track unchecked:
    click video -> point camera at image pixel.
- Track checked:
    click video -> acquire/track object.
- Hold Zoom In / Zoom Out:
    continuous camera zoom; release stops zoom.
- Polls GET_GEO_STATUS from remote_executor.py.
- Shows:
    * ArduPilot/GPS state
    * vehicle roll/pitch/yaw
    * gimbal roll/pitch/yaw
    * tracking pixel/FOV
    * approximate target GPS from flat-ground LOS intersection

The target GPS shown here is an ESTIMATE, not a surveyed fix.
"""

import argparse
import json
import math
import os
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

from PySide6 import QtCore, QtGui, QtWidgets

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

try:
    import mgrs as _mgrs_lib

    _MGRS = _mgrs_lib.MGRS()
except ImportError:  # ponytail: optional dep, UI still runs without it
    _MGRS = None


def to_mgrs(lat: float, lon: float) -> str:
    """Lat/lon (deg) -> MGRS string at 1 m precision, or a short reason."""
    if _MGRS is None:
        return "MGRS n/a (pip install mgrs)"
    try:
        out = _MGRS.toMGRS(lat, lon, MGRSPrecision=5)
        out = out.decode() if isinstance(out, bytes) else out
        return f"{out[:3]} {out[3:5]} {out[5:10]} {out[10:]}"

    except Exception:  # polar/invalid coords
        return "MGRS n/a"


CMD_PAYLOAD_TOUCH = "PAYLOAD_TOUCH"
CMD_PAYLOAD_TRACK = "PAYLOAD_TRACK"
CMD_PAYLOAD_ZOOM_IN = "PAYLOAD_ZOOM_IN"
CMD_PAYLOAD_ZOOM_OUT = "PAYLOAD_ZOOM_OUT"
CMD_PAYLOAD_ZOOM_STOP = "PAYLOAD_ZOOM_STOP"
CMD_GET_GEO_STATUS = "GET_GEO_STATUS"
CMD_PAYLOAD_CAMERA_PARAM = "PAYLOAD_CAMERA_PARAM"
CMD_PAYLOAD_RECORD = "PAYLOAD_RECORD"
CMD_PAYLOAD_GIMBAL_MODE = "PAYLOAD_GIMBAL_MODE"
CMD_PAYLOAD_GIMBAL_LEVEL_ROLL = "PAYLOAD_GIMBAL_LEVEL_ROLL"
CMD_PAYLOAD_GIMBAL_YAW_HEADING = "PAYLOAD_GIMBAL_YAW_HEADING"
CMD_PAYLOAD_GIMBAL_HEADING_HOLD = "PAYLOAD_GIMBAL_HEADING_HOLD"

GIMBAL_MODE_RESET = 4

# (label, payload param value)
# ponytail: Lynx (Guide sensor) names; FLIR-sensor payloads map the same ids
# to different palettes (see payload_camera_ir_palette in libs/*_define.py).
IR_PALETTES = [
    ("WhiteHot", 0),
    ("Fulgurite", 1),
    ("IronRed", 2),
    ("HotIron", 3),
    ("Medical", 4),
    ("Arctic", 5),
    ("Rainbow1", 6),
    ("Rainbow2", 7),
    ("Tint", 8),
    ("BlackHot", 9),
]

GREMSY_FRAME_W = 1920
GREMSY_FRAME_H = 1080

DEFAULT_GEO_TIMEOUT_S = 8.0


class GeoPollSignals(QtCore.QObject):
    result = QtCore.Signal(bool, str)


class ElidedLabel(QtWidgets.QLabel):
    """QLabel that never widens its parent: long text is elided with an
    ellipsis and the full text is shown in the tooltip."""

    def __init__(self, text: str = "", parent=None):
        super().__init__(parent)
        self._full = ""
        self.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Ignored,
            QtWidgets.QSizePolicy.Policy.Preferred,
        )
        self.setText(text)

    def setText(self, text: str) -> None:  # noqa: N802 (Qt override)
        self._full = text
        self.setToolTip(text)
        self._elide()

    def text(self) -> str:
        return self._full

    def resizeEvent(self, event) -> None:
        self._elide()
        super().resizeEvent(event)

    def _elide(self) -> None:
        super().setText(
            self.fontMetrics().elidedText(
                self._full, QtCore.Qt.TextElideMode.ElideRight, max(self.width(), 1)
            )
        )


class CompassWidget(QtWidgets.QWidget):
    """Compass with one gimbal needle + FOV wedge.

    heading-up (default): rose rotates with the aircraft, nose is always up.
    north-up: rose fixed with N at top, aircraft icon rotates to its heading.
    Clicking the dial emits the absolute heading of the nearest 45° sector.
    """

    headingClicked = QtCore.Signal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.vehicle_heading: Optional[float] = None
        self.gimbal_heading: Optional[float] = None
        self.hfov: Optional[float] = None
        self.north_up = False
        self.hover_heading: Optional[float] = None
        self.setFixedSize(130, 130)
        self.setMouseTracking(True)

    def set_north_up(self, enabled: bool) -> None:
        self.north_up = bool(enabled)
        self.update()

    def heading_at(self, x: float, y: float) -> Optional[float]:
        """Absolute heading of the 45° sector under widget pixel (x, y),
        or None when the click is outside the dial or at its centre."""
        dx, dy = x - self.width() / 2, y - (self.height() / 2 - 4)
        r = min(self.width(), self.height()) / 2 - 12
        if dx * dx + dy * dy > r * r or dx * dx + dy * dy < 100:
            return None
        screen = math.degrees(math.atan2(dx, -dy))  # 0 = up, clockwise
        if not self.north_up:
            screen += self.vehicle_heading or 0.0
        return round(screen / 45.0) * 45.0 % 360.0

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            hdg = self.heading_at(event.position().x(), event.position().y())
            if hdg is not None:
                self.headingClicked.emit(hdg)
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:
        hdg = self.heading_at(event.position().x(), event.position().y())
        if hdg != self.hover_heading:
            self.hover_heading = hdg
            self.update()
        super().mouseMoveEvent(event)

    def leaveEvent(self, event) -> None:
        self.hover_heading = None
        self.update()
        super().leaveEvent(event)

    def set_headings(
        self,
        vehicle: Optional[float],
        gimbal: Optional[float],
        hfov: Optional[float],
    ) -> None:
        self.vehicle_heading = vehicle
        self.gimbal_heading = gimbal
        self.hfov = hfov
        self.update()

    def paintEvent(self, _event) -> None:
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        r = min(w, h) / 2 - 12
        p.translate(w / 2, h / 2 - 4)

        hdg_val = self.vehicle_heading or 0.0
        # heading-up: rose turns by -heading, icon fixed, needle relative.
        # north-up: rose fixed, icon turns by heading, needle absolute.
        rose_rot = 0.0 if self.north_up else -hdg_val
        icon_rot = hdg_val if self.north_up else 0.0

        # Hovered 45° sector, drawn in screen space under everything else.
        if self.hover_heading is not None:
            p.save()
            p.rotate(self.hover_heading + rose_rot)
            p.setPen(QtCore.Qt.PenStyle.NoPen)
            p.setBrush(QtGui.QColor(56, 189, 248, 60))
            p.drawPie(QtCore.QRectF(-r, -r, 2 * r, 2 * r), int((90 - 22.5) * 16), 45 * 16)
            p.restore()

        # Rose.
        p.save()
        p.rotate(rose_rot)
        p.setPen(QtGui.QPen(QtGui.QColor("#94a3b8"), 1.5))
        p.drawEllipse(QtCore.QPointF(0, 0), r, r)
        for deg in range(0, 360, 30):
            p.save()
            p.rotate(deg)
            tick = 8 if deg % 90 == 0 else 4
            p.drawLine(QtCore.QPointF(0, -r), QtCore.QPointF(0, -r + tick))
            p.restore()
        font = p.font()
        font.setPointSize(8)
        font.setBold(True)
        p.setFont(font)
        for deg, letter in ((0, "N"), (90, "E"), (180, "S"), (270, "W")):
            a = math.radians(deg + rose_rot)
            x, y = (r - 14) * math.sin(a), -(r - 14) * math.cos(a)
            p.save()
            p.rotate(-rose_rot)  # keep letters upright
            p.setPen(QtGui.QColor("#ef4444" if letter == "N" else "#cbd5e1"))
            p.drawText(QtCore.QRectF(x - 8, y - 8, 16, 16), QtCore.Qt.AlignmentFlag.AlignCenter, letter)
            p.restore()
        p.restore()

        # Gimbal wedge + needle.
        if self.gimbal_heading is not None and self.vehicle_heading is not None:
            p.save()
            p.rotate(self.gimbal_heading - self.vehicle_heading + icon_rot)
            if self.hfov is not None and self.hfov > 0:
                p.setPen(QtCore.Qt.PenStyle.NoPen)
                p.setBrush(QtGui.QColor(245, 158, 11, 70))
                rect = QtCore.QRectF(-r, -r, 2 * r, 2 * r)
                p.drawPie(rect, int((90 - self.hfov / 2) * 16), int(self.hfov * 16))
            p.setPen(QtGui.QPen(QtGui.QColor("#f59e0b"), 2.5))
            p.drawLine(QtCore.QPointF(0, 0), QtCore.QPointF(0, -r))
            p.restore()

        # Aircraft icon.
        p.save()
        p.rotate(icon_rot)
        p.setPen(QtCore.Qt.PenStyle.NoPen)
        p.setBrush(QtGui.QColor("#38bdf8" if self.vehicle_heading is not None else "#64748b"))
        p.drawPolygon(QtGui.QPolygonF([
            QtCore.QPointF(0, -12),
            QtCore.QPointF(-7, 8),
            QtCore.QPointF(0, 4),
            QtCore.QPointF(7, 8),
        ]))
        p.restore()

        # Numeric readout.
        p.setPen(QtGui.QColor("#cbd5e1"))
        hdg = "HDG ---" if self.vehicle_heading is None else f"HDG {self.vehicle_heading % 360:03.0f}"
        p.drawText(QtCore.QRectF(-w / 2, r + 2, w, 14), QtCore.Qt.AlignmentFlag.AlignCenter, hdg)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(
        self,
        args: argparse.Namespace,
    ):
        super().__init__()

        self.setWindowTitle(
            "Payload UI Demo - Click / Track / Zoom / Target Geo"
        )
        self.resize(1450, 930)

        self.bridge: Optional[
            TcpCommandBridge
        ] = None
        self.bridge_connected = False
        self.tracking_enabled = False
        self.zoom_active = False
        # True while the RTSP URL ends in /ir (IR sensor view).
        self.ir_stream = False

        self.geo_timeout_s = max(
            0.0,
            float(args.geo_timeout),
        )
        self.geo_wait_started: Optional[
            float
        ] = None
        self.geo_timeout_notified = False

        self._geo_poll_busy = False
        self._geo_poll_lock = (
            threading.Lock()
        )
        self._geo_signals = GeoPollSignals()
        self._geo_signals.result.connect(
            self._on_geo_poll_result
        )

        self._build_ui()
        self._apply_defaults(args)

        self.geo_timer = QtCore.QTimer(self)
        self.geo_timer.setInterval(500)
        self.geo_timer.timeout.connect(
            self._poll_geo_status
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
            12, 12, 12, 12
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
        self.port_spin.setRange(
            1, 65535
        )

        self.token_edit = QtWidgets.QLineEdit()
        self.token_edit.setEchoMode(
            QtWidgets.QLineEdit.EchoMode.Password
        )

        self.connect_button = (
            QtWidgets.QPushButton(
                "Connect Bridge"
            )
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
            0, 0,
        )
        conn_grid.addWidget(
            self.role_combo,
            0, 1,
        )
        conn_grid.addWidget(
            QtWidgets.QLabel("Host"),
            0, 2,
        )
        conn_grid.addWidget(
            self.host_edit,
            0, 3,
        )
        conn_grid.addWidget(
            QtWidgets.QLabel("Port"),
            0, 4,
        )
        conn_grid.addWidget(
            self.port_spin,
            0, 5,
        )
        conn_grid.addWidget(
            QtWidgets.QLabel("Token"),
            0, 6,
        )
        conn_grid.addWidget(
            self.token_edit,
            0, 7,
        )
        conn_grid.addWidget(
            self.connect_button,
            0, 8,
        )
        conn_grid.addWidget(
            self.bridge_status,
            0, 9,
        )

        # --------------------------------------------------------------
        # Target geolocation telemetry
        # --------------------------------------------------------------
        geo_group = QtWidgets.QGroupBox(
            "Approximate Target Geolocation"
        )
        geo_grid = QtWidgets.QGridLayout(
            geo_group
        )

        self.geo_status_label = QtWidgets.QLabel(
            "Estimator: waiting for bridge"
        )
        self.geo_status_label.setWordWrap(
            True
        )
        self.geo_status_label.setStyleSheet(
            "color: #94a3b8; font-weight: 600;"
        )

        self.ardupilot_status_label = (
            QtWidgets.QLabel(
                "ArduPilot: --"
            )
        )
        self.gps_status_label = QtWidgets.QLabel(
            "GPS: --"
        )

        self.vehicle_position_label = (
            QtWidgets.QLabel(
                "Vehicle: --"
            )
        )

        self.vehicle_attitude_label = (
            QtWidgets.QLabel(
                "Vehicle attitude: --"
            )
        )

        self.gimbal_attitude_label = (
            QtWidgets.QLabel(
                "Gimbal attitude: --"
            )
        )

        self.track_info_label = QtWidgets.QLabel(
            "Track: --"
        )

        self.target_position_label = (
            QtWidgets.QLabel(
                "Estimated target: --"
            )
        )
        self.target_position_label.setStyleSheet(
            "font-weight: 700;"
        )
        self.target_position_label.setTextInteractionFlags(
            QtCore.Qt.TextInteractionFlag.TextSelectableByMouse
        )

        self.range_label = QtWidgets.QLabel(
            "Geometry: --"
        )

        self.native_target_label = QtWidgets.QLabel(
            "Gremsy TARGET_*: --"
        )
        self.native_target_label.setStyleSheet(
            "color: #64748b;"
        )

        geo_grid.addWidget(
            self.geo_status_label,
            0, 0, 1, 4,
        )
        geo_grid.addWidget(
            self.ardupilot_status_label,
            1, 0,
        )
        geo_grid.addWidget(
            self.gps_status_label,
            1, 1,
        )
        geo_grid.addWidget(
            self.vehicle_position_label,
            1, 2, 1, 2,
        )

        geo_grid.addWidget(
            self.vehicle_attitude_label,
            2, 0, 1, 2,
        )
        geo_grid.addWidget(
            self.gimbal_attitude_label,
            2, 2, 1, 2,
        )

        geo_grid.addWidget(
            self.track_info_label,
            3, 0, 1, 2,
        )
        geo_grid.addWidget(
            self.range_label,
            3, 2, 1, 2,
        )

        geo_grid.addWidget(
            self.target_position_label,
            4, 0, 1, 4,
        )
        geo_grid.addWidget(
            self.native_target_label,
            5, 0, 1, 4,
        )

        self.compass = CompassWidget()
        self.compass.setToolTip("Click a sector to yaw the gimbal to that heading.")
        self.compass.setCursor(QtCore.Qt.CursorShape.PointingHandCursor)
        self.compass.headingClicked.connect(
            lambda hdg: self._send_remote_command(CMD_PAYLOAD_GIMBAL_YAW_HEADING, [hdg])
        )
        geo_grid.addWidget(
            self.compass,
            0, 4, 6, 1,
            QtCore.Qt.AlignmentFlag.AlignTop,
        )
        self.north_up_button = QtWidgets.QPushButton("North up")
        self.north_up_button.setCheckable(True)
        self.north_up_button.setToolTip(
            "Checked: N fixed at top, aircraft icon rotates.\n"
            "Unchecked: nose fixed at top, rose rotates."
        )
        self.north_up_button.toggled.connect(self.compass.set_north_up)
        geo_grid.addWidget(
            self.north_up_button,
            6, 4,
            QtCore.Qt.AlignmentFlag.AlignHCenter,
        )
        self.hold_heading_checkbox = QtWidgets.QCheckBox("Hold heading")
        self.hold_heading_checkbox.setToolTip(
            "Keep the gimbal on the clicked compass heading as the aircraft yaws.\n"
            "Reset Gimbal turns it off."
        )
        self.hold_heading_checkbox.toggled.connect(
            lambda on: self._send_remote_command(
                CMD_PAYLOAD_GIMBAL_HEADING_HOLD, [1 if on else 0]
            )
        )
        geo_grid.addWidget(
            self.hold_heading_checkbox,
            7, 4,
            QtCore.Qt.AlignmentFlag.AlignHCenter,
        )
        geo_grid.setColumnStretch(3, 1)

        # --------------------------------------------------------------
        # RTSP
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
        # Controls
        # --------------------------------------------------------------
        controls_row = QtWidgets.QHBoxLayout()

        self.track_checkbox = QtWidgets.QCheckBox(
            "Track"
        )
        self.track_checkbox.setToolTip(
            "Unchecked: click points camera. "
            "Checked: click acquires/tracks target."
        )
        self.track_checkbox.stateChanged.connect(
            self._on_track_toggled
        )

        self.stop_tracking_button = (
            QtWidgets.QPushButton(
                "Stop Tracking"
            )
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

        self.zoom_out_button = (
            QtWidgets.QPushButton(
                "Zoom Out -"
            )
        )
        self.zoom_out_button.setToolTip(
            "Press and hold to zoom out."
        )
        self.zoom_out_button.pressed.connect(
            self._zoom_out_pressed
        )
        self.zoom_out_button.released.connect(
            self._zoom_released
        )

        self.zoom_in_button = (
            QtWidgets.QPushButton(
                "Zoom In +"
            )
        )
        self.zoom_in_button.setToolTip(
            "Press and hold to zoom in."
        )
        self.zoom_in_button.pressed.connect(
            self._zoom_in_pressed
        )
        self.zoom_in_button.released.connect(
            self._zoom_released
        )

        self.palette_combo = QtWidgets.QComboBox()
        self.palette_combo.setToolTip("IR color palette")
        for label, value in IR_PALETTES:
            self.palette_combo.addItem(label, value)
        self.palette_combo.activated.connect(
            lambda _i: self._send_remote_command(
                CMD_PAYLOAD_CAMERA_PARAM,
                ["ir_palette", self.palette_combo.currentData()],
            )
        )

        self.record_button = QtWidgets.QPushButton("Record")
        self.record_button.setCheckable(True)
        self.record_button.setToolTip(
            "Record EO + IR to the camera's SD card. Stream is unaffected."
        )
        self.record_button.setStyleSheet(
            "QPushButton:checked { background: #dc2626; color: white; }"
        )
        self.record_button.clicked.connect(self._on_record_clicked)

        self.reset_gimbal_button = QtWidgets.QPushButton("Reset Gimbal")
        self.reset_gimbal_button.setToolTip("Recenter all gimbal axes.")
        self.reset_gimbal_button.clicked.connect(
            lambda: self._send_remote_command(
                CMD_PAYLOAD_GIMBAL_MODE, [GIMBAL_MODE_RESET]
            )
        )

        self.level_roll_button = QtWidgets.QPushButton("Level Roll")
        self.level_roll_button.setToolTip(
            "Command roll to 0, keeping current pitch and yaw."
        )
        self.level_roll_button.clicked.connect(
            lambda: self._send_remote_command(CMD_PAYLOAD_GIMBAL_LEVEL_ROLL, [])
        )

        self.status_label = ElidedLabel("Ready")
        self.status_label.setStyleSheet(
            "color: #38bdf8;"
        )
        self.status_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight)

        controls_row.addWidget(
            self.track_checkbox
        )
        controls_row.addWidget(
            self.stop_tracking_button
        )
        controls_row.addWidget(
            self.mode_label
        )
        controls_row.addSpacing(24)
        controls_row.addWidget(
            self.zoom_out_button
        )
        controls_row.addWidget(
            self.zoom_in_button
        )
        controls_row.addSpacing(24)
        controls_row.addWidget(QtWidgets.QLabel("IR Palette"))
        controls_row.addWidget(self.palette_combo)
        controls_row.addSpacing(24)
        controls_row.addWidget(self.record_button)
        controls_row.addSpacing(24)
        controls_row.addWidget(self.reset_gimbal_button)
        controls_row.addWidget(self.level_roll_button)
        controls_row.addSpacing(24)
        controls_row.addWidget(self.status_label, 1)

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
            f"rtsp://{args.remote_host}:"
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
        if self.bridge_connected:
            self._disconnect_bridge()
            return

        cfg = BridgeConfig(
            role=self.role_combo.currentText(),
            host=self.host_edit.text().strip(),
            port=self.port_spin.value(),
            token=self.token_edit.text().strip(),
            connect_timeout=(
                RemoteBridgeConfig.CONNECT_TIMEOUT
            ),
            ack_timeout=(
                RemoteBridgeConfig.ACK_TIMEOUT
            ),
            retry_count=(
                RemoteBridgeConfig.RETRY_COUNT
            ),
            reconnect_interval=(
                RemoteBridgeConfig.RECONNECT_INTERVAL
            ),
        )

        self.bridge = TcpCommandBridge(
            cfg,
            logger=self._bridge_log,
        )
        self.bridge.start()

        if cfg.role == "connect":
            connected = (
                self.bridge.wait_until_connected(
                    RemoteBridgeConfig.CONNECT_TIMEOUT
                )
            )

            if not connected:
                self.status_label.setText(
                    "Bridge connect timeout"
                )
                self._disconnect_bridge()
                return

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

        self.geo_wait_started = (
            time.monotonic()
        )
        self.geo_timeout_notified = False

        self._send_remote_command(
            CMD_PAYLOAD_TRACK,
            [
                1
                if self.track_checkbox.isChecked()
                else 0
            ],
        )

        self._send_remote_command(
            CMD_PAYLOAD_ZOOM_STOP,
            [],
        )

    def _disconnect_bridge(self) -> None:
        if (
            self.bridge_connected
            and self.bridge is not None
        ):
            try:
                self._send_remote_command(
                    CMD_PAYLOAD_ZOOM_STOP,
                    [],
                )
            except Exception:
                pass

        if self.bridge is not None:
            self.bridge.stop()
            self.bridge = None

        self.bridge_connected = False
        self.tracking_enabled = False
        self.zoom_active = False

        self.connect_button.setText(
            "Connect Bridge"
        )
        self.bridge_status.setText(
            "Disconnected"
        )
        self.bridge_status.setStyleSheet(
            "color: #e11d48; font-weight: 600;"
        )

        self.geo_wait_started = None
        self.compass.set_headings(None, None, None)
        self.geo_status_label.setText(
            "Estimator: waiting for bridge"
        )
        self.geo_status_label.setStyleSheet(
            "color: #94a3b8; font-weight: 600;"
        )

    def _on_record_clicked(self, checked: bool) -> None:
        ok, _ = self._send_remote_command(
            CMD_PAYLOAD_RECORD, [1 if checked else 0]
        )
        if not ok:
            self.record_button.setChecked(not checked)
        self._set_record_ui(self.record_button.isChecked())

    def _set_record_ui(self, recording: bool) -> None:
        self.record_button.setChecked(recording)
        self.record_button.setText("Stop Rec" if recording else "Record")

    def _send_remote_command(
        self,
        command: str,
        params: List,
    ) -> Tuple[bool, str]:
        if (
            not self.bridge_connected
            or self.bridge is None
        ):
            detail = (
                f"Bridge offline, skipped {command}"
            )
            self.status_label.setText(
                detail
            )
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
    # Geolocation polling
    # ------------------------------------------------------------------

    def _poll_geo_status(self) -> None:
        if (
            not self.bridge_connected
            or self.bridge is None
        ):
            return

        with self._geo_poll_lock:
            if self._geo_poll_busy:
                return
            self._geo_poll_busy = True

        bridge = self.bridge

        thread = threading.Thread(
            target=self._geo_poll_worker,
            args=(bridge,),
            daemon=True,
            name="geo-status-poll",
        )
        thread.start()

    def _geo_poll_worker(
        self,
        bridge: TcpCommandBridge,
    ) -> None:
        try:
            ok, detail = bridge.send_command(
                command=CMD_GET_GEO_STATUS,
                params=[],
                ack_required=True,
            )
            self._geo_signals.result.emit(
                ok,
                detail,
            )
        except Exception as exc:
            self._geo_signals.result.emit(
                False,
                str(exc),
            )
        finally:
            with self._geo_poll_lock:
                self._geo_poll_busy = False

    @QtCore.Slot(bool, str)
    def _on_geo_poll_result(
        self,
        ok: bool,
        detail: str,
    ) -> None:
        if not self.bridge_connected:
            return

        if not ok:
            self._handle_geo_not_ready(
                f"Status request failed: {detail}"
            )
            return

        try:
            data = json.loads(detail)
        except Exception as exc:
            self._handle_geo_not_ready(
                f"Invalid GEO_STATUS: {exc}"
            )
            return

        self._render_geo_status(data)

    def _handle_geo_not_ready(
        self,
        message: str,
    ) -> None:
        elapsed = 0.0

        if self.geo_wait_started is not None:
            elapsed = (
                time.monotonic()
                - self.geo_wait_started
            )

        if elapsed >= self.geo_timeout_s:
            self.geo_status_label.setText(
                "Estimator unavailable: "
                f"{message}. "
                "Pointing/tracking still work."
            )
            self.geo_status_label.setStyleSheet(
                "color: #f59e0b; font-weight: 700;"
            )
        else:
            self.geo_status_label.setText(
                f"Estimator: {message}"
            )
            self.geo_status_label.setStyleSheet(
                "color: #f59e0b; font-weight: 600;"
            )

    def _render_geo_status(
        self,
        data: Dict,
    ) -> None:
        # Mirror the camera's IR palette; activated() only fires on user
        # clicks, so this never echoes a command back.
        palette = data.get("ir_palette")
        if (
            palette is not None
            and palette != self.palette_combo.currentData()
        ):
            idx = self.palette_combo.findData(palette)
            if idx >= 0:
                self.palette_combo.setCurrentIndex(idx)

        # Mirror camera-reported recording state (clicked() only fires on
        # user clicks, so setChecked here never echoes a command).
        recording = data.get("recording")
        if recording is not None and bool(recording) != self.record_button.isChecked():
            self._set_record_ui(bool(recording))

        # Mirror executor hold state (Reset Gimbal clears it there).
        hold = data.get("heading_hold_on")
        if hold is not None and bool(hold) != self.hold_heading_checkbox.isChecked():
            self.hold_heading_checkbox.blockSignals(True)
            self.hold_heading_checkbox.setChecked(bool(hold))
            self.hold_heading_checkbox.blockSignals(False)

        ap_connected = bool(
            data.get(
                "ardupilot_connected",
                False,
            )
        )
        gps_valid = bool(
            data.get(
                "gps_valid",
                False,
            )
        )
        estimate_valid = bool(
            data.get(
                "estimate_valid",
                False,
            )
        )

        self.ardupilot_status_label.setText(
            "ArduPilot: Connected"
            if ap_connected
            else "ArduPilot: Not connected"
        )
        self.gps_status_label.setText(
            "GPS: Valid"
            if gps_valid
            else "GPS: Invalid / waiting"
        )

        vehicle_lat = self._num(
            data.get("vehicle_lat")
        )
        vehicle_lon = self._num(
            data.get("vehicle_lon")
        )
        vehicle_alt = self._num(
            data.get("vehicle_alt_m")
        )

        if (
            vehicle_lat is not None
            and vehicle_lon is not None
        ):
            vehicle_text = (
                f"Vehicle: "
                f"{vehicle_lat:.7f}, "
                f"{vehicle_lon:.7f} | "
                f"{to_mgrs(vehicle_lat, vehicle_lon)}"
            )

            if vehicle_alt is not None:
                vehicle_text += (
                    f", alt={vehicle_alt:.1f} m"
                )

            self.vehicle_position_label.setText(
                vehicle_text
            )
        else:
            self.vehicle_position_label.setText(
                "Vehicle: --"
            )

        vr = self._num(
            data.get("vehicle_roll_deg")
        )
        vp = self._num(
            data.get("vehicle_pitch_deg")
        )
        vy = self._num(
            data.get("vehicle_yaw_deg")
        )

        if (
            vr is not None
            and vp is not None
            and vy is not None
        ):
            self.vehicle_attitude_label.setText(
                f"Vehicle attitude: "
                f"R {vr:.1f}°  "
                f"P {vp:.1f}°  "
                f"Y {vy:.1f}°"
            )
        else:
            self.vehicle_attitude_label.setText(
                "Vehicle attitude: --"
            )

        gr = self._num(
            data.get("gimbal_roll_deg")
        )
        gp = self._num(
            data.get("gimbal_pitch_deg")
        )
        gy = self._num(
            data.get("gimbal_yaw_deg")
        )
        gframe = str(
            data.get(
                "gimbal_frame",
                "unknown",
            )
        )

        if (
            gr is not None
            and gp is not None
            and gy is not None
        ):
            self.gimbal_attitude_label.setText(
                f"Gimbal attitude: "
                f"R {gr:.1f}°  "
                f"P {gp:.1f}°  "
                f"Y {gy:.1f}° "
                f"({gframe})"
            )
        else:
            self.gimbal_attitude_label.setText(
                "Gimbal attitude: --"
            )

        # Earth-frame camera yaw: executor value when the estimator ran,
        # else mirror its frame logic (vehicle-frame yaw adds vehicle yaw).
        cam_yaw = self._num(data.get("camera_yaw_ned_deg"))
        if cam_yaw is None and gy is not None:
            if gframe == "earth":
                cam_yaw = gy
            elif vy is not None:
                cam_yaw = vy + gy
        self.compass.set_headings(
            vy, cam_yaw, self._num(data.get("hfov_deg"))
        )

        tx = self._num(
            data.get("track_x")
        )
        ty = self._num(
            data.get("track_y")
        )
        track_status = data.get(
            "track_status"
        )
        track_source = str(
            data.get(
                "track_pixel_source",
                "none",
            )
        )

        hfov = self._num(
            data.get("hfov_deg")
        )
        vfov = self._num(
            data.get("vfov_deg")
        )
        fov_source = str(
            data.get(
                "fov_source",
                "unknown",
            )
        )

        track_text = "Track: "

        if tx is not None and ty is not None:
            track_text += (
                f"pixel=({tx:.0f}, {ty:.0f}) "
                f"[{track_source}]"
            )
        else:
            track_text += "--"

        if track_status is not None:
            status_name = {
                0: "IDLE",
                1: "TRACKED",
                2: "LOST",
            }.get(
                int(track_status),
                str(track_status),
            )
            track_text += (
                f" status={status_name}"
            )

        if (
            hfov is not None
            and vfov is not None
        ):
            track_text += (
                f" FOV={hfov:.1f}°x"
                f"{vfov:.1f}° "
                f"[{fov_source}]"
            )

        self.track_info_label.setText(
            track_text
        )

        down_angle = self._num(
            data.get("los_down_deg")
        )
        azimuth = self._num(
            data.get("los_azimuth_deg")
        )
        ground_range = self._num(
            data.get("ground_range_m")
        )
        height_agl = self._num(
            data.get("height_agl_m")
        )
        height_source = str(
            data.get(
                "height_source",
                "none",
            )
        )

        pieces = []

        if height_agl is not None:
            pieces.append(
                f"AGL={height_agl:.1f}m"
                f"[{height_source}]"
            )
        if azimuth is not None:
            pieces.append(
                f"LOS az={azimuth:.1f}°"
            )
        if down_angle is not None:
            pieces.append(
                f"down={down_angle:.1f}°"
            )
        if ground_range is not None:
            pieces.append(
                f"range≈{ground_range:.1f}m"
            )

        self.range_label.setText(
            "Geometry: "
            + (
                "  ".join(pieces)
                if pieces
                else "--"
            )
        )

        est_lat = self._num(
            data.get(
                "estimated_target_lat"
            )
        )
        est_lon = self._num(
            data.get(
                "estimated_target_lon"
            )
        )
        est_alt = self._num(
            data.get(
                "estimated_target_alt_m"
            )
        )

        reason = str(
            data.get(
                "estimate_reason",
                "",
            )
        )

        if (
            estimate_valid
            and est_lat is not None
            and est_lon is not None
        ):
            target_text = (
                "Estimated target: "
                f"{est_lat:.7f}, "
                f"{est_lon:.7f} | "
                f"{to_mgrs(est_lat, est_lon)}"
            )

            if est_alt is not None:
                target_text += (
                    f", alt≈{est_alt:.1f} m"
                )

            self.target_position_label.setText(
                target_text
            )
            self.target_position_label.setStyleSheet(
                "color: #16a34a; font-weight: 700;"
            )

            self.geo_status_label.setText(
                "Estimator: target solution valid "
                "(flat-ground approximation)"
            )
            self.geo_status_label.setStyleSheet(
                "color: #16a34a; font-weight: 700;"
            )

            print(
                "[TARGET EST] "
                f"lat={est_lat:.7f} "
                f"lon={est_lon:.7f} "
                + (
                    f"range={ground_range:.1f}m"
                    if ground_range is not None
                    else ""
                )
            )
        else:
            self.target_position_label.setText(
                "Estimated target: --"
            )
            self.target_position_label.setStyleSheet(
                "font-weight: 700;"
            )

            self._handle_geo_not_ready(
                reason
                or "No target solution yet"
            )

        native_lat = self._num(
            data.get("gremsy_target_lat")
        )
        native_lon = self._num(
            data.get("gremsy_target_lon")
        )
        native_alt = self._num(
            data.get("gremsy_target_alt")
        )

        if (
            native_lat is not None
            and native_lon is not None
            and not (
                abs(native_lat) < 1e-12
                and abs(native_lon) < 1e-12
            )
        ):
            text = (
                "Gremsy TARGET_*: "
                f"{native_lat:.7f}, "
                f"{native_lon:.7f} | "
                f"{to_mgrs(native_lat, native_lon)}"
            )

            if native_alt is not None:
                text += (
                    f", alt={native_alt:.1f} m"
                )

            self.native_target_label.setText(
                text
            )
        else:
            self.native_target_label.setText(
                "Gremsy TARGET_*: not provided"
            )

    @staticmethod
    def _num(
        value,
    ) -> Optional[float]:
        if value is None:
            return None

        try:
            number = float(value)
        except (
            TypeError,
            ValueError,
        ):
            return None

        if not math.isfinite(number):
            return None

        return number

    # ------------------------------------------------------------------
    # RTSP
    # ------------------------------------------------------------------

    def _start_stream(self) -> None:
        url = (
            self.rtsp_url_edit.text()
            .strip()
        )

        if not url:
            self.status_label.setText(
                "RTSP URL is empty"
            )
            return

        # ponytail: sensor is inferred from the relay path name.
        self.ir_stream = (
            url.rstrip("/").lower().endswith("/ir")
        )

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
    # Track
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
                "Track enabled - click target"
            )
        else:
            self.mode_label.setText(
                "Click mode: Point Camera"
            )
            self.status_label.setText(
                "Track disabled - click to point"
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
    # Zoom
    # ------------------------------------------------------------------

    def _zoom_in_pressed(self) -> None:
        if not self.bridge_connected:
            self.status_label.setText(
                "Bridge offline - cannot zoom"
            )
            return

        ok, detail = self._send_remote_command(
            CMD_PAYLOAD_ZOOM_IN,
            ["ir"] if self.ir_stream else [],
        )

        if ok:
            self.zoom_active = True
            self.status_label.setText(
                detail if self.ir_stream else "Zooming in..."
            )

    def _zoom_out_pressed(self) -> None:
        if not self.bridge_connected:
            self.status_label.setText(
                "Bridge offline - cannot zoom"
            )
            return

        ok, detail = self._send_remote_command(
            CMD_PAYLOAD_ZOOM_OUT,
            ["ir"] if self.ir_stream else [],
        )

        if ok:
            self.zoom_active = True
            self.status_label.setText(
                detail if self.ir_stream else "Zooming out..."
            )

    def _zoom_released(self) -> None:
        if not self.bridge_connected or self.ir_stream:
            # IR zoom is stepped per press; nothing to stop.
            self.zoom_active = False
            return

        ok, _ = self._send_remote_command(
            CMD_PAYLOAD_ZOOM_STOP,
            [],
        )

        self.zoom_active = False

        if ok:
            self.status_label.setText(
                "Zoom stopped"
            )

    # ------------------------------------------------------------------
    # Click mapping
    # ------------------------------------------------------------------

    def _widget_point_to_payload(
        self,
        x_widget: float,
        y_widget: float,
        frame_w: int,
        frame_h: int,
    ) -> Optional[
        Tuple[int, int]
    ]:
        if (
            frame_w <= 0
            or frame_h <= 0
        ):
            return None

        widget_w = (
            self.video_widget.width()
        )
        widget_h = (
            self.video_widget.height()
        )

        if (
            widget_w <= 0
            or widget_h <= 0
        ):
            return None

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

        if (
            x_widget < offset_x
            or x_widget
            >= offset_x + displayed_w
            or y_widget < offset_y
            or y_widget
            >= offset_y + displayed_h
        ):
            return None

        x_src = (
            x_widget - offset_x
        ) / scale
        y_src = (
            y_widget - offset_y
        ) / scale

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

    def _on_video_clicked(
        self,
        x_widget: float,
        y_widget: float,
        frame_w: int,
        frame_h: int,
    ) -> None:
        point = (
            self._widget_point_to_payload(
                x_widget,
                y_widget,
                frame_w,
                frame_h,
            )
        )

        if point is None:
            self.status_label.setText(
                "Click inside actual video image"
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
            f"Gremsy=({x_payload}, "
            f"{y_payload})"
        )

        ok, detail = self._send_remote_command(
            CMD_PAYLOAD_TOUCH,
            [x_payload, y_payload]
            + (["ir"] if self.ir_stream else []),
        )

        if not ok:
            return

        if self.ir_stream:
            # Executor reports the remapped EO pixel and any FOV hint.
            self.status_label.setText(detail)
        elif self.tracking_enabled:
            self.status_label.setText(
                f"Tracking target at "
                f"({x_payload}, {y_payload})"
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
                    CMD_PAYLOAD_ZOOM_STOP,
                    [],
                )
            except Exception:
                pass

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
            "Gremsy Lynx PyQt UI with "
            "target geolocation estimator"
        )
    )

    parser.add_argument(
        "--remote-mode",
        choices=[
            "connect",
            "listen",
        ],
        default="connect",
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
            "Seconds before unavailable "
            "estimator state becomes a warning."
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
