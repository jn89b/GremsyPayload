#!/usr/bin/env python3
"""PyQt remote gimbal UI MVP with embedded RTSP and click-to-track."""

import argparse
import os
import sys
from typing import List

from PySide6 import QtCore, QtGui, QtWidgets

# Reuse shared bridge + config from existing project modules.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "libs"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ui_demo"))

from config import ConnectionConfig, RemoteBridgeConfig
from remote_bridge import BridgeConfig, TcpCommandBridge
from widgets.video_widget import RtspVideoWidget

CMD_PAYLOAD_TOUCH = "PAYLOAD_TOUCH"
CMD_PAYLOAD_TRACK = "PAYLOAD_TRACK"


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, args: argparse.Namespace):
        super().__init__()
        self.setWindowTitle("Payload UI Demo - PyQt MVP")
        self.resize(1400, 860)

        self.bridge = None
        self.bridge_connected = False
        self.tracking_enabled = False

        self._build_ui()
        self._apply_defaults(args)

    def _build_ui(self) -> None:
        root = QtWidgets.QWidget(self)
        self.setCentralWidget(root)

        layout = QtWidgets.QVBoxLayout(root)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        conn_group = QtWidgets.QGroupBox("Remote Bridge")
        conn_grid = QtWidgets.QGridLayout(conn_group)

        self.role_combo = QtWidgets.QComboBox()
        self.role_combo.addItems(["connect", "listen"])

        self.host_edit = QtWidgets.QLineEdit()
        self.port_spin = QtWidgets.QSpinBox()
        self.port_spin.setRange(1, 65535)
        self.token_edit = QtWidgets.QLineEdit()
        self.token_edit.setEchoMode(QtWidgets.QLineEdit.EchoMode.Password)

        self.connect_button = QtWidgets.QPushButton("Connect Bridge")
        self.connect_button.clicked.connect(self._toggle_bridge)

        self.bridge_status = QtWidgets.QLabel("Disconnected")
        self.bridge_status.setStyleSheet("color: #e11d48; font-weight: 600;")

        conn_grid.addWidget(QtWidgets.QLabel("Role"), 0, 0)
        conn_grid.addWidget(self.role_combo, 0, 1)
        conn_grid.addWidget(QtWidgets.QLabel("Host"), 0, 2)
        conn_grid.addWidget(self.host_edit, 0, 3)
        conn_grid.addWidget(QtWidgets.QLabel("Port"), 0, 4)
        conn_grid.addWidget(self.port_spin, 0, 5)
        conn_grid.addWidget(QtWidgets.QLabel("Token"), 0, 6)
        conn_grid.addWidget(self.token_edit, 0, 7)
        conn_grid.addWidget(self.connect_button, 0, 8)
        conn_grid.addWidget(self.bridge_status, 0, 9)

        stream_group = QtWidgets.QGroupBox("RTSP Stream")
        stream_layout = QtWidgets.QHBoxLayout(stream_group)

        self.rtsp_url_edit = QtWidgets.QLineEdit()
        self.rtsp_url_edit.setPlaceholderText("rtsp://ip:8554/eo")

        self.play_button = QtWidgets.QPushButton("Play")
        self.play_button.clicked.connect(self._start_stream)
        self.stop_button = QtWidgets.QPushButton("Stop")
        self.stop_button.clicked.connect(self._stop_stream)

        stream_layout.addWidget(self.rtsp_url_edit, stretch=1)
        stream_layout.addWidget(self.play_button)
        stream_layout.addWidget(self.stop_button)

        self.video_widget = RtspVideoWidget()
        self.video_widget.clicked.connect(self._on_video_clicked)

        controls_row = QtWidgets.QHBoxLayout()
        self.touch_checkbox = QtWidgets.QCheckBox("Touch")
        self.touch_checkbox.setChecked(True)

        self.track_checkbox = QtWidgets.QCheckBox("Track")
        self.track_checkbox.stateChanged.connect(self._on_track_toggled)

        self.status_label = QtWidgets.QLabel("Ready")
        self.status_label.setStyleSheet("color: #38bdf8;")

        controls_row.addWidget(self.touch_checkbox)
        controls_row.addWidget(self.track_checkbox)
        controls_row.addStretch(1)
        controls_row.addWidget(self.status_label)

        layout.addWidget(conn_group)
        layout.addWidget(stream_group)
        layout.addWidget(self.video_widget, stretch=1)
        layout.addLayout(controls_row)

    def _apply_defaults(self, args: argparse.Namespace) -> None:
        self.role_combo.setCurrentText(args.remote_mode)
        self.host_edit.setText(args.remote_host)
        self.port_spin.setValue(args.remote_port)
        self.token_edit.setText(args.remote_token)

        default_rtsp_url = (
            f"rtsp://{ConnectionConfig.UDP_IP_TARGET}:"
            f"{ConnectionConfig.RTSP_PORT_TARGET}/{ConnectionConfig.RTSP_PATH_TARGET}"
        )
        self.rtsp_url_edit.setText(default_rtsp_url)

    def _toggle_bridge(self) -> None:
        if self.bridge_connected:
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

        self.bridge = TcpCommandBridge(cfg, logger=self._bridge_log)
        self.bridge.start()

        if cfg.role == "connect":
            connected = self.bridge.wait_until_connected(RemoteBridgeConfig.CONNECT_TIMEOUT)
            if not connected:
                self.status_label.setText("Bridge connect timeout")
                self._disconnect_bridge()
                return

        self.bridge_connected = True
        self.connect_button.setText("Disconnect Bridge")
        self.bridge_status.setText("Connected")
        self.bridge_status.setStyleSheet("color: #16a34a; font-weight: 600;")
        self.status_label.setText("Bridge ready")

    def _disconnect_bridge(self) -> None:
        if self.bridge is not None:
            self.bridge.stop()
            self.bridge = None

        self.bridge_connected = False
        self.connect_button.setText("Connect Bridge")
        self.bridge_status.setText("Disconnected")
        self.bridge_status.setStyleSheet("color: #e11d48; font-weight: 600;")

    def _start_stream(self) -> None:
        url = self.rtsp_url_edit.text().strip()
        if not url:
            self.status_label.setText("RTSP URL is empty")
            return

        ok = self.video_widget.start_stream(url)
        self.status_label.setText("Stream playing" if ok else "Failed to open stream")

    def _stop_stream(self) -> None:
        self.video_widget.stop_stream()
        self.status_label.setText("Stream stopped")

    def _on_track_toggled(self, state: int) -> None:
        enabled = 1 if state == QtCore.Qt.CheckState.Checked else 0
        self.tracking_enabled = bool(enabled)
        self._send_remote_command(CMD_PAYLOAD_TRACK, [float(enabled)])

    def _on_video_clicked(self, x_widget: float, y_widget: float, frame_w: int, frame_h: int) -> None:
        if not self.touch_checkbox.isChecked():
            return

        if frame_w <= 0 or frame_h <= 0:
            self.status_label.setText("No frame available for click mapping")
            return

        view_w = max(1, self.video_widget.width())
        view_h = max(1, self.video_widget.height())

        # Map widget click to source frame coordinates with clamping.
        x_src = max(0.0, min(float(frame_w - 1), (x_widget / view_w) * frame_w))
        y_src = max(0.0, min(float(frame_h - 1), (y_widget / view_h) * frame_h))

        # Payload tracking API expects a 1920x1080 coordinate space.
        x_payload = int((x_src / max(1.0, frame_w)) * 1920)
        y_payload = int((y_src / max(1.0, frame_h)) * 1080)

        self._send_remote_command(CMD_PAYLOAD_TOUCH, [x_payload, y_payload])

    def _send_remote_command(self, command: str, params: List) -> None:
        if not self.bridge_connected or self.bridge is None:
            self.status_label.setText(f"Bridge offline, skipped {command}")
            return

        ok, detail = self.bridge.send_command(command=command, params=params, ack_required=True)
        if ok:
            self.status_label.setText(f"ACK {command}: {detail}")
        else:
            self.status_label.setText(f"NACK {command}: {detail}")

    def _bridge_log(self, msg: str) -> None:
        print(msg)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self.video_widget.stop_stream()
        self._disconnect_bridge()
        super().closeEvent(event)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Payload PyQt UI Demo MVP")
    parser.add_argument(
        "--remote-mode",
        choices=["connect", "listen"],
        default="connect",
        help="Remote bridge role for this UI process",
    )
    parser.add_argument("--remote-host", default=RemoteBridgeConfig.HOST)
    parser.add_argument("--remote-port", type=int, default=RemoteBridgeConfig.PORT)
    parser.add_argument("--remote-token", default=RemoteBridgeConfig.TOKEN)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow(args)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
