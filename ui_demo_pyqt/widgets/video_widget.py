#!/usr/bin/env python3
"""PyQt video widget backed by OpenCV for RTSP display and click callbacks."""

import os
import threading
import time
from typing import Callable, Optional, Tuple

import cv2
from PySide6 import QtCore, QtGui, QtWidgets


class RtspVideoWidget(QtWidgets.QLabel):
    """Simple video surface that renders RTSP frames and reports click coordinates."""

    clicked = QtCore.Signal(float, float, int, int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.setText("No video stream")
        self.setMinimumSize(640, 360)
        self.setStyleSheet("background: #101215; color: #98a2b3; border: 1px solid #2a2f3a;")

        self._capture = None
        self._reader_thread = None
        self._running = False
        self._last_frame = None
        self._frame_lock = threading.Lock()
        self._poll_timer = QtCore.QTimer(self)
        self._poll_timer.timeout.connect(self._draw_latest_frame)


    def start_stream(self, rtsp_url: str) -> bool:
        """Start frame reader thread for RTSP URL."""
        self.stop_stream()

        # ponytail: FFmpeg low-latency flags; must be set before VideoCapture is created
        os.environ.setdefault(
            "OPENCV_FFMPEG_CAPTURE_OPTIONS",
            "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|max_delay;0",
        )

        cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
        if not cap.isOpened():
            self.setText("Failed to open RTSP stream")
            return False

        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self._capture = cap
        self._running = True
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()
        self._poll_timer.start(33)
        return True


    def stop_stream(self) -> None:
        """Stop stream and release resources."""
        self._running = False
        self._poll_timer.stop()

        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        self._reader_thread = None

        if self._capture is not None:
            self._capture.release()
        self._capture = None

        with self._frame_lock:
            self._last_frame = None

        self.setPixmap(QtGui.QPixmap())
        self.setText("No video stream")

    def _reader_loop(self) -> None:
        while self._running and self._capture is not None:
            ok, frame = self._capture.read()
            if not ok:
                time.sleep(0.05)
                continue

            with self._frame_lock:
                self._last_frame = frame

    def _draw_latest_frame(self) -> None:
        frame = None
        with self._frame_lock:
            if self._last_frame is not None:
                frame = self._last_frame.copy()

        if frame is None:
            return

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width, channels = frame_rgb.shape
        bytes_per_line = channels * width

        image = QtGui.QImage(
            frame_rgb.data,
            width,
            height,
            bytes_per_line,
            QtGui.QImage.Format.Format_RGB888,
        )

        pixmap = QtGui.QPixmap.fromImage(image)
        self.setPixmap(
            pixmap.scaled(
                self.width(),
                self.height(),
                QtCore.Qt.AspectRatioMode.KeepAspectRatio,
                QtCore.Qt.TransformationMode.SmoothTransformation,
            )
        )

    def current_frame_size(self) -> Tuple[int, int]:
        with self._frame_lock:
            if self._last_frame is None:
                return (0, 0)
            h, w = self._last_frame.shape[:2]
            return (w, h)

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            frame_w, frame_h = self.current_frame_size()
            self.clicked.emit(event.position().x(), event.position().y(), frame_w, frame_h)
        super().mousePressEvent(event)
