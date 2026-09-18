#!/usr/bin/env python3
"""PyQt video widget backed by OpenCV for RTSP display and click callbacks.

Optional local boat detection. Point BOAT_MODEL at YOLO weights and every stream the
app plays is run through the model on this machine (the Mac). Nothing extra is
streamed from the Pi; the app just decodes the same /eo or /ir stream it always did.

    BOAT_MODEL=~/models/eo_best.pt BOAT_MODEL_IR=~/models/ir_best.pt python app.py

Leave BOAT_MODEL unset and the widget behaves exactly like the original.

Env vars:
  BOAT_MODEL      weights for EO streams (.pt, or a CoreML .mlpackage)
  BOAT_MODEL_IR   weights for streams whose path ends in /ir (falls back to BOAT_MODEL)
  BOAT_CONF       confidence threshold (default 0.25)
  BOAT_IMGSZ      inference size (default: the size the model was trained at)
  BOAT_DEVICE     mps / cpu / cuda (default: mps when available)
  BOAT_SYNC       1 (default): show the exact frame the model saw, so boxes line up,
                  and video runs at model speed.
                  0: smooth video with the newest boxes drawn on it; boxes can trail
                  the boat while the gimbal is slewing.
Streams whose path contains "detections" are never run through the model, since the
Pi has already drawn boxes on them.
"""

import os
import threading
import time
from typing import Callable, Optional, Tuple

import cv2
from PySide6 import QtCore, QtGui, QtWidgets

BOX_COLOR = (0, 255, 255)  # BGR yellow, readable on both EO and white-hot IR


def _default_device() -> str:
    try:
        import torch

        if torch.backends.mps.is_available():
            return "mps"
        if torch.cuda.is_available():
            return "cuda"
    except Exception:
        pass
    return "cpu"


class _Detector:
    """Thin wrapper around an Ultralytics YOLO model. Only ever called from one thread."""

    def __init__(self, weights: str):
        from ultralytics import YOLO  # lazy import: the app still runs without ultralytics

        self.name = os.path.basename(weights.rstrip("/"))
        self.model = YOLO(weights, task="detect")
        self.kwargs = {"conf": float(os.environ.get("BOAT_CONF", "0.25")), "verbose": False}
        if os.environ.get("BOAT_IMGSZ"):
            self.kwargs["imgsz"] = int(os.environ["BOAT_IMGSZ"])
        if weights.endswith(".pt"):
            self.kwargs["device"] = os.environ.get("BOAT_DEVICE") or _default_device()
        self.device = self.kwargs.get("device", "coreml")

    def __call__(self, frame):
        r = self.model.predict(frame, **self.kwargs)[0]
        return r.boxes.xyxy.cpu().numpy().astype(int), r.boxes.conf.cpu().numpy()


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

        # detection state
        self._sync = os.environ.get("BOAT_SYNC", "1") != "0"
        self._detectors = {}  # weights path -> _Detector, so switching streams doesn't reload
        self._det_thread = None
        self._det_generation = 0
        self._det_active = False
        self._det_result = None  # (frame, boxes, confs)
        self._det_fps = 0.0
        self._drawn = (None, None)

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

        weights = self._weights_for(rtsp_url)
        if weights:
            self._det_generation += 1
            self._det_active = True
            self._det_thread = threading.Thread(
                target=self._detect_loop, args=(weights, self._det_generation), daemon=True
            )
            self._det_thread.start()

        self._poll_timer.start(33)
        return True

    def stop_stream(self) -> None:
        """Stop stream and release resources."""
        self._running = False
        self._det_active = False
        self._det_generation += 1
        self._poll_timer.stop()

        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        self._reader_thread = None

        if self._det_thread and self._det_thread.is_alive():
            self._det_thread.join(timeout=2.0)
        self._det_thread = None

        if self._capture is not None:
            self._capture.release()
        self._capture = None

        with self._frame_lock:
            self._last_frame = None
            self._det_result = None
        self._drawn = (None, None)

        self.setPixmap(QtGui.QPixmap())
        self.setText("No video stream")

    @staticmethod
    def _weights_for(url: str) -> Optional[str]:
        path = url.rstrip("/").lower()
        if "detections" in path.rsplit("/", 1)[-1]:
            return None
        eo = os.environ.get("BOAT_MODEL")
        ir = os.environ.get("BOAT_MODEL_IR")
        weights = (ir or eo) if path.endswith("/ir") else eo  # same /ir rule app.py uses
        return os.path.expanduser(weights) if weights else None

    def _reader_loop(self) -> None:
        while self._running and self._capture is not None:
            ok, frame = self._capture.read()
            if not ok:
                time.sleep(0.05)
                continue

            with self._frame_lock:
                self._last_frame = frame

    def _detect_loop(self, weights: str, generation: int) -> None:
        try:
            det = self._detectors.get(weights)
            if det is None:
                print(f"[detect] loading {weights}")
                det = _Detector(weights)
                self._detectors[weights] = det
        except Exception as exc:
            print(f"[detect] could not load {weights}: {exc}")
            self._det_active = False
            return
        print(f"[detect] running {det.name} on {det.device}")

        last_frame, count, t0 = None, 0, time.time()
        while self._running and generation == self._det_generation:
            with self._frame_lock:
                frame = self._last_frame
            if frame is None or frame is last_frame:  # only run on new frames
                time.sleep(0.002)
                continue
            last_frame = frame

            try:
                boxes, confs = det(frame)
            except Exception as exc:
                print(f"[detect] inference failed: {exc}")
                time.sleep(0.5)
                continue

            with self._frame_lock:
                if generation == self._det_generation:
                    self._det_result = (frame, boxes, confs)

            count += 1
            elapsed = time.time() - t0
            if elapsed >= 2.0:
                self._det_fps, count, t0 = count / elapsed, 0, time.time()

    def _draw_latest_frame(self) -> None:
        with self._frame_lock:
            raw, det = self._last_frame, self._det_result

        if self._det_active and det is not None:
            src, boxes, confs = det
            if not self._sync:
                src = raw if raw is not None else src
            if src is self._drawn[0] and det is self._drawn[1]:
                return  # nothing new to show
            self._drawn = (src, det)
            frame = src.copy()
            for (x1, y1, x2, y2), c in zip(boxes, confs):
                cv2.rectangle(frame, (x1, y1), (x2, y2), BOX_COLOR, 2)
                cv2.putText(frame, f"boat {c:.2f}", (x1, max(y1 - 6, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, BOX_COLOR, 1, cv2.LINE_AA)
            cv2.putText(frame, f"det {self._det_fps:.1f} fps  {len(boxes)} boats", (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, BOX_COLOR, 2, cv2.LINE_AA)
        else:
            if raw is None or raw is self._drawn[0]:
                return
            self._drawn = (raw, None)
            frame = raw.copy()

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

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:
        self._drawn = (None, None)  # force a redraw at the new size
        super().resizeEvent(event)

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            frame_w, frame_h = self.current_frame_size()
            self.clicked.emit(event.position().x(), event.position().y(), frame_w, frame_h)
        super().mousePressEvent(event)
