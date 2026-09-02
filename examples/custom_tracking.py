#!/usr/bin/env python3

import argparse
import json
import socket
import subprocess
import threading
import time
from dataclasses import dataclass
from fractions import Fraction
from typing import Optional

import cv2
import numpy as np


# ============================================================
# Track data
# ============================================================

@dataclass
class Track:
    track_id: int

    x1: float
    y1: float
    x2: float
    y2: float

    label: str = "target"
    confidence: float = 1.0

    # If True, coordinates are 0.0 -> 1.0
    normalized: bool = False

    # Optional coordinate-space resolution.
    # Useful if your model works on something like 640x640
    # while the video is 1280x720.
    source_width: Optional[int] = None
    source_height: Optional[int] = None

    received_time: float = 0.0


class TrackState:
    def __init__(self):
        self._lock = threading.Lock()
        self._track: Optional[Track] = None

    def update(self, track: Track):
        with self._lock:
            self._track = track

    def clear(self):
        with self._lock:
            self._track = None

    def get(self) -> Optional[Track]:
        with self._lock:
            return self._track


# ============================================================
# UDP track receiver
# ============================================================

class TrackReceiver:
    """
    Receives track information over UDP.

    Pixel coordinates:

    {
        "track_id": 7,
        "x1": 420,
        "y1": 205,
        "x2": 615,
        "y2": 390,
        "label": "vehicle",
        "confidence": 0.93
    }

    Normalized coordinates:

    {
        "track_id": 7,
        "x1": 0.32,
        "y1": 0.28,
        "x2": 0.48,
        "y2": 0.54,
        "normalized": true
    }

    Clear:

    {
        "clear": true
    }
    """

    def __init__(
        self,
        state: TrackState,
        host: str = "0.0.0.0",
        port: int = 5005,
    ):
        self.state = state
        self.host = host
        self.port = port

        self.running = False
        self.thread = None
        self.sock = None

    def start(self):
        self.running = True

        self.thread = threading.Thread(
            target=self._run,
            daemon=True,
        )

        self.thread.start()

    def stop(self):
        self.running = False

        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass

    def _run(self):
        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )

        self.sock.bind(
            (
                self.host,
                self.port,
            )
        )

        self.sock.settimeout(0.5)

        print(
            f"[TRACK] Listening on "
            f"udp://{self.host}:{self.port}"
        )

        while self.running:

            try:
                data, addr = self.sock.recvfrom(65535)

            except socket.timeout:
                continue

            except OSError:
                break

            except Exception as exc:
                print(f"[TRACK] UDP error: {exc}")
                continue

            try:
                msg = json.loads(
                    data.decode("utf-8")
                )

                if msg.get("clear", False):
                    self.state.clear()

                    print("[TRACK] Track cleared")
                    continue

                track = Track(
                    track_id=int(
                        msg.get("track_id", 0)
                    ),

                    x1=float(msg["x1"]),
                    y1=float(msg["y1"]),
                    x2=float(msg["x2"]),
                    y2=float(msg["y2"]),

                    label=str(
                        msg.get(
                            "label",
                            "target",
                        )
                    ),

                    confidence=float(
                        msg.get(
                            "confidence",
                            1.0,
                        )
                    ),

                    normalized=bool(
                        msg.get(
                            "normalized",
                            False,
                        )
                    ),

                    source_width=msg.get(
                        "source_width"
                    ),

                    source_height=msg.get(
                        "source_height"
                    ),

                    received_time=time.monotonic(),
                )

                self.state.update(track)

            except Exception as exc:
                print(
                    f"[TRACK] Invalid packet "
                    f"from {addr}: {exc}"
                )


# ============================================================
# RTSP information
# ============================================================

def probe_stream(url: str):
    """
    Uses ffprobe to determine codec, width, height and FPS.
    """

    cmd = [
        "ffprobe",

        "-v",
        "error",

        "-rtsp_transport",
        "tcp",

        "-select_streams",
        "v:0",

        "-show_entries",
        "stream=codec_name,width,height,avg_frame_rate",

        "-of",
        "json",

        url,
    ]

    print(f"[PROBE] Checking {url}")

    output = subprocess.check_output(
        cmd,
        text=True,
    )

    info = json.loads(output)

    streams = info.get(
        "streams",
        []
    )

    if not streams:
        raise RuntimeError(
            "No video stream found."
        )

    stream = streams[0]

    codec = stream.get(
        "codec_name",
        "unknown",
    )

    width = int(
        stream["width"]
    )

    height = int(
        stream["height"]
    )

    rate = stream.get(
        "avg_frame_rate",
        "30/1",
    )

    try:
        fps = float(
            Fraction(rate)
        )
    except Exception:
        fps = 30.0

    if fps <= 0 or fps > 120:
        fps = 30.0

    print(
        f"[PROBE] Codec: {codec}"
    )

    print(
        f"[PROBE] Resolution: "
        f"{width}x{height}"
    )

    print(
        f"[PROBE] FPS: {fps:.2f}"
    )

    return codec, width, height, fps


# ============================================================
# FFmpeg H.265 -> BGR decoder
# ============================================================

class RtspDecoder:
    """
    Reads the MediaMTX RTSP stream with FFmpeg.

    H.265 / HEVC decoding occurs inside FFmpeg.

    FFmpeg outputs raw BGR frames to stdout.
    """

    def __init__(
        self,
        url: str,
        width: int,
        height: int,
    ):
        self.url = url
        self.width = width
        self.height = height

        self.frame_size = (
            width
            * height
            * 3
        )

        self.process = None

        self.running = False

        self.thread = None

        self.condition = (
            threading.Condition()
        )

        self.latest_frame = None
        self.frame_number = 0

    def start(self):

        command = [
            "ffmpeg",

            "-hide_banner",
            "-loglevel",
            "warning",

            # Low latency RTSP
            "-rtsp_transport",
            "tcp",

            "-fflags",
            "nobuffer",

            "-flags",
            "low_delay",

            # Input
            "-i",
            self.url,

            "-map",
            "0:v:0",

            "-an",
            "-sn",
            "-dn",

            # Decode H265 into OpenCV-compatible BGR
            "-pix_fmt",
            "bgr24",

            "-f",
            "rawvideo",

            "pipe:1",
        ]

        print(
            "[DECODER] Starting:"
        )

        print(
            " ".join(command)
        )

        self.process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=None,
            bufsize=0,
        )

        self.running = True

        self.thread = threading.Thread(
            target=self._reader,
            daemon=True,
        )

        self.thread.start()

    def _read_exact_frame(self):

        if not self.process:
            return None

        if not self.process.stdout:
            return None

        buffer = bytearray(
            self.frame_size
        )

        view = memoryview(buffer)

        total = 0

        while total < self.frame_size:

            count = (
                self.process.stdout.readinto(
                    view[total:]
                )
            )

            if not count:
                return None

            total += count

        frame = np.frombuffer(
            buffer,
            dtype=np.uint8,
        ).reshape(
            (
                self.height,
                self.width,
                3,
            )
        )

        return frame

    def _reader(self):

        while self.running:

            frame = (
                self._read_exact_frame()
            )

            if frame is None:

                if self.running:
                    print(
                        "[DECODER] "
                        "Decoder stream ended."
                    )

                break

            # Important:
            # Do NOT queue every frame.
            #
            # Replace the current frame with
            # the newest frame.
            #
            # This prevents latency buildup.
            with self.condition:

                self.latest_frame = frame

                self.frame_number += 1

                self.condition.notify_all()

    def get_latest(
        self,
        after_frame: int,
        timeout: float = 1.0,
    ):

        with self.condition:

            self.condition.wait_for(
                lambda:
                (
                    self.frame_number
                    > after_frame
                )
                or not self.running,
                timeout=timeout,
            )

            if (
                self.latest_frame
                is None
            ):
                return (
                    after_frame,
                    None,
                )

            if (
                self.frame_number
                <= after_frame
            ):
                return (
                    after_frame,
                    None,
                )

            return (
                self.frame_number,
                self.latest_frame,
            )

    def stop(self):

        self.running = False

        if self.process:

            try:
                self.process.terminate()
            except Exception:
                pass

        with self.condition:
            self.condition.notify_all()


# ============================================================
# H.264 RTSP publisher
# ============================================================

class RtspPublisher:
    """
    Accepts raw BGR frames from Python.

    Re-encodes them as low-latency H.264.

    Publishes them to MediaMTX.
    """

    def __init__(
        self,
        url: str,
        width: int,
        height: int,
        fps: float,
        bitrate: str = "4M",
    ):
        self.url = url

        self.width = width
        self.height = height
        self.fps = fps
        self.bitrate = bitrate

        self.process = None

    def start(self):

        gop = max(
            1,
            round(self.fps),
        )

        command = [
            "ffmpeg",

            "-hide_banner",
            "-loglevel",
            "warning",

            # --------------------------------
            # Raw frames from Python
            # --------------------------------

            "-f",
            "rawvideo",

            "-pix_fmt",
            "bgr24",

            "-video_size",
            f"{self.width}x{self.height}",

            "-framerate",
            str(self.fps),

            "-i",
            "pipe:0",

            "-an",

            # --------------------------------
            # Pi 5 software H264 encoding
            # --------------------------------

            "-c:v",
            "libx264",

            "-preset",
            "ultrafast",

            "-tune",
            "zerolatency",

            "-profile:v",
            "baseline",

            "-pix_fmt",
            "yuv420p",

            # No B-frames = less latency
            "-bf",
            "0",

            # 1 second GOP
            "-g",
            str(gop),

            "-keyint_min",
            str(gop),

            "-sc_threshold",
            "0",

            # --------------------------------
            # Bitrate
            # --------------------------------

            "-b:v",
            self.bitrate,

            "-maxrate",
            self.bitrate,

            "-bufsize",
            "1M",

            # --------------------------------
            # Publish into MediaMTX
            # --------------------------------

            "-f",
            "rtsp",

            "-rtsp_transport",
            "tcp",

            self.url,
        ]

        print(
            "[ENCODER] Starting:"
        )

        print(
            " ".join(command)
        )

        self.process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            bufsize=0,
        )

    def write(self, frame):

        if not self.process:
            return False

        if not self.process.stdin:
            return False

        if self.process.poll() is not None:

            print(
                "[ENCODER] "
                "FFmpeg exited unexpectedly."
            )

            return False

        try:

            # Avoid an unnecessary frame.tobytes()
            # allocation.
            raw = memoryview(
                frame
            ).cast("B")

            self.process.stdin.write(
                raw
            )

            return True

        except BrokenPipeError:

            print(
                "[ENCODER] Broken pipe."
            )

            return False

    def stop(self):

        if not self.process:
            return

        try:

            if self.process.stdin:
                self.process.stdin.close()

        except Exception:
            pass

        try:
            self.process.terminate()
        except Exception:
            pass


# ============================================================
# Coordinate handling
# ============================================================

def resolve_bbox(
    track: Track,
    width: int,
    height: int,
):

    x1 = track.x1
    y1 = track.y1

    x2 = track.x2
    y2 = track.y2

    # --------------------------------------------
    # Normalized 0 -> 1 coordinates
    # --------------------------------------------

    if track.normalized:

        x1 *= width
        x2 *= width

        y1 *= height
        y2 *= height

    # --------------------------------------------
    # Different model resolution
    #
    # Example:
    # inference occurs at 640x640
    # output video is 1280x720
    # --------------------------------------------

    elif (
        track.source_width
        and track.source_height
    ):

        sx = (
            width
            / track.source_width
        )

        sy = (
            height
            / track.source_height
        )

        x1 *= sx
        x2 *= sx

        y1 *= sy
        y2 *= sy

    x1 = int(
        max(
            0,
            min(
                width - 1,
                x1,
            ),
        )
    )

    y1 = int(
        max(
            0,
            min(
                height - 1,
                y1,
            ),
        )
    )

    x2 = int(
        max(
            0,
            min(
                width - 1,
                x2,
            ),
        )
    )

    y2 = int(
        max(
            0,
            min(
                height - 1,
                y2,
            ),
        )
    )

    # Make sure coordinates are ordered
    if x2 < x1:
        x1, x2 = x2, x1

    if y2 < y1:
        y1, y2 = y2, y1

    return (
        x1,
        y1,
        x2,
        y2,
    )


# ============================================================
# Overlay
# ============================================================

def draw_track(
    frame,
    track: Track,
):

    height, width = (
        frame.shape[:2]
    )

    x1, y1, x2, y2 = (
        resolve_bbox(
            track,
            width,
            height,
        )
    )

    # Bounding box
    cv2.rectangle(
        frame,
        (x1, y1),
        (x2, y2),
        (0, 255, 0),
        2,
    )

    cx = (
        x1 + x2
    ) // 2

    cy = (
        y1 + y2
    ) // 2

    # Center marker
    cv2.circle(
        frame,
        (cx, cy),
        5,
        (0, 0, 255),
        -1,
    )

    # Crosshair
    length = 15

    cv2.line(
        frame,
        (
            cx - length,
            cy,
        ),
        (
            cx + length,
            cy,
        ),
        (0, 0, 255),
        2,
    )

    cv2.line(
        frame,
        (
            cx,
            cy - length,
        ),
        (
            cx,
            cy + length,
        ),
        (0, 0, 255),
        2,
    )

    text = (
        f"ID {track.track_id} | "
        f"{track.label} | "
        f"{track.confidence:.2f}"
    )

    cv2.putText(
        frame,
        text,
        (
            x1,
            max(
                y1 - 8,
                25,
            ),
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )

    return frame


# ============================================================
# Main application
# ============================================================

def run(args):

    # --------------------------------------------------------
    # Check input stream
    # --------------------------------------------------------

    (
        codec,
        width,
        height,
        fps,
    ) = probe_stream(
        args.input
    )

    if codec not in (
        "hevc",
        "h265",
    ):
        print(
            "[WARNING] Input codec is "
            f"{codec}, not HEVC/H265."
        )

    # --------------------------------------------------------
    # Track receiver
    # --------------------------------------------------------

    track_state = TrackState()

    receiver = TrackReceiver(
        state=track_state,
        host=args.track_host,
        port=args.track_port,
    )

    receiver.start()

    # --------------------------------------------------------
    # HEVC decoder
    # --------------------------------------------------------

    decoder = RtspDecoder(
        url=args.input,
        width=width,
        height=height,
    )

    decoder.start()

    # --------------------------------------------------------
    # H264 publisher
    # --------------------------------------------------------

    publisher = RtspPublisher(
        url=args.output,
        width=width,
        height=height,
        fps=fps,
        bitrate=args.bitrate,
    )

    publisher.start()

    print()
    print(
        "========================================="
    )
    print(
        " Tracking RTSP bridge running"
    )
    print(
        "========================================="
    )

    print(
        f"Input : {args.input}"
    )

    print(
        f"       {codec.upper()} "
        f"{width}x{height} "
        f"{fps:.1f} FPS"
    )

    print(
        f"Tracks: UDP {args.track_port}"
    )

    print(
        f"Output: {args.output}"
    )

    print(
        "       H264 / libx264 / ultrafast"
    )

    print(
        "========================================="
    )
    print()

    last_frame = 0

    try:

        while True:

            (
                frame_number,
                frame,
            ) = decoder.get_latest(
                after_frame=last_frame,
                timeout=1.0,
            )

            if frame is None:
                continue

            last_frame = frame_number
            
        # ==========================================
        # MODEL INFERENCE HAPPENS HERE
        # ==========================================

        detections = model.infer(frame)

        # Example result:
        #
        # detections = [
        #     {
        #         "track_id": 1,
        #         "x1": 400,
        #         "y1": 200,
        #         "x2": 600,
        #         "y2": 400,
        #         "label": "vehicle",
        #         "confidence": 0.91
        #     }
        # ]

        # ==========================================
        # DRAW RESULTS
        # ==========================================

    for detection in detections:

        track = Track(
            track_id=detection["track_id"],
            x1=detection["x1"],
            y1=detection["y1"],
            x2=detection["x2"],
            y2=detection["y2"],
            label=detection["label"],
            confidence=detection["confidence"],
        )

        draw_track(
            frame,
            track,
        )

    # ==========================================
    # SEND ANNOTATED FRAME TO RTSP
    # ==========================================

    publisher.write(frame)

            # ------------------------------------------------
            # Latest track
            # ------------------------------------------------

            track = track_state.get()

            if track:

                age = (
                    time.monotonic()
                    - track.received_time
                )

                if (
                    age
                    <= args.track_timeout
                ):

                    draw_track(
                        frame,
                        track,
                    )

                else:
                    track_state.clear()

            # ------------------------------------------------
            # Optional system status
            # ------------------------------------------------

            if args.show_status:

                cv2.putText(
                    frame,
                    (
                        f"FRAME {frame_number}"
                    ),
                    (15, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )

            # ------------------------------------------------
            # H264 encode -> MediaMTX
            # ------------------------------------------------

            if not publisher.write(
                frame
            ):
                raise RuntimeError(
                    "RTSP publisher failed."
                )

    except KeyboardInterrupt:

        print(
            "\n[APP] Shutting down..."
        )

    finally:

        receiver.stop()
        decoder.stop()
        publisher.stop()


# ============================================================
# CLI
# ============================================================

def main():

    parser = argparse.ArgumentParser(
        description=(
            "Decode H265 MediaMTX RTSP, "
            "overlay track information, "
            "and republish H264 RTSP."
        )
    )

    parser.add_argument(
        "--input",
        default=(
            "rtsp://127.0.0.1:8554/eo"
        ),
        help=(
            "H265 input stream"
        ),
    )

    parser.add_argument(
        "--output",
        default=(
            "rtsp://127.0.0.1:8554/tracking"
        ),
        help=(
            "H264 MediaMTX output path"
        ),
    )

    parser.add_argument(
        "--track-host",
        default="0.0.0.0",
    )

    parser.add_argument(
        "--track-port",
        type=int,
        default=5005,
    )

    parser.add_argument(
        "--track-timeout",
        type=float,
        default=0.75,
        help=(
            "Remove bounding box if no "
            "track update is received "
            "for this many seconds."
        ),
    )

    parser.add_argument(
        "--bitrate",
        default="4M",
        help=(
            "H264 output bitrate."
        ),
    )

    parser.add_argument(
        "--show-status",
        action="store_true",
    )

    args = parser.parse_args()

    run(args)


if __name__ == "__main__":
    main()