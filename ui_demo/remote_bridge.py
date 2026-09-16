#!/usr/bin/env python3
"""
Remote command bridge for multi-machine Payload SDK control.

Protocol:
- JSON messages over TCP, one message per line.
- Request fields: type=request, msg_id, command, params, timestamp, protocol_version.
- Response fields: type=ack|nack, msg_id, timestamp, protocol_version, detail(optional).
"""

import json
import socket
import threading
import time
import uuid
from collections import deque
from typing import Callable, Dict, List, Optional, Tuple

PROTOCOL_VERSION = 1


class BridgeConfig:
    """Runtime configuration for command bridge transport."""

    def __init__(
        self,
        role: str,
        host: str,
        port: int,
        token: str = "",
        connect_timeout: float = 3.0,
        ack_timeout: float = 1.5,
        retry_count: int = 2,
        reconnect_interval: float = 1.0,
    ):
        self.role = role
        self.host = host
        self.port = int(port)
        self.token = token or ""
        self.connect_timeout = float(connect_timeout)
        self.ack_timeout = float(ack_timeout)
        self.retry_count = int(retry_count)
        self.reconnect_interval = float(reconnect_interval)


class TcpCommandBridge:
    """Persistent TCP endpoint that can connect or listen, then exchange JSON-line messages."""

    def __init__(self, cfg: BridgeConfig, logger: Optional[Callable[[str], None]] = None):
        if cfg.role not in ("connect", "listen"):
            raise ValueError("Bridge role must be 'connect' or 'listen'")

        self.cfg = cfg
        self._logger = logger or print
        self._running = False

        self._server_sock: Optional[socket.socket] = None
        self._conn: Optional[socket.socket] = None
        self._conn_lock = threading.Lock()
        self._io_lock = threading.Lock()
        self._conn_ready = threading.Event()
        self._manager_thread: Optional[threading.Thread] = None

    def start(self) -> None:
        """Start connection manager thread."""
        if self._running:
            return

        self._running = True
        self._manager_thread = threading.Thread(target=self._connection_manager, daemon=True)
        self._manager_thread.start()
        self._log(
            f"bridge start role={self.cfg.role} host={self.cfg.host} port={self.cfg.port}"
        )

    def stop(self) -> None:
        """Stop bridge and close sockets."""
        self._running = False
        self._conn_ready.clear()

        with self._conn_lock:
            if self._conn:
                try:
                    self._conn.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                try:
                    self._conn.close()
                except OSError:
                    pass
                self._conn = None

            if self._server_sock:
                try:
                    self._server_sock.close()
                except OSError:
                    pass
                self._server_sock = None

        self._log("bridge stopped")

    def wait_until_connected(self, timeout: Optional[float] = None) -> bool:
        """Wait for an active peer connection."""
        return self._conn_ready.wait(timeout)

    def is_connected(self) -> bool:
        """Return True when a peer connection exists."""
        with self._conn_lock:
            return self._conn is not None

    def send_command(self, command: str, params: List, ack_required: bool = True) -> Tuple[bool, str]:
        """Send request and wait for ack/nack with timeout and retry."""
        attempts = max(0, self.cfg.retry_count) + 1

        for attempt in range(1, attempts + 1):
            if not self.wait_until_connected(self.cfg.connect_timeout):
                self._log(f"send command={command} attempt={attempt} no connection")
                continue

            conn = self._get_conn()
            if not conn:
                continue

            msg_id = str(uuid.uuid4())
            payload = {
                "type": "request",
                "msg_id": msg_id,
                "command": command,
                "params": params or [],
                "timestamp": time.time(),
                "protocol_version": PROTOCOL_VERSION,
                "ack_required": bool(ack_required),
            }
            if self.cfg.token:
                payload["token"] = self.cfg.token

            try:
                with self._io_lock:
                    self._send_json_line(conn, payload)
                    if not ack_required:
                        return True, "sent"

                    resp = self._recv_json_line(conn, timeout=self.cfg.ack_timeout)

                if not resp:
                    raise TimeoutError("ack timeout")

                if resp.get("msg_id") != msg_id:
                    raise RuntimeError("ack msg_id mismatch")

                resp_type = resp.get("type")
                detail = str(resp.get("detail", ""))
                if resp_type == "ack":
                    return True, detail or "ack"
                return False, detail or "nack"

            except Exception as exc:
                self._log(f"send command={command} attempt={attempt} error={exc}")
                self._drop_connection()

        return False, "retry exhausted"

    def serve_commands(self, handler: Callable[[str, List], Tuple[bool, str]]) -> None:
        """Run command-receive loop and respond with ack/nack."""
        seen_ids = deque(maxlen=512)

        while self._running:
            if not self.wait_until_connected(timeout=1.0):
                continue

            conn = self._get_conn()
            if not conn:
                continue

            try:
                req = self._recv_json_line(conn, timeout=1.0)
                if not req:
                    continue

                msg_type = req.get("type")
                msg_id = req.get("msg_id", "")
                command = req.get("command")
                params = req.get("params", [])

                if msg_type != "request" or not msg_id:
                    self._safe_reply(conn, msg_id, False, "invalid request frame")
                    continue

                if self.cfg.token and req.get("token", "") != self.cfg.token:
                    self._safe_reply(conn, msg_id, False, "unauthorized token")
                    continue

                if msg_id in seen_ids:
                    self._safe_reply(conn, msg_id, True, "duplicate acknowledged")
                    continue

                seen_ids.append(msg_id)

                ok, detail = handler(command, params)
                self._safe_reply(conn, msg_id, bool(ok), detail)

            except (socket.timeout, TimeoutError):
                continue
            except Exception as exc:
                self._log(f"serve loop error={exc}")
                self._drop_connection(conn)

    def _safe_reply(self, conn: socket.socket, msg_id: str, ok: bool, detail: str) -> None:
        frame = {
            "type": "ack" if ok else "nack",
            "msg_id": msg_id,
            "timestamp": time.time(),
            "protocol_version": PROTOCOL_VERSION,
            "detail": detail,
        }
        with self._io_lock:
            self._send_json_line(conn, frame)

    def _connection_manager(self) -> None:
        while self._running:
            if self.cfg.role == "connect":
                self._run_connect_cycle()
            else:
                self._run_listen_cycle()

            if self._running:
                time.sleep(self.cfg.reconnect_interval)

    def _run_connect_cycle(self) -> None:
        if self.is_connected():
            return

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(self.cfg.connect_timeout)
        try:
            sock.connect((self.cfg.host, self.cfg.port))
            sock.settimeout(None)
            with self._conn_lock:
                self._conn = sock
                self._conn_ready.set()
            self._log(f"bridge connected to {self.cfg.host}:{self.cfg.port}")
        except Exception as exc:
            try:
                sock.close()
            except OSError:
                pass
            self._log(f"bridge connect failed error={exc}")

    def _run_listen_cycle(self) -> None:
        if self._server_sock is None:
            self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._server_sock.bind((self.cfg.host, self.cfg.port))
            self._server_sock.listen(1)
            self._server_sock.settimeout(1.0)
            self._log(f"bridge listening on {self.cfg.host}:{self.cfg.port}")

        # ponytail: newest peer wins. Always accept so a half-open stale
        # peer (no FIN, no keepalive) can't hold the single slot forever
        # and fill the listen backlog, which makes new connects time out.
        try:
            conn, addr = self._server_sock.accept()
            conn.settimeout(None)
            with self._conn_lock:
                old = self._conn
                self._conn = conn
                self._conn_ready.set()
            if old is not None:
                try:
                    old.close()
                except OSError:
                    pass
                self._log("bridge replaced stale peer")
            self._log(f"bridge accepted peer={addr[0]}:{addr[1]}")
        except socket.timeout:
            return
        except Exception as exc:
            self._log(f"bridge listen accept failed error={exc}")

    def _drop_connection(self, conn: Optional[socket.socket] = None) -> None:
        with self._conn_lock:
            if conn is not None and self._conn is not conn:
                return  # already replaced by a newer peer
            if self._conn:
                try:
                    self._conn.close()
                except OSError:
                    pass
            self._conn = None
            self._conn_ready.clear()

    def _get_conn(self) -> Optional[socket.socket]:
        with self._conn_lock:
            return self._conn

    @staticmethod
    def _send_json_line(conn: socket.socket, payload: Dict) -> None:
        wire = (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")
        conn.sendall(wire)

    @staticmethod
    def _recv_json_line(conn: socket.socket, timeout: float) -> Optional[Dict]:
        conn.settimeout(timeout)
        chunks = []
        while True:
            b = conn.recv(1)
            if b == b"":
                raise ConnectionError("peer disconnected")
            if b == b"\n":
                break
            chunks.append(b)

        if not chunks:
            return None

        raw = b"".join(chunks).decode("utf-8")
        return json.loads(raw)

    def _log(self, message: str) -> None:
        self._logger(f"[REMOTE_BRIDGE] {message}")
