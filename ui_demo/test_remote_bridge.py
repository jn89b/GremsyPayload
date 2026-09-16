"""Smallest check that a new peer can take over the listen bridge from a stale one.

Run: python3 ui_demo/test_remote_bridge.py
"""
import json
import os
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(__file__))
from remote_bridge import BridgeConfig, TcpCommandBridge  # noqa: E402


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def main() -> None:
    port = _free_port()
    bridge = TcpCommandBridge(
        BridgeConfig(role="listen", host="127.0.0.1", port=port, reconnect_interval=0.1),
        logger=lambda m: None,
    )
    bridge.start()
    threading.Thread(
        target=bridge.serve_commands, args=(lambda c, p: (True, "ok"),), daemon=True
    ).start()
    time.sleep(0.3)

    # Stale peer: connected, then silent forever (half-open link, no FIN).
    stale = socket.create_connection(("127.0.0.1", port), timeout=2)
    time.sleep(0.3)

    # Fresh app instance must get served even though the stale peer holds the slot.
    fresh = socket.create_connection(("127.0.0.1", port), timeout=2)
    fresh.settimeout(2)
    fresh.sendall((json.dumps({"type": "request", "msg_id": "1", "command": "X", "params": []}) + "\n").encode())
    reply = json.loads(fresh.makefile().readline())
    assert reply["type"] == "ack", reply

    stale.settimeout(2)
    assert stale.recv(1) == b"", "stale peer should have been closed"

    bridge.stop()
    print("ok")


if __name__ == "__main__":
    main()
