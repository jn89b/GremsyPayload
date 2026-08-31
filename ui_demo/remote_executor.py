#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""
Remote executor service.

Runs on the machine physically connected to payload/gimbal.
Receives remote commands over TCP and executes a safe command subset.
"""

import argparse
import os
import sys
import threading
import time
from typing import List, Tuple

# Add libs path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'libs'))

from remote_bridge import BridgeConfig, TcpCommandBridge

try:
    from payload_sdk import PayloadSdkInterface
    from config import ConnectionConfig
except ImportError as exc:
    print(f"Error importing payload SDK modules: {exc}")
    sys.exit(1)

CMD_PAYLOAD_TOUCH = "PAYLOAD_TOUCH"
CMD_PAYLOAD_TRACK = "PAYLOAD_TRACK"


class RemoteExecutor:
    """Execute remote UI commands on local payload SDK."""

    def __init__(
        self,
        role: str,
        host: str,
        port: int,
        token: str,
        ack_timeout: float,
        retries: int,
        payload_ip: str,
    ):
        self.bridge = TcpCommandBridge(
            BridgeConfig(
                role=role,
                host=host,
                port=port,
                token=token,
                ack_timeout=ack_timeout,
                retry_count=retries,
            ),
            logger=self._log,
        )
        self.sdk = None
        self.running = True
        self.is_connected = False
        self.payload_ip = payload_ip

    def start(self) -> int:
        """Start SDK and bridge service."""
        ConnectionConfig.UDP_IP_TARGET = self.payload_ip
        self.sdk = PayloadSdkInterface()

        if not self.sdk.sdkInitConnection():
            self._log("payload SDK init failed")
            return 1

        if not self._wait_payload_connection(timeout=8.0):
            self._log("payload connection timeout")
            self.stop()
            return 1

        self.bridge.start()
        self._log("executor ready, waiting for commands")

        try:
            self.bridge.serve_commands(self._handle_command)
        except KeyboardInterrupt:
            self._log("executor interrupted")
        finally:
            self.stop()

        return 0

    def stop(self) -> None:
        """Stop executor."""
        self.running = False
        self.bridge.stop()
        if self.sdk:
            self.sdk.sdkQuit()
            self.sdk = None
        self._log("executor stopped")
    
    def _wait_payload_connection(self, timeout: float) -> bool:
        start = time.time()
        while self.running and (time.time() - start) < timeout:
            if self.sdk and self.sdk.checkPayloadConnection():
                self.is_connected = True
                self._log(f"payload connected at {self.payload_ip}")
                return True
            time.sleep(0.1)
        return False

    def _handle_command(self, command: str, params: List) -> Tuple[bool, str]:
        if not self.sdk or not self.is_connected:
            return False, "payload not connected"

        try:
            # MVP whitelist: tracking mode + touch-to-track
            if command == CMD_PAYLOAD_TOUCH:
                x = int(params[0]) if len(params) > 0 else 960
                y = int(params[1]) if len(params) > 1 else 540
        
                self.sdk.setPayloadObjectTrackingPosition(x, y)
                print("Sending commands at", x,y)
                return True, f"tracking position set x={x} y={y}"

            if command == CMD_PAYLOAD_TRACK:
                enable = int(params[0]) if params else 0
                self.sdk.setPayloadObjectTrackingMode(enable)
                return True, f"tracking mode set {enable}"

            return False, f"unsupported command: {command}"

        except Exception as exc:
            return False, f"execution error: {exc}"

    @staticmethod
    def _log(message: str) -> None:
        print(f"[REMOTE_EXECUTOR] {message}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Payload SDK Remote Executor")
    parser.add_argument("--role", choices=["connect", "listen"], default="listen")
    parser.add_argument("--host", default="0.0.0.0", help="Bind/connect host for bridge")
    parser.add_argument("--port", type=int, default=5000, help="Bind/connect port for bridge")
    parser.add_argument("--token", default="", help="Optional shared token")
    parser.add_argument("--ack-timeout", type=float, default=1.5, help="ACK timeout seconds")
    parser.add_argument("--retries", type=int, default=2, help="Retry count")
    parser.add_argument("--payload-ip", default=ConnectionConfig.UDP_IP_TARGET, help="Payload/gimbal IP")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    app = RemoteExecutor(
        role=args.role,
        host=args.host,
        port=args.port,
        token=args.token,
        ack_timeout=args.ack_timeout,
        retries=args.retries,
        payload_ip=args.payload_ip,
    )
    return app.start()


if __name__ == "__main__":
    raise SystemExit(main())
