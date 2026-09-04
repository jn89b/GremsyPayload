#!/usr/bin/env bash
set -euo pipefail

# Name of the systemd service, can set to anything, but the var is used to restart and enable the service
SERVICE=gremsy

# Either the user running the script, or sudo
RUN_USER="${SUDO_USER:-$(id -un)}"

# BASH_SOURCE[0] is taking everything except the script name from the path (like "." from "./install_gremsy_service.sh")
# The dirname of BASH_SOURCE[0] with the /.. will take the exact path (like "/home/user/GremsyPayload")
# This gets the directory that the install script is in dynamically
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Defaults to port 5000 if nothing is set
PORT="${PORT:-5000}"
EXTRA_ARGS="${EXTRA_ARGS:-}"

# There is three different ways that venv is created (.venv is the best if youre reading this), so we check all possible ones
if [[ -z "${PYTHON:-}" ]]; then
    for v in venv .venv payloadsdk_env; do
        [[ -x "$REPO_DIR/$v/bin/python" ]] && PYTHON="$REPO_DIR/$v/bin/python" && break
    done
fi
[[ -x "${PYTHON:-}" ]] || { echo "ERROR: no venv python in $REPO_DIR (venv/, .venv/, payloadsdk_env/). Set PYTHON=/path/to/python"; exit 1; }

# Make the system service and save it to the /etc/systemd files
sudo tee "/etc/systemd/system/$SERVICE.service" >/dev/null <<EOF
[Unit]
Description=Gremsy Payload Remote Executor
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$RUN_USER
WorkingDirectory=$REPO_DIR/ui_demo
ExecStart=$PYTHON $REPO_DIR/ui_demo/remote_executor.py --role listen --host 0.0.0.0 --port $PORT $EXTRA_ARGS
Restart=always
RestartSec=5
TimeoutStopSec=10

[Install]
WantedBy=multi-user.target
EOF

# This reloads the services, starts and enables the one we just created on boot
sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE.service"
sudo systemctl restart "$SERVICE.service"

# Print the system service status to the user to verify if anything else is wrong
sudo systemctl status "$SERVICE.service" --no-pager -l
echo "Logs: journalctl -u $SERVICE -f"
