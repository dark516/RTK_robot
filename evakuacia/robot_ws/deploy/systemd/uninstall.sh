#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="rover-bringup.service"

sudo systemctl disable --now "${SERVICE_NAME}" 2>/dev/null || true
sudo rm -f "/etc/systemd/system/${SERVICE_NAME}"
sudo systemctl daemon-reload

cat <<EOF
Removed ${SERVICE_NAME}.

The environment file was left in place:
  /etc/default/rover-bringup

Remove it manually if it is no longer needed.
EOF
