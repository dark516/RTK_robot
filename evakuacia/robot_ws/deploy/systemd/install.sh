#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

SERVICE_NAME="rover-bringup.service"
ENV_NAME="rover-bringup"
SERVICE_SRC="${SCRIPT_DIR}/${SERVICE_NAME}"
ENV_SRC="${SCRIPT_DIR}/${ENV_NAME}.env"
SERVICE_DST="/etc/systemd/system/${SERVICE_NAME}"
ENV_DST="/etc/default/${ENV_NAME}"
RUN_USER="${ROVER_SERVICE_USER:-pi}"
RUN_GROUP="${ROVER_SERVICE_GROUP:-${RUN_USER}}"
WORKSPACE="${ROVER_WS:-${REPO_ROOT}}"

if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  cat >&2 <<EOF
Workspace install/setup.bash was not found:
  ${WORKSPACE}/install/setup.bash

Build the workspace first, or pass the workspace path explicitly:
  ROVER_WS=/home/pi/sverk_rover ${SCRIPT_DIR}/install.sh
EOF
  exit 1
fi

tmp_service="$(mktemp)"
sed \
  -e "s|^User=.*|User=${RUN_USER}|" \
  -e "s|^Group=.*|Group=${RUN_GROUP}|" \
  -e "s|^WorkingDirectory=.*|WorkingDirectory=${WORKSPACE}|" \
  "${SERVICE_SRC}" > "${tmp_service}"

sudo install -m 0644 "${tmp_service}" "${SERVICE_DST}"
rm -f "${tmp_service}"

if [[ ! -f "${ENV_DST}" ]]; then
  tmp_env="$(mktemp)"
  sed -e "s|^ROVER_WS=.*|ROVER_WS=${WORKSPACE}|" "${ENV_SRC}" > "${tmp_env}"
  sudo install -m 0640 -o root -g "${RUN_GROUP}" "${tmp_env}" "${ENV_DST}"
  rm -f "${tmp_env}"
else
  echo "Keeping existing ${ENV_DST}"
fi

sudo systemctl daemon-reload
sudo systemctl enable "${SERVICE_NAME}"

cat <<EOF
Installed ${SERVICE_NAME}.

Edit launch settings:
  sudo nano ${ENV_DST}

Start/stop/status:
  sudo systemctl start ${SERVICE_NAME}
  sudo systemctl stop ${SERVICE_NAME}
  systemctl status ${SERVICE_NAME}

Logs:
  journalctl -u ${SERVICE_NAME} -f
EOF
