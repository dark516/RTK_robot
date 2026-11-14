#!/usr/bin/env bash
set -euo pipefail

export PATH="/opt/homebrew/opt/python@3.12/libexec/bin:/opt/homebrew/bin:$PATH"
source "$HOME/esp/esp-idf/export.sh"

PORT="${PORT:-/dev/cu.usbmodem11401}"

idf.py -p "$PORT" monitor
