#!/usr/bin/env bash
set -euo pipefail

WHISPER_BIN="${WHISPER_BIN:-$(command -v whisper)}"
WHISPER_PY="$(head -n 1 "$WHISPER_BIN" | sed 's/^#!//')"

exec "$WHISPER_PY" "$(dirname "$0")/serial_whisper_bridge.py" "$@"
