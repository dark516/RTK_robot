#!/usr/bin/env bash
set -euo pipefail

VOICE_DIR="${VOICE_DIR:-$HOME/sverk_rover/tts_voices}"
VOICE_NAME="${VOICE_NAME:-ru_RU-irina-medium}"
BASE_URL="https://huggingface.co/rhasspy/piper-voices/resolve/main/ru/ru_RU/irina/medium"

mkdir -p "$VOICE_DIR"

if python3 -m piper.download_voices --data-dir "$VOICE_DIR" "$VOICE_NAME"; then
  echo
  echo "Piper Russian voice installed with piper.download_voices:"
  echo "  $VOICE_DIR/$VOICE_NAME.onnx"
  echo "  $VOICE_DIR/$VOICE_NAME.onnx.json"
  exit 0
fi

echo
echo "piper.download_voices failed; falling back to direct Hugging Face download."
echo "If Piper is not installed yet, run: python3 -m pip install -U piper-tts"

download() {
  local file_name="$1"
  local target="$VOICE_DIR/$file_name"
  if [[ -s "$target" ]]; then
    echo "Already exists: $target"
    return
  fi

  echo "Downloading $file_name..."
  curl -L --fail --retry 3 --output "$target" "$BASE_URL/$file_name"
}

download "$VOICE_NAME.onnx"
download "$VOICE_NAME.onnx.json"

echo
echo "Piper Russian voice installed:"
echo "  $VOICE_DIR/$VOICE_NAME.onnx"
echo "  $VOICE_DIR/$VOICE_NAME.onnx.json"
