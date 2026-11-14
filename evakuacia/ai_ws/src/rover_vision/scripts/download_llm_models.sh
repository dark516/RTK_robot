#!/usr/bin/env bash
set -euo pipefail

MODELS_ROOT="${MODELS_ROOT:-${XDG_DATA_HOME:-$HOME/.local/share}/rtk-evacuation/models/llm}"
selection="${1:-9b}"
if [[ "$#" -gt 1 ]]; then
  echo "Usage: $0 [2b|9b|35b-a3b|all]" >&2
  exit 2
fi
case "$selection" in
  2b|9b|35b-a3b|all) ;;
  -h|--help) echo "Usage: $0 [2b|9b|35b-a3b|all] (default: 9b; override MODELS_ROOT)"; exit 0 ;;
  *) echo "Unknown model: $selection; choose 2b, 9b, 35b-a3b or all" >&2; exit 2 ;;
esac
command -v aria2c >/dev/null || { echo "Install aria2 before downloading models" >&2; exit 1; }

download_model() {
  local url="$1"
  local directory="$2"
  local filename="$3"
  local expected_size="$4"
  local target="${directory}/${filename}"

  mkdir -p "${directory}"
  if [[ -f "${target}" ]] && [[ "$(stat -c %s "${target}")" == "${expected_size}" ]] && [[ ! -f "${target}.aria2" ]]; then
    echo "Already downloaded: ${filename}"
    return
  fi

  aria2c \
    --continue=true \
    --max-connection-per-server=8 \
    --split=8 \
    --min-split-size=4M \
    --file-allocation=none \
    --console-log-level=notice \
    --summary-interval=30 \
    --dir="${directory}" \
    --out="${filename}" \
    "${url}"

  local actual_size
  actual_size="$(stat -c %s "${target}")"
  if [[ "${actual_size}" != "${expected_size}" ]]; then
    echo "Size mismatch for ${filename}: ${actual_size}, expected ${expected_size}" >&2
    exit 1
  fi
}

if [[ "$selection" == "2b" || "$selection" == "all" ]]; then
  download_model \
    "https://hf-mirror.com/jc-builds/Qwen3.5-2B-Q4_K_M-GGUF/resolve/main/Qwen3.5-2B-Q4_K_M.gguf" \
    "${MODELS_ROOT}/qwen3.5-2b" \
    "Qwen3.5-2B-Q4_K_M.gguf" \
    "1280835840"
fi

if [[ "$selection" == "9b" || "$selection" == "all" ]]; then
  download_model \
    "https://hf-mirror.com/jc-builds/Qwen3.5-9B-Q4_K_M-GGUF/resolve/main/Qwen3.5-9B-Q4_K_M.gguf" \
    "${MODELS_ROOT}/qwen3.5-9b" \
    "Qwen3.5-9B-Q4_K_M.gguf" \
    "5680522464"
fi

if [[ "$selection" == "35b-a3b" || "$selection" == "all" ]]; then
  download_model \
    "https://hf-mirror.com/bartowski/Qwen_Qwen3.5-35B-A3B-GGUF/resolve/main/Qwen_Qwen3.5-35B-A3B-Q3_K_S.gguf" \
    "${MODELS_ROOT}/qwen3.5-35b-a3b" \
    "Qwen_Qwen3.5-35B-A3B-Q3_K_S.gguf" \
    "16404920128"
fi

echo "Selected local LLM model files are ready."
