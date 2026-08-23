#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="${ROOT}/build/vulkan_engine"

if [[ ! -x "$BIN" ]]; then
  echo "Binary not found: $BIN"
  echo "Build first: cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build -j"
  exit 1
fi

# Homebrew MoltenVK ICD (macOS)
if [[ "$(uname -s)" == "Darwin" ]]; then
  for candidate in \
    "$(brew --prefix)/etc/vulkan/icd.d/MoltenVK_icd.json" \
    "$(brew --prefix molten-vk)/etc/vulkan/icd.d/MoltenVK_icd.json" \
    "$(brew --prefix molten-vk)/share/vulkan/icd.d/MoltenVK_icd.json"
  do
    if [[ -f "$candidate" ]]; then
      export VK_ICD_FILENAMES="$candidate"
      break
    fi
  done

  for layer_dir in \
    "$(brew --prefix)/share/vulkan/explicit_layer.d" \
    "$(brew --prefix vulkan-validationlayers)/share/vulkan/explicit_layer.d"
  do
    if [[ -d "$layer_dir" ]]; then
      export VK_LAYER_PATH="$layer_dir"
      break
    fi
  done
fi

exec "$BIN" "$@"
