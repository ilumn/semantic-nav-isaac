#!/usr/bin/env bash
set -euo pipefail

tool_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
scene_dir="$(cd -- "${tool_dir}/.." && pwd)"
output="${1:-${scene_dir}/generated/tools/dae_to_obj}"
mkdir -p -- "$(dirname -- "${output}")"

compiler="${CXX:-c++}"
"${compiler}" -std=c++17 -O2 -Wall -Wextra -Wpedantic \
  "${tool_dir}/dae_to_obj.cpp" -lassimp -o "${output}"
printf '%s\n' "${output}"
