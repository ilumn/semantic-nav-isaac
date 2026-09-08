#!/usr/bin/env bash
# Run checks that do not require ROS, Isaac Sim, a display, or a GPU.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
PORT_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd -P)"
SCENE_ROOT="${SCRIPT_DIR}/scene"

export PYTHONDONTWRITEBYTECODE=1

python3 - "${PORT_ROOT}" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
excluded = {".git", ".venv", "build", "generated", "install", "log", "__pycache__"}
for path in root.rglob("*.py"):
    if any(part in excluded for part in path.parts):
        continue
    compile(path.read_text(encoding="utf-8"), str(path), "exec")

with (root / "dev/isaac_sim/scene/config/scene_manifest.json").open(encoding="utf-8") as stream:
    json.load(stream)
PY

while IFS= read -r -d '' script; do
  bash -n "${script}"
done < <(find "${SCRIPT_DIR}" "${PORT_ROOT}/dev/turtlebot3/ros2_ws" -type f -name '*.sh' \
  -not -path '*/build/*' -not -path '*/install/*' -print0)

(
  cd -- "${SCENE_ROOT}"
  python3 -m unittest discover -s tests -v
)

printf '[PASS] Portable source checks completed successfully\n'
