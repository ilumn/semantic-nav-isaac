#!/usr/bin/env bash
# Shared, host-facing helpers for the Isaac semantic-navigation launch scripts.

if [[ -n "${ISAAC_SEMANTIC_NAV_COMMON_LOADED:-}" ]]; then
  return 0
fi
ISAAC_SEMANTIC_NAV_COMMON_LOADED=1

ISAAC_TOOL_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
ISAAC_PORT_ROOT="$(cd -- "${ISAAC_TOOL_DIR}/../.." && pwd -P)"
ISAAC_DEFAULT_ROS_WS="${ISAAC_PORT_ROOT}/dev/turtlebot3/ros2_ws"
ISAAC_DEFAULT_SEMANTIC_VENV="${ISAAC_PORT_ROOT}/.venv"
ISAAC_DEFAULT_SCENE_SCRIPT="${ISAAC_TOOL_DIR}/scene/run_semantic_nav.py"
ISAAC_DEFAULT_STAGE_PATH="${ISAAC_TOOL_DIR}/generated/semantic_nav.usd"
ISAAC_EXPECTED_VERSION="${ISAAC_SIM_VERSION_EXPECTED:-6.0.1}"
ISAAC_RUNTIME_DIR="${ISAAC_RUNTIME_DIR:-${ISAAC_TOOL_DIR}/.runtime}"
ISAAC_LIVE_MODEL_ID="nvidia/LocateAnything-3B"
ISAAC_LIVE_MODEL_REVISION="c32291ca5e996f5a7a485845b4f57a233936bba0"
ISAAC_REFINER_MODEL="${ISAAC_PORT_ROOT}/dev/turtlebot3/external/semantic-nav-memory/assets/models/yolov8s-worldv2.pt"
ISAAC_REFINER_MODEL_SHA256="9b2c17ab6124a913e9b3a5c170617920d91b0f01111a8479da69f00e2cf27792"
ISAAC_CLIP_MODEL="${ISAAC_PORT_ROOT}/dev/turtlebot3/external/semantic-nav-memory/assets/models/ViT-B-32.pt"
ISAAC_CLIP_MODEL_SHA256="40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af"
ISAAC_CLIP_WHEEL="${ISAAC_TOOL_DIR}/vendor/clip-1.0-py3-none-any.whl"
ISAAC_CLIP_WHEEL_SHA256="b0246e0be945d1ebc0711093683da47f39f9ce5c5ba5a27f9a30a142ba93f46e"

isaac_info() {
  printf '[INFO] %s\n' "$*"
}

isaac_pass() {
  printf '[PASS] %s\n' "$*"
}

isaac_warn() {
  printf '[WARN] %s\n' "$*" >&2
}

isaac_fail() {
  printf '[FAIL] %s\n' "$*" >&2
}

isaac_die() {
  isaac_fail "$*"
  exit 1
}

isaac_is_true() {
  case "${1:-}" in
    1 | true | TRUE | yes | YES | on | ON) return 0 ;;
    *) return 1 ;;
  esac
}

isaac_load_env() {
  local env_file="${ISAAC_ENV_FILE:-${ISAAC_TOOL_DIR}/.env}"

  if [[ -f "${env_file}" ]]; then
    # .env is a trusted, user-owned shell fragment. Export values so Isaac's
    # child processes and the ROS 2 bridge see the same configuration.
    set -a
    # shellcheck disable=SC1090
    source "${env_file}"
    set +a
  fi

  ISAAC_EXPECTED_VERSION="${ISAAC_SIM_VERSION_EXPECTED:-6.0.1}"
  ISAAC_RUNTIME_DIR="${ISAAC_RUNTIME_DIR:-${ISAAC_TOOL_DIR}/.runtime}"
}

isaac_candidate_root() {
  local candidate="${1:-}"

  [[ -n "${candidate}" ]] || return 1
  if [[ -f "${candidate}" ]]; then
    candidate="$(cd -- "$(dirname -- "${candidate}")" && pwd -P)"
  elif [[ -d "${candidate}" ]]; then
    candidate="$(cd -- "${candidate}" && pwd -P)"
  else
    return 1
  fi

  if [[ -x "${candidate}/python.sh" || -x "${candidate}/isaac-sim.sh" ]]; then
    printf '%s\n' "${candidate}"
    return 0
  fi

  return 1
}

isaac_resolve_root() {
  local candidate resolved command_path
  local -a candidates=()

  [[ -n "${ISAAC_SIM_PATH:-}" ]] && candidates+=("${ISAAC_SIM_PATH}")
  [[ -n "${ISAAC_SIM_ROOT:-}" ]] && candidates+=("${ISAAC_SIM_ROOT}")
  [[ -n "${ISAAC_PYTHON:-}" ]] && candidates+=("${ISAAC_PYTHON}")

  candidates+=(
    "/opt/isaac-sim"
    "/usr/local/isaac-sim"
    "${HOME}/isaac-sim"
    "${HOME}/isaacsim"
    "${HOME}/.local/share/ov/pkg/isaac-sim-${ISAAC_EXPECTED_VERSION}"
    "${HOME}/.local/share/ov/pkg/isaac_sim-${ISAAC_EXPECTED_VERSION}"
    "${HOME}/.local/share/ov/pkg/isaac_sim"
  )

  shopt -s nullglob
  for candidate in \
    "${HOME}"/.local/share/ov/pkg/isaac-sim-* \
    "${HOME}"/.local/share/ov/pkg/isaac_sim-*; do
    candidates+=("${candidate}")
  done
  shopt -u nullglob

  command_path="$(command -v isaac-sim.sh 2>/dev/null || true)"
  [[ -n "${command_path}" ]] && candidates+=("${command_path}")

  for candidate in "${candidates[@]}"; do
    if resolved="$(isaac_candidate_root "${candidate}" 2>/dev/null)"; then
      printf '%s\n' "${resolved}"
      return 0
    fi
  done

  return 1
}

isaac_resolve_python() {
  local root="${1:-}"

  if [[ -n "${ISAAC_PYTHON:-}" && -x "${ISAAC_PYTHON}" ]]; then
    printf '%s\n' "${ISAAC_PYTHON}"
    return 0
  fi
  if [[ -n "${root}" && -x "${root}/python.sh" ]]; then
    printf '%s\n' "${root}/python.sh"
    return 0
  fi

  return 1
}

isaac_resolve_app() {
  local root="${1:-}"

  if [[ -n "${root}" && -x "${root}/isaac-sim.sh" ]]; then
    printf '%s\n' "${root}/isaac-sim.sh"
    return 0
  fi

  return 1
}

isaac_detect_version() {
  local root="${1:-}" python_launcher="${2:-}" version_file value
  local -a version_files=()

  if [[ -n "${ISAAC_SIM_VERSION:-}" ]]; then
    printf '%s\n' "${ISAAC_SIM_VERSION}"
    return 0
  fi

  if [[ -n "${root}" ]]; then
    version_files=(
      "${root}/VERSION"
      "${root}/VERSION.txt"
      "${root}/version.txt"
    )
    for version_file in "${version_files[@]}"; do
      if [[ -r "${version_file}" ]]; then
        value="$(grep -Eo '[0-9]+\.[0-9]+\.[0-9]+' "${version_file}" | head -n 1 || true)"
        if [[ -n "${value}" ]]; then
          printf '%s\n' "${value}"
          return 0
        fi
      fi
    done

    value="$(basename -- "${root}" | grep -Eo '[0-9]+\.[0-9]+\.[0-9]+' | head -n 1 || true)"
    if [[ -n "${value}" ]]; then
      printf '%s\n' "${value}"
      return 0
    fi
  fi

  if [[ -n "${python_launcher}" && -x "${python_launcher}" ]] && command -v timeout >/dev/null 2>&1; then
    value="$(timeout 20 "${python_launcher}" -c \
      'from importlib.metadata import PackageNotFoundError, version
for name in ("isaacsim", "isaacsim-app"):
    try:
        print("__ISAAC_SIM_VERSION__=" + version(name))
        break
    except PackageNotFoundError:
        pass' 2>/dev/null | sed -nE \
      's/^__ISAAC_SIM_VERSION__=([0-9]+\.[0-9]+\.[0-9]+).*$/\1/p' | head -n 1 || true)"
    if [[ -n "${value}" ]]; then
      printf '%s\n' "${value}"
      return 0
    fi
  fi

  return 1
}

isaac_source_ros() {
  local require_workspace="${1:-false}"
  local ros_setup="/opt/ros/jazzy/setup.bash"
  local workspace="${ISAAC_ROS_WS:-${ISAAC_DEFAULT_ROS_WS}}"
  local workspace_setup="${workspace}/install/setup.bash"
  local nounset_was_enabled=0

  [[ -r "${ros_setup}" ]] || isaac_die "ROS 2 Jazzy setup is missing: ${ros_setup}"

  [[ $- == *u* ]] && nounset_was_enabled=1
  set +u
  # shellcheck disable=SC1091
  source "${ros_setup}"
  if [[ -r "${workspace_setup}" ]]; then
    # shellcheck disable=SC1090
    source "${workspace_setup}"
  elif isaac_is_true "${require_workspace}"; then
    (( nounset_was_enabled )) && set -u
    isaac_die "Workspace is not built: ${workspace_setup} (run colcon build --symlink-install in ${workspace})"
  else
    isaac_warn "Workspace overlay is not built yet: ${workspace_setup}"
  fi
  (( nounset_was_enabled )) && set -u

  export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
}

isaac_semantic_site_packages() {
  local semantic_venv="${ISAAC_SEMANTIC_VENV:-${ISAAC_DEFAULT_SEMANTIC_VENV}}"
  local python_version candidate

  [[ -d "${semantic_venv}/lib" ]] || return 1
  python_version="$(/usr/bin/python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
  candidate="${semantic_venv}/lib/python${python_version}/site-packages"
  if [[ -d "${candidate}" ]]; then
    printf '%s\n' "${candidate}"
    return 0
  fi

  find "${semantic_venv}/lib" -mindepth 2 -maxdepth 2 -type d \
    -path '*/python*/site-packages' -print -quit 2>/dev/null
}

isaac_add_semantic_pythonpath() {
  local site_packages

  site_packages="$(isaac_semantic_site_packages || true)"
  [[ -n "${site_packages}" ]] || return 1
  case ":${PYTHONPATH:-}:" in
    *":${site_packages}:"*) ;;
    *) export PYTHONPATH="${site_packages}${PYTHONPATH:+:${PYTHONPATH}}" ;;
  esac
  ISAAC_SEMANTIC_SITE_PACKAGES="${site_packages}"
  export ISAAC_SEMANTIC_SITE_PACKAGES
}

isaac_probe_semantic_runtime() {
  local worker_root="${ISAAC_PORT_ROOT}/dev/turtlebot3/external/semantic-nav-memory"

  PYTHONPATH="${worker_root}${PYTHONPATH:+:${PYTHONPATH}}" /usr/bin/python3 -c '
from importlib import metadata
import cv2
import clip
import numpy
import pycolmap
import pydantic
import yaml
import rclpy
import torch
import torchvision
import transformers
import tokenizers
import accelerate
import timm
import peft
import decord
import lmdb
import PIL
import huggingface_hub
import ultralytics
from cv_bridge import CvBridge
from semantic_nav_memory import cli as semantic_worker_cli

expected = {
    "torch": "2.11.0",
    "torchvision": "0.26.0",
    "ultralytics": "8.4.38",
    "ultralytics-thop": "2.0.18",
    "transformers": "4.57.1",
    "tokenizers": "0.22.0",
    "accelerate": "1.5.2",
    "timm": "1.0.22",
    "peft": "0.12.0",
    "decord": "0.6.0",
    "lmdb": "1.7.5",
    "Pillow": "11.1.0",
    "huggingface-hub": "0.36.0",
    "pycolmap": "4.0.3",
    "opencv-python": "4.8.1.78",
    "numpy": "1.26.4",
    "pydantic": "2.13.2",
    "pyyaml": "6.0.3",
    "clip": "1.0",
    "ftfy": "6.3.1",
    "regex": "2026.7.19",
    "tqdm": "4.70.0",
    "wcwidth": "0.8.2",
}

detected = {name: metadata.version(name) for name in expected}
pins_ok = detected == expected
cuda_ok = torch.cuda.is_available() and torch.cuda.device_count() > 0
ros_imports_ok = rclpy.__name__ == "rclpy" and CvBridge.__name__ == "CvBridge"
worker_import_ok = semantic_worker_cli.__name__ == "semantic_nav_memory.cli"
print(
    " ".join(f"{name}={detected[name]}" for name in expected)
    + f" torch_runtime={torch.__version__} cuda={torch.cuda.is_available()} "
      f"devices={torch.cuda.device_count()} pins={pins_ok} ros_imports={ros_imports_ok} "
      f"worker_import={worker_import_ok}"
)
raise SystemExit(0 if cuda_ok and pins_ok and ros_imports_ok and worker_import_ok else 2)
' 2>/dev/null
}

isaac_verify_port_assets() {
  local model expected actual
  command -v sha256sum >/dev/null 2>&1 || return 1
  while IFS='|' read -r model expected; do
    [[ -r "${model}" ]] || return 1
    actual="$(sha256sum -- "${model}" | awk '{print $1}')"
    [[ "${actual}" == "${expected}" ]] || return 1
  done <<EOF
${ISAAC_REFINER_MODEL}|${ISAAC_REFINER_MODEL_SHA256}
${ISAAC_CLIP_MODEL}|${ISAAC_CLIP_MODEL_SHA256}
${ISAAC_CLIP_WHEEL}|${ISAAC_CLIP_WHEEL_SHA256}
EOF

  isaac_add_semantic_pythonpath || return 1
  /usr/bin/python3 - "${ISAAC_LIVE_MODEL_ID}" "${ISAAC_LIVE_MODEL_REVISION}" <<'PY' >/dev/null 2>&1
import sys
from huggingface_hub import snapshot_download

snapshot_download(
    repo_id=sys.argv[1],
    revision=sys.argv[2],
    local_files_only=True,
)
PY
}

isaac_prepare_runtime_dir() {
  umask 077
  mkdir -p -- "${ISAAC_RUNTIME_DIR}"
}

isaac_pid_file() {
  local kind="${1:?process kind is required}"
  printf '%s/%s.pid\n' "${ISAAC_RUNTIME_DIR}" "${kind}"
}

isaac_proc_start_ticks() {
  local pid="${1:?pid is required}" stat_line stat_tail
  local -a stat_fields=()

  [[ -r "/proc/${pid}/stat" ]] || return 1
  IFS= read -r stat_line < "/proc/${pid}/stat" || return 1
  stat_tail="${stat_line#*) }"
  read -r -a stat_fields <<< "${stat_tail}"
  [[ "${#stat_fields[@]}" -gt 19 ]] || return 1
  printf '%s\n' "${stat_fields[19]}"
}

isaac_proc_cmdline() {
  local pid="${1:?pid is required}"
  [[ -r "/proc/${pid}/cmdline" ]] || return 1
  tr '\0' ' ' < "/proc/${pid}/cmdline"
}

isaac_proc_state() {
  local pid="${1:?pid is required}" stat_line stat_tail
  local -a stat_fields=()

  [[ -r "/proc/${pid}/stat" ]] || return 1
  IFS= read -r stat_line < "/proc/${pid}/stat" || return 1
  stat_tail="${stat_line#*) }"
  read -r -a stat_fields <<< "${stat_tail}"
  [[ "${#stat_fields[@]}" -gt 0 ]] || return 1
  printf '%s\n' "${stat_fields[0]}"
}

isaac_record_field() {
  local record="${1:?record path is required}" key="${2:?field name is required}"
  [[ -r "${record}" ]] || return 1
  awk -F= -v wanted="${key}" '$1 == wanted { sub(/^[^=]*=/, ""); print; exit }' "${record}"
}

isaac_write_pid_record() {
  local kind="${1:?process kind is required}"
  local pid="${2:?pid is required}"
  local marker="${3:?process marker is required}"
  local record temp_record start_ticks pgid

  record="$(isaac_pid_file "${kind}")"
  temp_record="${record}.tmp.$$"
  start_ticks="$(isaac_proc_start_ticks "${pid}")" || return 1
  pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"
  [[ "${pgid}" =~ ^[0-9]+$ ]] || pgid="${pid}"

  {
    printf 'kind=%s\n' "${kind}"
    printf 'pid=%s\n' "${pid}"
    printf 'pgid=%s\n' "${pgid}"
    printf 'start_ticks=%s\n' "${start_ticks}"
    printf 'marker=%s\n' "${marker}"
    printf 'port_root=%s\n' "${ISAAC_PORT_ROOT}"
  } > "${temp_record}"
  mv -f -- "${temp_record}" "${record}"
}

isaac_record_is_live() {
  local record="${1:?record path is required}"
  local pid expected_ticks actual_ticks marker cmdline recorded_root

  [[ -r "${record}" ]] || return 1
  pid="$(isaac_record_field "${record}" pid || true)"
  expected_ticks="$(isaac_record_field "${record}" start_ticks || true)"
  marker="$(isaac_record_field "${record}" marker || true)"
  recorded_root="$(isaac_record_field "${record}" port_root || true)"

  [[ "${pid}" =~ ^[0-9]+$ ]] || return 2
  [[ "${expected_ticks}" =~ ^[0-9]+$ ]] || return 2
  [[ -n "${marker}" && "${recorded_root}" == "${ISAAC_PORT_ROOT}" ]] || return 2
  kill -0 "${pid}" 2>/dev/null || return 1
  [[ "$(isaac_proc_state "${pid}" || true)" != "Z" ]] || return 1
  actual_ticks="$(isaac_proc_start_ticks "${pid}" || true)"
  [[ "${actual_ticks}" == "${expected_ticks}" ]] || return 2
  cmdline="$(isaac_proc_cmdline "${pid}" || true)"
  [[ "${cmdline}" == *"${marker}"* ]] || return 2
  return 0
}

isaac_remove_own_record() {
  local kind="${1:?process kind is required}" pid="${2:?pid is required}"
  local record recorded_pid

  record="$(isaac_pid_file "${kind}")"
  [[ -r "${record}" ]] || return 0
  recorded_pid="$(isaac_record_field "${record}" pid || true)"
  if [[ "${recorded_pid}" == "${pid}" ]]; then
    rm -f -- "${record}"
  fi
}

isaac_term_owned_child() {
  local pid="${1:?pid is required}" pgid

  kill -0 "${pid}" 2>/dev/null || return 0
  pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"
  if [[ "${pgid}" =~ ^[0-9]+$ && "${pgid}" == "${pid}" ]]; then
    /bin/kill -TERM -- "-${pgid}" 2>/dev/null || true
  else
    kill -TERM "${pid}" 2>/dev/null || true
  fi
}

isaac_assert_no_live_record() {
  local kind="${1:?process kind is required}" record status pid
  record="$(isaac_pid_file "${kind}")"

  if [[ ! -e "${record}" ]]; then
    return 0
  fi

  if isaac_record_is_live "${record}"; then
    pid="$(isaac_record_field "${record}" pid)"
    isaac_die "${kind} is already running under this port (PID ${pid})."
  else
    status=$?
    if [[ "${status}" -eq 2 ]]; then
      isaac_die "Refusing to replace untrusted PID metadata: ${record}. Inspect it manually."
    fi
    isaac_warn "Removing stale ${kind} PID metadata: ${record}"
    rm -f -- "${record}"
  fi
}

isaac_wait_for_marker() {
  local pid="${1:?pid is required}" marker="${2:?process marker is required}"
  local attempt cmdline

  for attempt in {1..40}; do
    kill -0 "${pid}" 2>/dev/null || return 1
    cmdline="$(isaac_proc_cmdline "${pid}" || true)"
    [[ "${cmdline}" == *"${marker}"* ]] && return 0
    sleep 0.05
  done

  return 1
}

isaac_count_scene_runners() {
  local marker="${1:-${ISAAC_DEFAULT_SCENE_SCRIPT}}"
  local proc_dir pid pgid cmdline count=0
  declare -A seen_pgids=()

  for proc_dir in /proc/[0-9]*; do
    [[ -r "${proc_dir}/cmdline" ]] || continue
    cmdline="$(tr '\0' ' ' < "${proc_dir}/cmdline" 2>/dev/null || true)"
    [[ "${cmdline}" == *"${marker}"* ]] || continue

    # A native Isaac launch has a python.sh wrapper and a Kit Python child;
    # both command lines contain the scene path but belong to one dedicated
    # process group. Count launch groups so one scene is one instance while
    # independent run_sim.sh invocations still trip the duplicate guard.
    pid="${proc_dir##*/}"
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"
    [[ "${pgid}" =~ ^[0-9]+$ ]] || pgid="${pid}"
    if [[ -z "${seen_pgids[${pgid}]+present}" ]]; then
      seen_pgids["${pgid}"]=1
      ((count += 1))
    fi
  done
  printf '%s\n' "${count}"
}

isaac_external_isaac_pids() {
  local managed_marker="${1:-${ISAAC_DEFAULT_SCENE_SCRIPT}}"
  local proc_dir cmdline current_uid

  current_uid="$(id -u)"
  for proc_dir in /proc/[0-9]*; do
    [[ -r "${proc_dir}/cmdline" ]] || continue
    [[ "$(stat -c %u "${proc_dir}" 2>/dev/null || true)" == "${current_uid}" ]] || continue
    cmdline="$(isaac_proc_cmdline "${proc_dir##*/}" 2>/dev/null || true)"
    [[ -n "${cmdline}" && "${cmdline}" != *"${managed_marker}"* ]] || continue
    case "${cmdline}" in
      *"/isaac-sim.sh"* | *"/kit/kit "* | *isaacsim.exp.*)
        printf '%s\n' "${proc_dir##*/}"
        ;;
    esac
  done
}

isaac_conflicting_gazebo_pids() {
  local proc_dir cmdline current_uid

  current_uid="$(id -u)"
  for proc_dir in /proc/[0-9]*; do
    [[ -r "${proc_dir}/cmdline" ]] || continue
    [[ "$(stat -c %u "${proc_dir}" 2>/dev/null || true)" == "${current_uid}" ]] || continue
    cmdline="$(isaac_proc_cmdline "${proc_dir##*/}" 2>/dev/null || true)"
    case "${cmdline}" in
      *"gz sim"* | *gzserver* | *gzclient* | *full_semantic_nav.launch.py*)
        printf '%s\n' "${proc_dir##*/}"
        ;;
    esac
  done
}
