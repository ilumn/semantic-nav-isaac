#!/usr/bin/env bash
# Verify or explicitly populate the project-local semantic runtime venv.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
isaac_load_env

mode=check
selected_mode=""
requirements_file="${ISAAC_RUNTIME_REQUIREMENTS:-${SCRIPT_DIR}/requirements-runtime.txt}"
semantic_venv="${ISAAC_SEMANTIC_VENV:-${ISAAC_DEFAULT_SEMANTIC_VENV}}"

usage() {
  cat <<EOF
Usage: ./bootstrap_runtime.sh [option]

With no option, only verifies the pinned imports and CUDA runtime; it does not
create a venv, install packages, or access the network.

Options:
  --check             Verify only (default).
  --install-offline   Install from vendored wheels plus the existing uv cache; no network.
  --allow-download    Explicitly allow uv to use configured package indexes.
  -h, --help          Show this help.

Runtime venv: ${semantic_venv}
Requirements: ${requirements_file}
EOF
}

while (($#)); do
  case "$1" in
    --check)
      [[ -z "${selected_mode}" || "${selected_mode}" == check ]] || isaac_die "Choose only one bootstrap mode"
      mode=check
      selected_mode=check
      ;;
    --install-offline)
      [[ -z "${selected_mode}" || "${selected_mode}" == offline ]] || isaac_die "Choose only one bootstrap mode"
      mode=offline
      selected_mode=offline
      ;;
    --allow-download)
      [[ -z "${selected_mode}" || "${selected_mode}" == online ]] || isaac_die "Choose only one bootstrap mode"
      mode=online
      selected_mode=online
      ;;
    -h | --help)
      usage
      exit 0
      ;;
    *) isaac_die "Unknown option: $1" ;;
  esac
  shift
done

[[ -r "${requirements_file}" ]] || isaac_die \
  "Pinned runtime requirements are missing: ${requirements_file}"

if [[ "${mode}" != check ]]; then
  command -v uv >/dev/null 2>&1 || isaac_die "uv is required for explicit runtime installation"
  if [[ ! -x "${semantic_venv}/bin/python" ]]; then
    isaac_info "Creating Python 3.12 venv: ${semantic_venv}"
    uv venv --python /usr/bin/python3 "${semantic_venv}"
  fi
  venv_python_version="$("${semantic_venv}/bin/python" -c \
    'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
  [[ "${venv_python_version}" == "3.12" ]] || isaac_die \
    "Runtime venv must use Python 3.12; found ${venv_python_version}"

  declare -a install_command=(
    uv pip install
    --python "${semantic_venv}/bin/python"
    --requirements "${requirements_file}"
    --find-links "${SCRIPT_DIR}/vendor"
  )
  if [[ "${mode}" == offline ]]; then
    install_command+=(--offline)
    isaac_info "Installing from vendored wheels plus the existing uv cache; network access is disabled"
  else
    isaac_warn "Network-enabled installation was explicitly requested with --allow-download"
  fi
  "${install_command[@]}"
fi

isaac_source_ros false
if ! isaac_add_semantic_pythonpath; then
  isaac_die "Runtime venv site-packages are missing: ${semantic_venv}"
fi
runtime_report="$(isaac_probe_semantic_runtime || true)"
[[ -n "${runtime_report}" && "${runtime_report}" == *"cuda=True"* && "${runtime_report}" == *"pins=True"* && "${runtime_report}" == *"ros_imports=True"* && "${runtime_report}" == *"worker_import=True"* ]] || isaac_die \
  "Pinned runtime import/CUDA verification failed for ${ISAAC_SEMANTIC_SITE_PACKAGES}"
isaac_pass "Pinned runtime verified: ${runtime_report}"
