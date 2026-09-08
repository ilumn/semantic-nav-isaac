#!/usr/bin/env bash
# Build the shared ROS 2 workspace with Jazzy's Ubuntu system Python.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
isaac_load_env

clean_cache=false
declare -a colcon_args=()

usage() {
  cat <<EOF
Usage: ./build_ros.sh [--clean-cache] [-- colcon-arguments...]

Builds ${ISAAC_ROS_WS:-${ISAAC_DEFAULT_ROS_WS}} with --symlink-install while
forcing /usr/bin/python3 for CMake and Python package generation.

Options:
  --clean-cache  Add colcon's --cmake-clean-cache for a one-time reconfigure.
  -h, --help     Show this help.

Put optional colcon selection flags after --, for example:
  ./build_ros.sh -- --packages-select isaac_semantic_nav
EOF
}

while (($#)); do
  case "$1" in
    --clean-cache)
      clean_cache=true
      ;;
    --)
      shift
      colcon_args+=("$@")
      break
      ;;
    -h | --help)
      usage
      exit 0
      ;;
    *)
      isaac_die "Unknown option: $1 (put colcon arguments after --)"
      ;;
  esac
  shift
done

ros_workspace="${ISAAC_ROS_WS:-${ISAAC_DEFAULT_ROS_WS}}"
[[ -d "${ros_workspace}/src" ]] || isaac_die \
  "ROS workspace source tree is missing: ${ros_workspace}/src"
[[ -x /usr/bin/python3 ]] || isaac_die "Required system interpreter is missing: /usr/bin/python3"
[[ -r /opt/ros/jazzy/setup.bash ]] || isaac_die \
  "ROS 2 Jazzy setup is missing: /opt/ros/jazzy/setup.bash"

# Keep user virtual environments and ~/.local/bin from influencing colcon or
# CMake. These changes are confined to this build script's process.
export PATH="/usr/bin:/bin:${PATH}"
export PYTHONNOUSERSITE=1
unset PYTHONHOME PYTHONPATH VIRTUAL_ENV
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
hash -r

nounset_was_enabled=0
[[ $- == *u* ]] && nounset_was_enabled=1
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
(( nounset_was_enabled )) && set -u

[[ "$(command -v python3)" == "/usr/bin/python3" ]] || isaac_die \
  "System Python isolation failed; python3 resolves to $(command -v python3)"
command -v colcon >/dev/null 2>&1 || isaac_die "colcon is not installed on the ROS 2 host"

declare -a build_command=(colcon build --symlink-install)
[[ "${clean_cache}" == true ]] && build_command+=(--cmake-clean-cache)
build_command+=("${colcon_args[@]}")
build_command+=(--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3)

isaac_info "Workspace: ${ros_workspace}"
isaac_info "Python: $(/usr/bin/python3 --version 2>&1) at /usr/bin/python3"
isaac_info "Command: ${build_command[*]}"

cd -- "${ros_workspace}"
"${build_command[@]}"
