#!/usr/bin/env bash
# Development environment for TurtleBot3 ROS2 workspace.
# Usage: source enter_tb3_dev.sh  (must be sourced, not executed)

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "Error: source this script, do not execute it."
  echo "Usage: source enter_tb3_dev.sh"
  exit 1
fi

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"

# 1. Source ROS 2
if [ ! -f "${ROS_SETUP}" ]; then
  echo "Error: ROS setup script not found: ${ROS_SETUP}"
  echo "Install ROS 2 ${ROS_DISTRO_NAME} first."
  return 1 2>/dev/null || exit 1
fi

source "${ROS_SETUP}"

# 2. Source workspace install
if [ -f "${WS_ROOT}/install/setup.bash" ]; then
  source "${WS_ROOT}/install/setup.bash"
else
  echo "Warning: ${WS_ROOT}/install/setup.bash not found. Run ./build.sh first."
fi

# 3. Print environment info
echo "=== TurtleBot3 ROS2 dev environment ==="
echo "ROS_DISTRO:        ${ROS_DISTRO:-<not set>}"
echo "ROS setup:         ${ROS_SETUP}"
echo "AMENT_PREFIX_PATH: ${AMENT_PREFIX_PATH:-<not set>}"
echo ""

# 4. Confirm workspace is active
echo "Workspace active: ${WS_ROOT}"
echo "Ready for development. Run nodes from this terminal."
echo ""
