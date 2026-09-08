#!/usr/bin/env bash
# Build tb3_frontier_exploration using system Python (has ROS build deps),
# so that colcon/ament do not pick up Conda or other alternate interpreters.
# Run from: ros2_ws (this directory)
#   ./build_tb3_frontier.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"

cd "${SCRIPT_DIR}"

# Force system Python first (must be before any other setup)
export PATH="/usr/bin:${PATH}"
if ! command -v python3 >/dev/null 2>&1; then
  echo "Error: python3 not found in /usr/bin. Install ROS 2 ${ROS_DISTRO_NAME} or set PATH."
  exit 1
fi
echo "Using python3: $(which python3)"
echo "Using ROS distro: ${ROS_DISTRO_NAME}"

# Remove cached build so CMake does not reuse alternate python paths
rm -rf build/tb3_frontier_exploration install/tb3_frontier_exploration

if [ ! -f "${ROS_SETUP}" ]; then
  echo "Error: ROS setup script not found: ${ROS_SETUP}"
  echo "Install ROS 2 ${ROS_DISTRO_NAME} first."
  exit 1
fi

source "${ROS_SETUP}"
# Keep system Python first after ROS setup (ROS setup can change PATH)
export PATH="/usr/bin:${PATH}"

colcon build --packages-select tb3_frontier_exploration "$@"

echo ""
echo "Done. Then run:  source install/setup.bash"
