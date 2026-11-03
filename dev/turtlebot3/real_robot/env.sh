# semnav runtime environment for this robot (bash). Every semnav systemd unit and the
# `semnav` command source this file; source it in a shell to use ros2 tools:
#   source /mnt/data/semnav/env.sh
#
# All ROS traffic uses CycloneDDS on loopback only (see cyclonedds.xml). Shells that do
# not source this file (plain Fast DDS) will not see the robot's topics.

export SEMNAV_HOME=/mnt/data/semnav
export SEMNAV_REPO=/mnt/data/semantic-nav-isaac
export SEMNAV_WS=$SEMNAV_REPO/dev/turtlebot3/ros2_ws

# ROS setup scripts reference unset variables; do not let a caller's `set -u` break them.
case $- in *u*) _semnav_nounset=1; set +u ;; *) _semnav_nounset=0 ;; esac
source /opt/ros/humble/setup.bash
source "$SEMNAV_WS/install/setup.bash"
[ "$_semnav_nounset" = 1 ] && set -u
unset _semnav_nounset

export ROS_DOMAIN_ID=99
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$SEMNAV_HOME/cyclonedds.xml
unset FASTRTPS_DEFAULT_PROFILES_FILE ROS_LOCALHOST_ONLY

export TURTLEBOT3_MODEL=waffle_pi
export LDS_MODEL=LDS-02

# Python deps layered over the JetPack CUDA torch (transformers, ultralytics, ...).
case ":${PYTHONPATH:-}:" in
  *":$SEMNAV_REPO/.venv/lib/python3.10/site-packages:"*) ;;
  *) export PYTHONPATH="$SEMNAV_REPO/.venv/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}" ;;
esac
export HF_HOME=/mnt/data/hf-cache
export HF_HUB_OFFLINE=1
export YOLO_CONFIG_DIR=/mnt/data/models/ultralytics_config

# Plain, unbuffered output so journald gets complete, uncoloured lines.
export PYTHONUNBUFFERED=1
export RCUTILS_COLORIZED_OUTPUT=0
