#!/usr/bin/env bash
# Shared live visual demo environment. Do not enable set -u in this file:
# ROS and colcon setup scripts are not nounset-safe.

ROOT=/home/nuaa/ZHY/3DPlanner_FULL

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
unset ROS_LOCALHOST_ONLY

export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-$ROOT/Map/config/fastdds_shm_only.xml}
export FASTDDS_DEFAULT_PROFILES_FILE=${FASTDDS_DEFAULT_PROFILES_FILE:-$ROOT/Map/config/fastdds_shm_only.xml}

source /opt/ros/humble/setup.bash
source "$ROOT/Map/ros2_ws/install/setup.bash"
source "$ROOT/Air/ros2_ws/install/setup.bash"
source "$ROOT/Ground/ros2_ws/install/setup.bash"

export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1
unset WAYLAND_DISPLAY

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-UNSET}"
echo "DISPLAY=${DISPLAY:-}"
echo "XAUTHORITY=${XAUTHORITY:-}"
