#!/usr/bin/env bash
set -u
export ROS_LOG_DIR=${ROS_LOG_DIR:-/home/nuaa/ZHY/3DPlanner_FULL/Air/test-log/ros_logs}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-/home/nuaa/ZHY/3DPlanner_FULL/Map/config/fastdds_shm_only.xml}
export FASTDDS_DEFAULT_PROFILES_FILE=${FASTDDS_DEFAULT_PROFILES_FILE:-/home/nuaa/ZHY/3DPlanner_FULL/Map/config/fastdds_shm_only.xml}
mkdir -p "$ROS_LOG_DIR"
set +u
source /opt/ros/humble/setup.bash
source /home/nuaa/ZHY/3DPlanner_FULL/Air/ros2_ws/install/setup.bash
set -u
ros2 launch bimodal_air_bringup air_baseline.launch.py
