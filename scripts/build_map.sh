#!/usr/bin/env bash
set -u
ROOT=/home/nuaa/ZHY/3DPlanner_FULL
WS="$ROOT/Map/ros2_ws"
set +u; source /opt/ros/humble/setup.bash 2>/dev/null || true; set -u
cd "$WS" || exit 1
colcon build --symlink-install
