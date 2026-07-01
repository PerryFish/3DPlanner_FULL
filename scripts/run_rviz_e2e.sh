#!/usr/bin/env bash
set -u
ROOT=/home/nuaa/ZHY/3DPlanner_FULL
set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$ROOT/Map/ros2_ws/install/setup.bash" 2>/dev/null || true
set -u
if [ -z "${DISPLAY:-}" ]; then
  echo "DISPLAY_ENVIRONMENT_LIMITATION=YES"
fi
rviz2 -d "$ROOT/Map/rviz/e2e_closed_loop.rviz"
