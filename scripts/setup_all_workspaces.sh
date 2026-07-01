#!/usr/bin/env bash
set -u
ROOT=/home/nuaa/ZHY/3DPlanner_FULL
TS=$(date +%Y%m%d_%H%M%S)
LOG="$ROOT/test-log/$TS"
mkdir -p "$LOG"
UBUNTU=$(lsb_release -rs 2>/dev/null || echo unknown)
ROS=${ROS_DISTRO:-}
PY=$(python3 --version 2>&1 || true)
COLCON=$(command -v colcon || true)
ARCH=$(uname -m)
JETSON="not_jetson"
test -f /etc/nv_tegra_release && JETSON="$(cat /etc/nv_tegra_release)"
{
  echo "# System Environment"
  echo "- Ubuntu version: $UBUNTU"
  echo "- ROS distro: ${ROS:-unset}"
  echo "- Python version: $PY"
  echo "- colcon: ${COLCON:-missing}"
  echo "- RMW implementation: ${RMW_IMPLEMENTATION:-default}"
  echo "- GPU / Jetson: $JETSON"
  echo "- Architecture: $ARCH"
} > "$LOG/system_env.md"
{
  echo "# Build Report"
  for item in Map Air Ground; do
    ws="$ROOT/$item/ros2_ws"
    echo "## $item"
    if [ -z "$COLCON" ]; then
      echo "$item build FAIL: colcon missing"
      continue
    fi
    (set +u; source /opt/ros/humble/setup.bash 2>/dev/null || true; set -u; cd "$ws" && colcon build --symlink-install) > "$LOG/${item,,}_build.log" 2>&1
    rc=$?
    if [ $rc -eq 0 ]; then echo "$item build PASS"; else echo "$item build FAIL rc=$rc"; tail -80 "$LOG/${item,,}_build.log"; fi
  done
} > "$LOG/build_report.md"
echo "$LOG"
