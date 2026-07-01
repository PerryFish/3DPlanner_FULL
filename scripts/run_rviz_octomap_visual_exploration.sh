#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
RVIZ_CONFIG="$ROOT/Map/rviz/octomap_visual_exploration_demo.rviz"
LOG_FILE="$ROOT/test-log/latest_rviz_octomap_visual_exploration.log"

echo "current_stage=P1B-OCTOMAP-POINTCLOUD-BACKEND"
echo "purpose=RViz GUI display access only; no real control topics are started"

set +u
source "$ROOT/scripts/env_visual_demo.sh"

if [ -z "${DISPLAY:-}" ]; then
  echo "ERROR: DISPLAY is empty. Please run this from Ubuntu graphical desktop terminal."
  exit 2
fi

if ! bash "$ROOT/scripts/check_rviz_tf_ready.sh" --wait 10 > "$ROOT/test-log/latest_rviz_octomap_tf_ready_before_rviz.log" 2>&1; then
  echo "RVIZ_NOT_STARTED_BECAUSE_TF_NOT_READY"
  cat "$ROOT/test-log/latest_rviz_tf_ready_report.md" 2>/dev/null || true
  exit 5
fi

if ! command -v rviz2 >/dev/null 2>&1; then
  echo "ERROR: rviz2 not found after sourcing ROS environment."
  exit 4
fi

if command -v xdpyinfo >/dev/null 2>&1 && ! xdpyinfo >/dev/null 2>&1; then
  echo "ERROR: DISPLAY is set but X server is not accessible."
  exit 3
fi

echo "RVIZ_LAUNCHED_AFTER_TF_READY=YES"
echo "Starting RViz with config: $RVIZ_CONFIG"
mkdir -p "$(dirname "$LOG_FILE")"
rviz2 -d "$RVIZ_CONFIG" 2>&1 | tee "$LOG_FILE"
