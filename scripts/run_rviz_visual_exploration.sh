#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
RVIZ_CONFIG="$ROOT/Map/rviz/visual_exploration_demo.rviz"
LOG_FILE="$ROOT/test-log/latest_rviz_visual_exploration.log"

echo "current_stage=P2C-LIVE-DEMO-ORCHESTRATION-FIX"
echo "purpose=RViz GUI display access only; no real control topics are started"
echo "USER=$(whoami)"

set +u
source "$ROOT/scripts/env_visual_demo.sh"

if [ -z "${DISPLAY:-}" ]; then
  echo "ERROR: DISPLAY is empty. Please run this from Ubuntu graphical desktop terminal."
  exit 2
fi

if ! bash "$ROOT/scripts/check_rviz_tf_ready.sh" --wait 10 > "$ROOT/test-log/latest_rviz_tf_ready_before_rviz.log" 2>&1; then
  FAILURE_CLASS=$(grep '^FAILURE_CLASS=' "$ROOT/test-log/latest_rviz_tf_ready_report.md" 2>/dev/null | tail -1 | cut -d= -f2-)
  echo "RVIZ_NOT_STARTED_BECAUSE_TF_NOT_READY"
  echo "FAILURE_CLASS=${FAILURE_CLASS:-UNKNOWN}"
  echo "Please run:"
  echo "cd $ROOT && bash scripts/start_visual_demo_keepalive.sh --mode-switch-period 60"
  cat "$ROOT/test-log/latest_rviz_tf_ready_report.md" 2>/dev/null || true
  exit 5
fi

if ! command -v rviz2 >/dev/null 2>&1; then
  echo "ERROR: rviz2 not found after sourcing ROS environment."
  exit 4
fi

if command -v xdpyinfo >/dev/null 2>&1; then
  if ! xdpyinfo >/dev/null 2>&1; then
    echo "ERROR: DISPLAY is set but X server is not accessible."
    echo "Try: xhost +SI:localuser:$(whoami)"
    exit 3
  fi
fi

echo "RVIZ_LAUNCHED_AFTER_TF_READY=YES"
echo "Starting RViz with config: $RVIZ_CONFIG"
mkdir -p "$(dirname "$LOG_FILE")"
rviz2 -d "$RVIZ_CONFIG" 2>&1 | tee "$LOG_FILE"
