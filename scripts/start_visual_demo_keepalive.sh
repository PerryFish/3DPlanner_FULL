#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
MODE_SWITCH_PERIOD=60
LOG=""

while [ "$#" -gt 0 ]; do
  case "$1" in
    --mode-switch-period) MODE_SWITCH_PERIOD="$2"; shift 2 ;;
    --duration) shift 2 ;;
    --log-dir) LOG="$2"; shift 2 ;;
    *) echo "unknown_arg=$1" >&2; exit 2 ;;
  esac
done

TS=$(date +%Y%m%d_%H%M%S)
LOG="${LOG:-$ROOT/test-log/${TS}_p2c_live_visual_demo}"
mkdir -p "$LOG" "$LOG/ros_logs"
printf '%s\n' "$LOG" > "$ROOT/test-log/.latest_p2c_live_demo_orchestration_fix_dir"
printf '%s\n' "$LOG" > "$ROOT/test-log/.latest_p2c_rviz_and_exploration_quality_dir"
printf '%s\n' "$LOG" > "$ROOT/test-log/.latest_p2c_rviz_tf_guard_integration_dir"
export ROS_LOG_DIR="$LOG/ros_logs"
export P2C_LIVE_DEMO_LOG_DIR="$LOG"

set +u
source "$ROOT/scripts/env_visual_demo.sh" > "$LOG/env_visual_demo_keepalive.txt" 2>&1

cleanup_old_visual_demo() {
  patterns=(
    "ros2 launch bimodal_map_bringup visual_exploration_demo_map_side.launch.py"
    "ros2 launch bimodal_air_bringup air_baseline.launch.py"
    "ros2 launch bimodal_ground_bringup ground_baseline.launch.py"
    "fake_path_executor_node"
    "visual_tf_guard_node"
    "virtual_sensor_node"
    "bimodal_mode_mux_node"
    "fallback_3d_map_adapter_node"
  )
  for pattern in "${patterns[@]}"; do
    while read -r pid cmdline; do
      [ -n "$pid" ] || continue
      case "$cmdline" in
        *start_visual_demo_keepalive.sh*|*run_live_rviz_demo_all_in_one.sh*) continue ;;
      esac
      kill "$pid" 2>/dev/null || true
    done < <(pgrep -af "$pattern" 2>/dev/null || true)
  done
  sleep 2
}

pids=()
cleanup() {
  for p in "${pids[@]}"; do kill "$p" 2>/dev/null || true; done
  sleep 2
  for p in "${pids[@]}"; do kill -KILL "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
handle_stop() {
  cleanup
  exit 0
}
trap cleanup EXIT
trap handle_stop INT TERM

cleanup_old_visual_demo

ros2 launch bimodal_map_bringup visual_exploration_demo_map_side.launch.py \
  e2e_log_dir:="$LOG" mode_switch_period_sec:="$MODE_SWITCH_PERIOD" \
  use_external_odom_for_virtual_sensor:=true virtual_sensor_publish_odom:=false \
  publish_world_gt_cloud:=true fallback_accumulate_map:=true \
  > "$LOG/map_keepalive.log" 2>&1 &
pids+=($!)
sleep 4
ros2 launch bimodal_air_bringup air_baseline.launch.py > "$LOG/air_keepalive.log" 2>&1 &
pids+=($!)
ros2 launch bimodal_ground_bringup ground_baseline.launch.py > "$LOG/ground_keepalive.log" 2>&1 &
pids+=($!)

if bash "$ROOT/scripts/check_rviz_tf_ready.sh" --wait 20 > "$LOG/initial_tf_ready_check.log" 2>&1; then
  echo "current_stage=P2C-LIVE-DEMO-ORCHESTRATION-FIX"
  echo "TF_GUARD_ENABLED=YES"
  echo "VISUAL_DEMO_KEEPALIVE_RUNNING=YES"
  echo "TF_READY=PASS"
  echo "log_dir=$LOG"
  echo "RVIZ_START_COMMAND=cd $ROOT && bash scripts/run_rviz_visual_exploration.sh"
  echo "TOPIC_WATCH_COMMAND=cd $ROOT && bash scripts/visual_topic_watch.sh"
  echo "TF_CHECK_COMMAND=cd $ROOT && bash scripts/check_rviz_tf_ready.sh"
else
  echo "current_stage=P2C-LIVE-DEMO-ORCHESTRATION-FIX"
  echo "VISUAL_DEMO_KEEPALIVE_RUNNING=NO"
  echo "TF_READY=FAIL"
  cat "$LOG/initial_tf_ready_check.log" || true
  for f in "$LOG/map_keepalive.log" "$LOG/air_keepalive.log" "$LOG/ground_keepalive.log"; do
    echo "## tail $f"
    tail -120 "$f" 2>/dev/null || true
  done
  exit 1
fi

while true; do
  sleep 10
  TOPICS=$(timeout 4 ros2 topic list --no-daemon 2>/dev/null || true)
  for topic in /bimodal/tf_guard_status /tf /tf_static; do
    if ! echo "$TOPICS" | grep -qx "$topic"; then
      echo "WARNING_TF_TOPIC_DROPPED=$topic sample_time=$(date -Is)"
    fi
  done
done
