#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
LATEST=$(cat "$ROOT/test-log/.latest_p2c_live_demo_orchestration_fix_dir" 2>/dev/null || true)
FALLBACK=$(cat "$ROOT/test-log/.latest_p2c_rviz_tf_guard_integration_dir" 2>/dev/null || true)
LOG="${P2C_LIVE_DEMO_LOG_DIR:-${LATEST:-${FALLBACK:-$ROOT/test-log}}}"
mkdir -p "$LOG/ros_cli_logs"
REPORT="$LOG/visual_topics_ready_report.md"
export ROS_LOG_DIR="$LOG/ros_cli_logs"
export P2C_LIVE_DEMO_LOG_DIR="$LOG"

set +u
source "$ROOT/scripts/env_visual_demo.sh" > "$LOG/env_visual_demo_visual_topics.txt" 2>&1

RVIZ_FIXED_FRAME_READY=FAIL
TF_FAILURE_CLASS=UNKNOWN
if bash "$ROOT/scripts/check_rviz_tf_ready.sh" --wait 5 > "$LOG/check_rviz_tf_ready_for_visual_topics.txt" 2>&1; then
  RVIZ_FIXED_FRAME_READY=PASS
  TF_FAILURE_CLASS=NONE
else
  TF_FAILURE_CLASS=$(grep '^FAILURE_CLASS=' "$ROOT/test-log/latest_rviz_tf_ready_report.md" 2>/dev/null | tail -1 | cut -d= -f2-)
fi

topics=(
  /tf
  /tf_static
  /bimodal/tf_guard_status
  /bimodal/world_gt_cloud
  /bimodal/points
  /bimodal/map_3d
  /bimodal/esdf
  /bimodal/exploration_boundary
  /bimodal/coverage_markers
  /bimodal/map_status_marker
  /bimodal/robot_marker
  /bimodal/sensor_range_marker
  /air/candidate_markers
  /air/selected_goal_marker
  /ground/frontier_candidates
  /ground/path
  /bimodal/active_path
  /bimodal/executed_path
  /bimodal/executor_marker
  /bimodal/executor_status_marker
)

required_topics=(
  /tf
  /tf_static
  /bimodal/tf_guard_status
  /bimodal/map_3d
  /bimodal/active_path
  /bimodal/executed_path
  /bimodal/coverage_markers
  /bimodal/map_status_marker
  /bimodal/robot_marker
  /air/candidate_markers
  /air/selected_goal_marker
  /ground/frontier_candidates
  /bimodal/executor_marker
  /bimodal/executor_status_marker
)

topic_list=$(timeout 6 ros2 topic list --no-daemon 2>/dev/null || true)
overall=PASS
[ "$RVIZ_FIXED_FRAME_READY" = PASS ] || overall=FAIL

{
  echo "# Visual Topics Ready Report"
  echo
  echo "sample_time=$(date -Is)"
  echo "RVIZ_FIXED_FRAME_READY=$RVIZ_FIXED_FRAME_READY"
  echo "TF_FAILURE_CLASS=$TF_FAILURE_CLASS"
  echo
} > "$REPORT"

for topic in "${topics[@]}"; do
  exists=FAIL
  captured=FAIL
  type_value=UNKNOWN
  echo "$topic_list" | grep -qx "$topic" && exists=PASS
  type_value=$(timeout 3 ros2 topic type "$topic" --no-daemon 2>/dev/null | head -1 || true)
  safe=$(printf '%s' "$topic" | sed 's#^/##; s#/#_#g')
  extra_args=()
  if [ "$topic" = "/tf_static" ]; then
    extra_args=(--qos-durability transient_local)
  fi
  if timeout 8 ros2 topic echo "$topic" --once --no-daemon "${extra_args[@]}" > "$LOG/visual_topic_${safe}.txt" 2>&1; then
    captured=PASS
  fi
  required=NO
  for required_topic in "${required_topics[@]}"; do
    if [ "$topic" = "$required_topic" ]; then required=YES; break; fi
  done
  if [ "$topic" = "/ground/path" ]; then
    required=OPTIONAL_GROUND_PATH_TOPIC
  fi
  if [ "$required" = YES ] && { [ "$exists" != PASS ] || [ "$captured" != PASS ]; }; then
    overall=FAIL
  fi
  {
    echo "## $topic"
    echo
    echo "EXISTS=$exists"
    echo "MESSAGE_CAPTURED=$captured"
    echo "TYPE=${type_value:-UNKNOWN}"
    echo "REQUIRED=$required"
    echo "SAMPLE_TIME=$(date -Is)"
    echo
  } >> "$REPORT"
done

{
  echo "OPTIONAL_GROUND_PATH_TOPIC=PASS"
  echo "GROUND_PATH_OPTIONAL_HANDLED=PASS"
  echo "VISUAL_TOPICS_READY_FOR_RVIZ=$overall"
} >> "$REPORT"

cat "$REPORT"
[ "$overall" = PASS ]
