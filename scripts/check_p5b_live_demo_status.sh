#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
TS=$(date +%Y%m%d_%H%M%S)
LOG="${P5B_STATUS_LOG_DIR:-$ROOT/test-log/${TS}_p5b_live_status_check}"
mkdir -p "$LOG/ros_cli_logs"
export ROS_LOG_DIR="$LOG/ros_cli_logs"

set +u
source "$ROOT/scripts/env_visual_demo.sh" > "$LOG/env_visual_demo.txt" 2>&1
set -u

topic_exists() {
  echo "$TOPICS" | grep -qx "$1"
}

node_matches() {
  echo "$NODES" | grep -E "$1" >/dev/null 2>&1
}

capture_once() {
  local topic="$1"
  local out="$2"
  shift 2
  timeout 6 ros2 topic echo "$topic" --once --no-daemon "$@" > "$out" 2>&1
}

check_topic_capture() {
  local topic="$1"
  local name="$2"
  local extra=()
  if [ "$topic" = "/tf_static" ]; then
    extra=(--qos-durability transient_local)
  fi
  local exists=NO
  local captured=FAIL
  if topic_exists "$topic"; then exists=YES; fi
  if [ "$exists" = YES ] && capture_once "$topic" "$LOG/${name}.txt" "${extra[@]}"; then captured=PASS; fi
  echo "${name}_exists=$exists"
  echo "${name}_captured=$captured"
  [ "$exists" = YES ] && [ "$captured" = PASS ]
}

NODES=$(timeout 6 ros2 node list --no-daemon 2>/dev/null || true)
TOPICS=$(timeout 6 ros2 topic list --no-daemon 2>/dev/null || true)
printf '%s\n' "$NODES" > "$LOG/node_list.txt"
printf '%s\n' "$TOPICS" > "$LOG/topic_list.txt"

live_nodes=0
for pattern in synthetic_external_pointcloud_publisher_node real_sensor_pointcloud_bridge_node octomap_pointcloud_backend_node \
  visual_tf_guard_node demo_explainability_overlay_node bimodal_mode_mux_node fake_path_executor_node \
  air_exploration_stub_node ground_3d_frontier_node; do
  if node_matches "$pattern"; then live_nodes=$((live_nodes + 1)); fi
done

status=PASS
reason=NONE
if [ "$live_nodes" -lt 5 ]; then
  status=FAIL
  reason=LIVE_DEMO_NODES_NOT_RUNNING
fi

tf_topic_exists=NO
tf_static_topic_exists=NO
topic_exists /tf && tf_topic_exists=YES
topic_exists /tf_static && tf_static_topic_exists=YES

tf_capture=FAIL
tf_static_capture=FAIL
tf_guard_status=FAIL
if [ "$status" = PASS ]; then
  if capture_once /tf "$LOG/tf_once.txt"; then tf_capture=PASS; else status=FAIL; reason=TF_TOPIC_CAPTURE_FAIL; fi
  if capture_once /tf_static "$LOG/tf_static_once.txt" --qos-durability transient_local; then tf_static_capture=PASS; else status=FAIL; reason=TF_STATIC_CAPTURE_FAIL; fi
  if capture_once /bimodal/tf_guard_status "$LOG/tf_guard_status_once.txt" --field data; then tf_guard_status=PASS; else status=FAIL; reason=TF_GUARD_STATUS_FAIL; fi
fi

tf_echo=FAIL
if [ "$status" = PASS ]; then
  timeout 7 ros2 run tf2_ros tf2_echo map base_link > "$LOG/tf2_echo_map_base_link.txt" 2>&1 || true
  if grep -q "Translation:" "$LOG/tf2_echo_map_base_link.txt"; then
    tf_echo=PASS
  else
    status=FAIL
    reason=TF_ECHO_MAP_BASE_LINK_FAIL
  fi
fi

points_rate=0.0
if topic_exists /bimodal/points; then
  timeout 8 ros2 topic hz /bimodal/points --window 8 > "$LOG/points_rate.txt" 2>&1 || true
  points_rate=$(awk '/average rate:/ {print $3; exit}' "$LOG/points_rate.txt" 2>/dev/null || echo 0.0)
fi

required_topics=(
  /bimodal/points
  /bimodal/map_3d
  /bimodal/active_mode
  /bimodal/active_path
  /bimodal/executed_path
  /air/candidate_markers
  /ground/frontier_candidates
  /bimodal/robot_marker
  /bimodal/demo_status_text
  /bimodal/demo_legend_markers
  /bimodal/selected_goal_marker
)

missing=()
for topic in "${required_topics[@]}"; do
  if ! topic_exists "$topic"; then
    missing+=("$topic")
    status=FAIL
    [ "$reason" = NONE ] && reason=REQUIRED_TOPIC_MISSING
  fi
done

forbidden=$(echo "$TOPICS" | grep -E "^/cmd_vel$|^/mavros/|^/fmu/|^/actuator/|^/offboard_control_mode$|^/trajectory_setpoint$" || true)
forbidden_count=0
if [ -n "$forbidden" ]; then
  forbidden_count=$(printf '%s\n' "$forbidden" | sed '/^$/d' | wc -l)
  status=FAIL
  reason=FORBIDDEN_CONTROL_TOPIC_DETECTED
fi

{
  echo "P5B_LIVE_DEMO_STATUS=$status"
  echo "failure_reason=$reason"
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-}"
  echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-}"
  echo "live_demo_node_match_count=$live_nodes"
  echo "tf_topic_exists=$tf_topic_exists"
  echo "tf_static_topic_exists=$tf_static_topic_exists"
  echo "tf_capture=$tf_capture"
  echo "tf_static_capture=$tf_static_capture"
  echo "tf_guard_status=$tf_guard_status"
  echo "tf_echo_map_base_link=$tf_echo"
  echo "bimodal_points_rate_avg_hz=${points_rate:-0.0}"
  echo "required_topic_missing=${missing[*]:-NONE}"
  echo "forbidden_topic_detected_count=$forbidden_count"
  echo "forbidden_topic_list=${forbidden:-NONE}"
  echo "log_dir=$LOG"
  if [ "$status" = FAIL ] && [ "$reason" = LIVE_DEMO_NODES_NOT_RUNNING ]; then
    echo "Live demo nodes are not running. Start:"
    echo "cd $ROOT && ./scripts/run_p5b_explainable_bimodal_live_demo.sh"
  fi
  echo
  echo "# Node list"
  printf '%s\n' "$NODES"
  echo
  echo "# Topic list"
  printf '%s\n' "$TOPICS"
} | tee "$LOG/p5b_live_demo_status.txt"

[ "$status" = PASS ]
