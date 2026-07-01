#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
WAIT_SEC=5

while [ "$#" -gt 0 ]; do
  case "$1" in
    --wait) WAIT_SEC="$2"; shift 2 ;;
    *) echo "unknown_arg=$1" >&2; exit 2 ;;
  esac
done

LATEST=$(cat "$ROOT/test-log/.latest_p2c_live_demo_orchestration_fix_dir" 2>/dev/null || true)
FALLBACK=$(cat "$ROOT/test-log/.latest_p2c_rviz_tf_guard_integration_dir" 2>/dev/null || true)
LOG="${P2C_LIVE_DEMO_LOG_DIR:-${LATEST:-${FALLBACK:-$ROOT/test-log}}}"
mkdir -p "$LOG/ros_cli_logs"
REPORT="$ROOT/test-log/latest_rviz_tf_ready_report.md"
LOCAL_REPORT="$LOG/tf_ready_report.md"
export ROS_LOG_DIR="$LOG/ros_cli_logs"

set +u
source "$ROOT/scripts/env_visual_demo.sh" > "$LOG/env_visual_demo_last_check.txt" 2>&1

SUGGESTED_COMMAND="cd $ROOT && bash scripts/start_visual_demo_keepalive.sh --mode-switch-period 60"
DEMO_RUNNING=FAIL
TF_GUARD_RUNNING=FAIL
TF_TOPIC_EXISTS=FAIL
TF_STATIC_TOPIC_EXISTS=FAIL
TF_GUARD_STATUS_TOPIC_EXISTS=FAIL
TF_MESSAGE_CAPTURED=FAIL
TF_STATIC_MESSAGE_CAPTURED=FAIL
TF_GUARD_STATUS_CAPTURED=FAIL
TF2_ECHO_MAP_BASE_LINK=FAIL
RVIZ_FIXED_FRAME_READY=FAIL
FAILURE_CLASS=UNKNOWN

deadline=$((SECONDS + WAIT_SEC))

node_present() {
  local nodes="$1"
  local name="$2"
  echo "$nodes" | grep -Eq "(^|/)$name$"
}

topic_exists() {
  local topics="$1"
  local topic="$2"
  echo "$topics" | grep -qx "$topic"
}

capture_topic() {
  local topic="$1"
  local out="$2"
  shift 2
  timeout 6 ros2 topic echo "$topic" --once --no-daemon "$@" > "$out" 2>&1
}

while true; do
  NODE_LIST=$(timeout 4 ros2 node list --no-daemon 2>/dev/null || true)
  TOPIC_LIST=$(timeout 4 ros2 topic list --no-daemon 2>/dev/null || true)

  core_count=0
  for node in visual_tf_guard_node fake_path_executor_node virtual_sensor_node bimodal_mode_mux_node; do
    if node_present "$NODE_LIST" "$node"; then core_count=$((core_count + 1)); fi
  done
  if [ "$core_count" -gt 0 ]; then DEMO_RUNNING=PASS; else DEMO_RUNNING=FAIL; fi

  if node_present "$NODE_LIST" visual_tf_guard_node || topic_exists "$TOPIC_LIST" /bimodal/tf_guard_status; then
    TF_GUARD_RUNNING=PASS
  else
    TF_GUARD_RUNNING=FAIL
  fi

  topic_exists "$TOPIC_LIST" /tf && TF_TOPIC_EXISTS=PASS || TF_TOPIC_EXISTS=FAIL
  topic_exists "$TOPIC_LIST" /tf_static && TF_STATIC_TOPIC_EXISTS=PASS || TF_STATIC_TOPIC_EXISTS=FAIL
  topic_exists "$TOPIC_LIST" /bimodal/tf_guard_status && TF_GUARD_STATUS_TOPIC_EXISTS=PASS || TF_GUARD_STATUS_TOPIC_EXISTS=FAIL

  if [ "$DEMO_RUNNING" = PASS ] && [ "$TF_GUARD_RUNNING" = PASS ] && \
     [ "$TF_TOPIC_EXISTS" = PASS ] && [ "$TF_STATIC_TOPIC_EXISTS" = PASS ] && \
     [ "$TF_GUARD_STATUS_TOPIC_EXISTS" = PASS ]; then
    break
  fi
  [ "$SECONDS" -ge "$deadline" ] && break
  sleep 1
done

if [ "$DEMO_RUNNING" = FAIL ]; then
  FAILURE_CLASS=DEMO_NOT_RUNNING
elif [ "$TF_GUARD_RUNNING" = FAIL ] || [ "$TF_GUARD_STATUS_TOPIC_EXISTS" = FAIL ]; then
  FAILURE_CLASS=TF_GUARD_NOT_RUNNING
elif [ "$TF_TOPIC_EXISTS" = FAIL ] || [ "$TF_STATIC_TOPIC_EXISTS" = FAIL ]; then
  FAILURE_CLASS=TF_TOPIC_MISSING
else
  if capture_topic /tf "$LOG/tf_once.txt"; then TF_MESSAGE_CAPTURED=PASS; fi
  if capture_topic /tf_static "$LOG/tf_static_once.txt" --qos-durability transient_local; then TF_STATIC_MESSAGE_CAPTURED=PASS; fi
  if capture_topic /bimodal/tf_guard_status "$LOG/tf_guard_status_once.txt"; then TF_GUARD_STATUS_CAPTURED=PASS; fi
  timeout 5 ros2 run tf2_ros tf2_echo map base_link > "$LOG/tf2_echo_map_base_link.txt" 2>&1 || true
  if grep -q "Translation:" "$LOG/tf2_echo_map_base_link.txt"; then TF2_ECHO_MAP_BASE_LINK=PASS; fi
  if [ "$TF_MESSAGE_CAPTURED" = PASS ] && [ "$TF_STATIC_MESSAGE_CAPTURED" = PASS ] && \
     [ "$TF_GUARD_STATUS_CAPTURED" = PASS ] && [ "$TF2_ECHO_MAP_BASE_LINK" = PASS ]; then
    RVIZ_FIXED_FRAME_READY=PASS
    FAILURE_CLASS=NONE
  else
    FAILURE_CLASS=TF_ECHO_FAIL
  fi
fi

{
  echo "# RViz TF Ready Report"
  echo
  echo "sample_time=$(date -Is)"
  echo "DEMO_RUNNING=$DEMO_RUNNING"
  echo "TF_GUARD_RUNNING=$TF_GUARD_RUNNING"
  echo "TF_TOPIC_EXISTS=$TF_TOPIC_EXISTS"
  echo "TF_STATIC_TOPIC_EXISTS=$TF_STATIC_TOPIC_EXISTS"
  echo "TF_GUARD_STATUS_TOPIC_EXISTS=$TF_GUARD_STATUS_TOPIC_EXISTS"
  echo "TF_MESSAGE_CAPTURED=$TF_MESSAGE_CAPTURED"
  echo "TF_STATIC_MESSAGE_CAPTURED=$TF_STATIC_MESSAGE_CAPTURED"
  echo "TF_GUARD_STATUS_CAPTURED=$TF_GUARD_STATUS_CAPTURED"
  echo "TF2_ECHO_MAP_BASE_LINK=$TF2_ECHO_MAP_BASE_LINK"
  echo "RVIZ_FIXED_FRAME_READY=$RVIZ_FIXED_FRAME_READY"
  echo "FAILURE_CLASS=$FAILURE_CLASS"
  echo "SUGGESTED_COMMAND=$SUGGESTED_COMMAND"
  if [ "$FAILURE_CLASS" = DEMO_NOT_RUNNING ]; then
    echo
    echo "Please run:"
    echo "$SUGGESTED_COMMAND"
  fi
} > "$REPORT"
cp "$REPORT" "$LOCAL_REPORT"
cat "$REPORT"
[ "$RVIZ_FIXED_FRAME_READY" = PASS ]
