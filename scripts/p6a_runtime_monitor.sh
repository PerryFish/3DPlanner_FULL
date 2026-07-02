#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
TS=$(date +%Y%m%d_%H%M%S)
LOG="${P6A_LOG_DIR:-$ROOT/test-log/${TS}_p6a_runtime_monitor}"
mkdir -p "$LOG"
REPORT="$LOG/p6a_runtime_monitor.txt"

status=PASS
warnings=0
failures=0

warn() { warnings=$((warnings + 1)); echo "WARN: $*" >> "$REPORT"; }
fail() { failures=$((failures + 1)); echo "FAIL: $*" >> "$REPORT"; }

{
  echo "# P6A Runtime Monitor"
  echo "sample_time=$(date -Is)"
  echo "root=$ROOT"
  echo "log_dir=$LOG"
  echo
  echo "## CPU"
  top -bn1 | head -20 2>/dev/null || true
  echo
  echo "## Memory"
  free -h 2>/dev/null || true
  echo
  echo "## Disk"
  df -h "$ROOT" 2>/dev/null || true
} > "$REPORT"

set +u
source "$ROOT/scripts/env_visual_demo.sh" >> "$REPORT" 2>&1 || warn "failed to source env_visual_demo.sh"
set -u

nodes=$(timeout 6 ros2 node list --no-daemon 2>/dev/null || true)
topics=$(timeout 6 ros2 topic list --no-daemon 2>/dev/null || true)
node_count=$(printf '%s\n' "$nodes" | sed '/^$/d' | wc -l)
topic_count=$(printf '%s\n' "$topics" | sed '/^$/d' | wc -l)

{
  echo
  echo "ros_node_count=$node_count"
  echo "ros_topic_count=$topic_count"
  echo
  echo "## Nodes"
  printf '%s\n' "$nodes"
  echo
  echo "## Topics"
  printf '%s\n' "$topics"
} >> "$REPORT"

if printf '%s\n' "$topics" | grep -qx /bimodal/points; then
  timeout 10 ros2 topic hz /bimodal/points --window 8 > "$LOG/p6a_bimodal_points_rate.txt" 2>&1 || warn "could not sample /bimodal/points rate"
else
  warn "/bimodal/points not visible"
fi

for topic in /bimodal/active_path /bimodal/executed_path /bimodal/map_metrics; do
  if printf '%s\n' "$topics" | grep -qx "$topic"; then
    timeout 5 ros2 topic echo "$topic" --once --no-daemon > "$LOG/$(basename "$topic").txt" 2>&1 || warn "could not capture $topic"
  else
    warn "$topic not visible"
  fi
done

forbidden=$(printf '%s\n' "$topics" | grep -E '^/cmd_vel$|^/mavros/|^/fmu/|^/actuator/|^/offboard_control_mode$|^/trajectory_setpoint$' || true)
if [ -n "$forbidden" ]; then
  fail "forbidden control topics detected: $forbidden"
fi

if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi >> "$REPORT" 2>&1 || warn "nvidia-smi sample failed"
else
  warn "nvidia-smi not found"
fi

if command -v tegrastats >/dev/null 2>&1; then
  echo "tegrastats is available. For continuous Jetson sampling run: tegrastats" >> "$REPORT"
  timeout 3 tegrastats >> "$REPORT" 2>&1 || true
else
  warn "tegrastats not found"
fi

if [ "$failures" -gt 0 ]; then
  status=FAIL
elif [ "$warnings" -gt 0 ]; then
  status=PASS_WITH_WARNINGS
fi

{
  echo "warning_count=$warnings"
  echo "failure_count=$failures"
  echo "P6A_RUNTIME_MONITOR=$status"
  echo "report=$REPORT"
} >> "$REPORT"

cat "$REPORT"
[ "$status" != FAIL ]
