#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
TS=$(date +%Y%m%d_%H%M%S)
LOG="${P6A_LOG_DIR:-$ROOT/test-log/${TS}_p6a_jetson_deployment_prep}"
mkdir -p "$LOG"
REPORT="$LOG/p6a_runtime_preflight.txt"

status=PASS
warnings=0
failures=0

warn() { warnings=$((warnings + 1)); echo "WARN: $*" >> "$REPORT"; }
fail() { failures=$((failures + 1)); echo "FAIL: $*" >> "$REPORT"; }
pass() { echo "PASS: $*" >> "$REPORT"; }

{
  echo "# P6A Runtime Preflight"
  echo "sample_time=$(date -Is)"
  echo "root=$ROOT"
  echo "log_dir=$LOG"
} > "$REPORT"

if [ -f "$ROOT/scripts/env_visual_demo.sh" ]; then
  set +u
  source "$ROOT/scripts/env_visual_demo.sh" >> "$REPORT" 2>&1 || warn "env_visual_demo.sh source returned nonzero"
  set -u
else
  fail "env_visual_demo.sh missing"
fi

if command -v ros2 >/dev/null 2>&1; then
  nodes=$(timeout 6 ros2 node list --no-daemon 2>/dev/null || true)
  topics=$(timeout 6 ros2 topic list --no-daemon 2>/dev/null || true)
else
  nodes=""
  topics=""
  fail "ros2 command unavailable"
fi

printf '%s\n' "$nodes" > "$LOG/p6a_preflight_node_list.txt"
printf '%s\n' "$topics" > "$LOG/p6a_preflight_topic_list.txt"

owned=$(printf '%s\n' "$nodes" | grep -E 'synthetic_external_pointcloud|real_sensor_pointcloud_bridge|octomap_pointcloud_backend|visual_tf_guard|demo_explainability_overlay|bimodal_mode_mux|fake_path_executor|air_exploration|ground_3d_frontier' || true)
if [ -n "$owned" ]; then
  warn "existing P5/P6 demo nodes are running; stop them before a clean live demo"
else
  pass "no existing P5/P6 demo nodes detected"
fi

forbidden=$(printf '%s\n' "$topics" | grep -E '^/cmd_vel$|^/mavros/|^/fmu/|^/actuator/|^/offboard_control_mode$|^/trajectory_setpoint$' || true)
if [ -n "$forbidden" ]; then
  fail "forbidden control topics detected: $forbidden"
else
  pass "no forbidden control topic currently visible"
fi

for path in \
  "$ROOT/scripts/run_p5b_explainable_bimodal_live_demo.sh" \
  "$ROOT/scripts/run_p5c_sensor_interface_validation.sh" \
  "$ROOT/scripts/run_p5c_real_bag_replay_live_demo.sh" \
  "$ROOT/scripts/run_p5c_live_external_pointcloud_demo.sh" \
  "$ROOT/scripts/check_p5b_live_demo_status.sh" \
  "$ROOT/Map/rviz/p5b_explainable_bimodal_demo.rviz" \
  "$ROOT/Map/config/p5c_real_sensor_input_interface.yaml"; do
  if [ -e "$path" ]; then pass "exists: $path"; else fail "missing: $path"; fi
done

if find "$ROOT/Map/ros2_ws" "$ROOT/Air/ros2_ws" "$ROOT/Ground/ros2_ws" -path '*/install/setup.bash' -type f | grep -q .; then
  pass "workspace install setup files are present"
else
  warn "workspace install setup files not found; run ./scripts/p6a_build_all.sh"
fi

if [ -w "$ROOT/test-log" ] || mkdir -p "$ROOT/test-log" 2>/dev/null; then
  pass "test-log writable"
else
  fail "test-log is not writable"
fi

display_limit=NO
if [ -z "${DISPLAY:-}" ]; then
  display_limit=YES
  warn "DISPLAY_ENVIRONMENT_LIMITATION: DISPLAY is unset"
elif command -v xdpyinfo >/dev/null 2>&1 && ! xdpyinfo >/dev/null 2>&1; then
  display_limit=YES
  warn "DISPLAY_ENVIRONMENT_LIMITATION: DISPLAY is set but X server is inaccessible"
else
  pass "DISPLAY appears usable or xdpyinfo is unavailable"
fi

if [ "$failures" -gt 0 ]; then
  status=FAIL
elif [ "$display_limit" = YES ]; then
  status=PASS_WITH_DISPLAY_LIMITATION
elif [ "$warnings" -gt 0 ]; then
  status=PASS_WITH_WARNINGS
fi

{
  echo "warning_count=$warnings"
  echo "failure_count=$failures"
  echo "P6A_RUNTIME_PREFLIGHT=$status"
  echo "report=$REPORT"
} >> "$REPORT"

cat "$REPORT"
[ "$status" != FAIL ]
