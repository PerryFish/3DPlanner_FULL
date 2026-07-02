#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
TS=$(date +%Y%m%d_%H%M%S)
LOG="${P6A_LOG_DIR:-$ROOT/test-log/${TS}_p6a_jetson_deployment_prep}"
mkdir -p "$LOG"
REPORT="$LOG/p6a_environment_check.txt"

status=PASS
warnings=0
failures=0

warn() {
  warnings=$((warnings + 1))
  echo "WARN: $*" >> "$REPORT"
}

fail() {
  failures=$((failures + 1))
  echo "FAIL: $*" >> "$REPORT"
}

pass() {
  echo "PASS: $*" >> "$REPORT"
}

{
  echo "# P6A Environment Check"
  echo "sample_time=$(date -Is)"
  echo "root=$ROOT"
  echo "log_dir=$LOG"
  echo
} > "$REPORT"

if [ -r /etc/os-release ]; then
  . /etc/os-release
  echo "os_pretty_name=${PRETTY_NAME:-UNKNOWN}" >> "$REPORT"
  if echo "${PRETTY_NAME:-}" | grep -q "Ubuntu 22.04"; then pass "Ubuntu 22.04 detected"; else warn "Expected Ubuntu 22.04"; fi
else
  fail "Cannot read /etc/os-release"
fi

echo "uname=$(uname -a)" >> "$REPORT"
command -v python3 >/dev/null 2>&1 && echo "python=$(python3 --version 2>&1)" >> "$REPORT" || fail "python3 missing"
command -v colcon >/dev/null 2>&1 && echo "colcon=$(command -v colcon)" >> "$REPORT" || fail "colcon missing"
command -v rosdep >/dev/null 2>&1 && echo "rosdep=$(command -v rosdep)" >> "$REPORT" || warn "rosdep missing"

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  pass "ROS2 Humble setup exists"
else
  fail "ROS2 Humble setup missing at /opt/ros/humble/setup.bash"
fi

echo "ROS_DISTRO=${ROS_DISTRO:-UNSET}" >> "$REPORT"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-UNSET}" >> "$REPORT"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-UNSET}" >> "$REPORT"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-UNSET}" >> "$REPORT"
echo "DISPLAY=${DISPLAY:-UNSET}" >> "$REPORT"

{
  echo
  echo "## System"
  lscpu 2>/dev/null || true
  echo
  free -h 2>/dev/null || true
  echo
  df -h "$ROOT" 2>/dev/null || true
} >> "$REPORT"

if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi >> "$REPORT" 2>&1 || warn "nvidia-smi exists but failed"
else
  warn "nvidia-smi not found; this is normal on many Jetson systems"
fi

if command -v tegrastats >/dev/null 2>&1; then
  echo "tegrastats=$(command -v tegrastats)" >> "$REPORT"
else
  warn "tegrastats not found; expected on Jetson only"
fi

for ws in Map Air Ground; do
  if [ -d "$ROOT/$ws/ros2_ws" ]; then pass "$ws/ros2_ws exists"; else fail "$ws/ros2_ws missing"; fi
  if [ -f "$ROOT/$ws/ros2_ws/install/setup.bash" ]; then pass "$ws install/setup.bash exists"; else warn "$ws install/setup.bash missing; run build"; fi
done

for path in \
  "$ROOT/scripts/run_p5b_explainable_bimodal_live_demo.sh" \
  "$ROOT/scripts/check_p5b_live_demo_status.sh" \
  "$ROOT/scripts/run_p5c_sensor_interface_validation.sh" \
  "$ROOT/scripts/run_p5c_real_bag_replay_live_demo.sh" \
  "$ROOT/scripts/run_p5c_live_external_pointcloud_demo.sh" \
  "$ROOT/scripts/preflight_p5c_pointcloud_input.sh" \
  "$ROOT/Map/config/p5c_real_sensor_input_interface.yaml" \
  "$ROOT/Map/config/p5b_visual_explainability_demo.yaml" \
  "$ROOT/Map/rviz/p5b_explainable_bimodal_demo.rviz"; do
  if [ -e "$path" ]; then pass "exists: $path"; else fail "missing: $path"; fi
done

forbidden=""
if command -v ros2 >/dev/null 2>&1; then
  topics=$(timeout 6 ros2 topic list --no-daemon 2>/dev/null || true)
  forbidden=$(printf '%s\n' "$topics" | grep -E '^/cmd_vel$|^/mavros/|^/fmu/|^/actuator/|^/offboard_control_mode$|^/trajectory_setpoint$' || true)
fi
if [ -n "$forbidden" ]; then
  fail "forbidden control topics detected: $forbidden"
else
  pass "no forbidden control topic currently visible"
fi

if [ "$failures" -gt 0 ]; then
  status=FAIL
elif [ "$warnings" -gt 0 ]; then
  status=PASS_WITH_WARNINGS
fi

{
  echo
  echo "warning_count=$warnings"
  echo "failure_count=$failures"
  echo "P6A_ENV_CHECK=$status"
  echo "report=$REPORT"
} >> "$REPORT"

cat "$REPORT"
[ "$status" != FAIL ]
