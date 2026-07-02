#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
INPUT_TOPIC=${P5A_INPUT_TOPIC:-/points_raw}
MODE_SWITCH_PERIOD=${P5A_MODE_SWITCH_PERIOD_SEC:-75}
AIR_MODE=${P5A_AIR_PLANNER_MODE:-fuel_style_v0}
GROUND_MODE=${P5A_GROUND_PLANNER_MODE:-ground_3d_frontier_v0}
AIR_CONFIG=${P5A_AIR_CONFIG:-$ROOT/Air/config/p3b_air_fuel_quality.yaml}
GROUND_CONFIG=${P5A_GROUND_CONFIG:-$ROOT/Ground/config/p4b_ground_3d_quality.yaml}
RVIZ_CONFIG=${P5A_RVIZ_CONFIG:-$ROOT/Map/rviz/p5a_full_bimodal_demo.rviz}
TS=$(date +%Y%m%d_%H%M%S)
LOG="${P5A_LIVE_LOG_DIR:-$ROOT/test-log/${TS}_p5a_full_bimodal_live_demo}"
mkdir -p "$LOG" "$LOG/ros_logs"
printf '%s\n' "$LOG" > "$ROOT/test-log/.latest_p5a_full_bimodal_live_demo_dir"
export ROS_LOG_DIR="$LOG/ros_logs"

set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
for setup in "$ROOT/Map/ros2_ws/install/setup.bash" "$ROOT/Air/ros2_ws/install/setup.bash" "$ROOT/Ground/ros2_ws/install/setup.bash"; do
  [ -f "$setup" ] && source "$setup"
done
source "$ROOT/scripts/env_visual_demo.sh" > "$LOG/env_visual_demo.txt" 2>&1 || true
set -u

{
  echo "# P5A Live Demo Static Safety Scan"
  rg -n "create_publisher\\([^\\n]*(/cmd_vel|mavros|/fmu|actuator|offboard_control_mode|trajectory_setpoint)" \
    "$ROOT/Air" "$ROOT/Ground" "$ROOT/Map" 2>/dev/null || true
} > "$LOG/p5a_live_static_safety_scan.txt"

if rg -n "create_publisher\\([^\\n]*(/cmd_vel|mavros|/fmu|actuator|offboard_control_mode|trajectory_setpoint)" \
  "$ROOT/Air" "$ROOT/Ground" "$ROOT/Map" >/tmp/p5a_live_forbidden_publishers.txt 2>/dev/null; then
  cat /tmp/p5a_live_forbidden_publishers.txt
  echo "P5A_LIVE_DEMO_ABORTED=SAFETY_FAIL"
  exit 20
fi

pids=()
cleanup() {
  {
    echo "# P5A Live Demo Process Cleanup"
    echo "cleanup_time=$(date -Is)"
    echo "tracked_pids=${pids[*]:-NONE}"
  } > "$LOG/p5a_live_process_cleanup_check.txt"
  for p in "${pids[@]}"; do kill -- "-$p" 2>/dev/null || kill "$p" 2>/dev/null || true; done
  sleep 2
  for p in "${pids[@]}"; do kill -KILL -- "-$p" 2>/dev/null || kill -KILL "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
  remaining=0
  for p in "${pids[@]}"; do kill -0 "$p" 2>/dev/null && remaining=$((remaining + 1)); done
  echo "remaining_owned_process_count=$remaining" >> "$LOG/p5a_live_process_cleanup_check.txt"
}
trap cleanup EXIT INT TERM

setsid ros2 launch bimodal_map_bringup p1c_real_sensor_pointcloud_input.launch.py \
  e2e_log_dir:="$LOG" sensor_input_mode:=external_pointcloud input_topic:="$INPUT_TOPIC" \
  enable_synthetic_pointcloud:=true backend_mode:=octomap_style_voxel \
  mode_switch_period_sec:="$MODE_SWITCH_PERIOD" > "$LOG/map_runtime.log" 2>&1 &
pids+=($!)
sleep 5

setsid ros2 launch bimodal_air_bringup air_baseline.launch.py planner_mode:="$AIR_MODE" \
  config_file:="$AIR_CONFIG" > "$LOG/air_runtime.log" 2>&1 &
pids+=($!)

setsid ros2 launch bimodal_ground_bringup ground_baseline.launch.py planner_mode:="$GROUND_MODE" \
  config_file:="$GROUND_CONFIG" > "$LOG/ground_runtime.log" 2>&1 &
pids+=($!)

timeout 45 bash "$ROOT/scripts/check_rviz_tf_ready.sh" --wait 20 > "$LOG/p5a_live_tf_snapshot.txt" 2>&1 || true
P2C_LIVE_DEMO_LOG_DIR="$LOG" timeout 45 bash "$ROOT/scripts/check_visual_topics_ready.sh" > "$LOG/p5a_live_visual_topics_report.txt" 2>&1 || true
timeout 6 ros2 topic list --no-daemon > "$LOG/p5a_live_topic_snapshot.txt" 2>&1 || true
ros2 topic list 2>/dev/null | grep -E "^/cmd_vel$|^/mavros/|^/fmu/|^/actuator/|^/offboard_control_mode$|^/trajectory_setpoint$" > "$LOG/p5a_live_no_real_control_topic_check.txt" || true

rviz_status=SKIP_DISPLAY_LIMITATION
if [ -n "${DISPLAY:-}" ] && command -v rviz2 >/dev/null 2>&1; then
  if ! command -v xdpyinfo >/dev/null 2>&1 || xdpyinfo >/dev/null 2>&1; then
    setsid rviz2 -d "$RVIZ_CONFIG" > "$LOG/rviz_runtime.log" 2>&1 &
    pids+=($!)
    rviz_status=STARTED
  else
    echo "DISPLAY_ENVIRONMENT_LIMITATION: DISPLAY is set but X server is not accessible." > "$LOG/rviz_runtime.log"
  fi
else
  echo "DISPLAY_ENVIRONMENT_LIMITATION: DISPLAY or rviz2 is unavailable." > "$LOG/rviz_runtime.log"
fi

echo "P5A_FULL_BIMODAL_LIVE_DEMO_STARTED=YES"
echo "rviz_status=$rviz_status"
echo "log_dir=$LOG"
echo "final_live_demo_command=cd $ROOT && ./scripts/run_p5a_full_bimodal_live_demo.sh"
echo "headless_validation_command=cd $ROOT && P5A_DURATION_SEC=300 ./scripts/run_p5a_full_bimodal_acceptance_validation.sh"
echo "rviz_only_command=cd $ROOT && rviz2 -d $RVIZ_CONFIG"
echo "debug_package=$ROOT/latest_p5a_full_bimodal_demo_package.tar.gz"
echo "Press Ctrl-C to stop and clean owned ROS processes."

while true; do sleep 5; done
