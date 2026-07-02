#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
INPUT_TOPIC=${P5C_INPUT_TOPIC:-}
ALLOW_FALLBACK=${P5C_ALLOW_SYNTHETIC_FALLBACK:-0}
MODE_SWITCH_PERIOD=${P5C_MODE_SWITCH_PERIOD_SEC:-75}
AIR_CONFIG=${P5C_AIR_CONFIG:-$ROOT/Air/config/p3b_air_fuel_quality.yaml}
GROUND_CONFIG=${P5C_GROUND_CONFIG:-$ROOT/Ground/config/p4b_ground_3d_quality.yaml}
RVIZ_CONFIG=${P5C_RVIZ_CONFIG:-$ROOT/Map/rviz/p5b_explainable_bimodal_demo.rviz}
TS=$(date +%Y%m%d_%H%M%S)
LOG="${P5C_LIVE_LOG_DIR:-$ROOT/test-log/${TS}_p5c_live_external_pointcloud_demo}"
mkdir -p "$LOG" "$LOG/ros_logs" "$LOG/ros_cli_logs"
printf '%s\n' "$LOG" > "$ROOT/test-log/.latest_p5c_live_external_pointcloud_demo_dir"
export ROS_LOG_DIR="$LOG/ros_logs"

set +u
source "$ROOT/scripts/env_visual_demo.sh" > "$LOG/env_visual_demo.txt" 2>&1
set -u

if [ -z "$INPUT_TOPIC" ]; then
  echo "P5C_LIVE_EXTERNAL_POINTCLOUD_DEMO=NOT_RUN"
  echo "reason=P5C_INPUT_TOPIC is required, for example P5C_INPUT_TOPIC=/camera/depth/points ./scripts/run_p5c_live_external_pointcloud_demo.sh"
  if [ "$ALLOW_FALLBACK" = "1" ]; then
    echo "synthetic_fallback_command=cd $ROOT && ./scripts/run_p5b_explainable_bimodal_live_demo.sh"
  fi
  exit 2
fi

P5C_LOG_DIR="$LOG" P5C_INPUT_TOPIC="$INPUT_TOPIC" "$ROOT/scripts/preflight_p5c_pointcloud_input.sh" > "$LOG/preflight_stdout.txt" 2>&1 || true
cat "$LOG/preflight_stdout.txt"
if ! grep -q '^preflight_result=PASS$' "$LOG/p5c_pointcloud_input_preflight_report.md" 2>/dev/null; then
  echo "P5C_LIVE_EXTERNAL_POINTCLOUD_DEMO=NOT_RUN"
  echo "reason=Input topic is missing or not sensor_msgs/msg/PointCloud2."
  if [ "$ALLOW_FALLBACK" = "1" ]; then
    echo "Starting explicit synthetic fallback because P5C_ALLOW_SYNTHETIC_FALLBACK=1."
    exec "$ROOT/scripts/run_p5b_explainable_bimodal_live_demo.sh"
  fi
  exit 12
fi

pids=()
cleanup() {
  {
    echo "# P5C Live External Topic Cleanup"
    echo "cleanup_time=$(date -Is)"
    echo "tracked_pids=${pids[*]:-NONE}"
  } > "$LOG/p5c_live_external_process_cleanup_check.txt"
  for p in "${pids[@]}"; do kill -- "-$p" 2>/dev/null || kill "$p" 2>/dev/null || true; done
  sleep 2
  for p in "${pids[@]}"; do kill -KILL -- "-$p" 2>/dev/null || kill -KILL "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

setsid ros2 launch bimodal_map_bringup p1c_real_sensor_pointcloud_input.launch.py \
  e2e_log_dir:="$LOG" sensor_input_mode:=external_pointcloud input_topic:="$INPUT_TOPIC" \
  enable_synthetic_pointcloud:=false scene_profile:=realistic_room_corridor_v1 \
  enable_explainability_overlay:=true backend_mode:=octomap_style_voxel \
  mode_switch_period_sec:="$MODE_SWITCH_PERIOD" > "$LOG/map_runtime.log" 2>&1 &
pids+=($!)
sleep 5
setsid ros2 launch bimodal_air_bringup air_baseline.launch.py planner_mode:=fuel_style_v0 \
  config_file:="$AIR_CONFIG" > "$LOG/air_runtime.log" 2>&1 &
pids+=($!)
setsid ros2 launch bimodal_ground_bringup ground_baseline.launch.py planner_mode:=ground_3d_frontier_v0 \
  config_file:="$GROUND_CONFIG" > "$LOG/ground_runtime.log" 2>&1 &
pids+=($!)

rviz_status=SKIP_DISPLAY_LIMITATION
if [ -n "${DISPLAY:-}" ] && command -v rviz2 >/dev/null 2>&1; then
  if ! command -v xdpyinfo >/dev/null 2>&1 || xdpyinfo >/dev/null 2>&1; then
    setsid rviz2 -d "$RVIZ_CONFIG" > "$LOG/rviz_runtime.log" 2>&1 &
    pids+=($!)
    rviz_status=STARTED
  fi
fi

echo "P5C_LIVE_EXTERNAL_POINTCLOUD_DEMO_STARTED=YES"
echo "selected_live_input_topic=$INPUT_TOPIC"
echo "synthetic_publisher_enabled=false"
echo "bridge_output_topic=/bimodal/points"
echo "rviz_status=$rviz_status"
echo "log_dir=$LOG"
echo "status_check_command=cd $ROOT && ./scripts/check_p5b_live_demo_status.sh"
echo "no_real_control_warning=This script does not publish /cmd_vel, /mavros/*, /fmu/*, /actuator/*, /offboard_control_mode, or /trajectory_setpoint."
echo "Press Ctrl-C to stop."

while true; do sleep 5; done
