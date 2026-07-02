#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
BAG_PATH=${P5C_BAG_PATH:-}
INPUT_TOPIC=${P5C_INPUT_TOPIC:-}
BAG_PLAY_RATE=${P5C_BAG_PLAY_RATE:-1.0}
LOOP_BAG=${P5C_LOOP_BAG:-0}
USE_SIM_TIME=${P5C_USE_SIM_TIME:-0}
MODE_SWITCH_PERIOD=${P5C_MODE_SWITCH_PERIOD_SEC:-75}
AIR_CONFIG=${P5C_AIR_CONFIG:-$ROOT/Air/config/p3b_air_fuel_quality.yaml}
GROUND_CONFIG=${P5C_GROUND_CONFIG:-$ROOT/Ground/config/p4b_ground_3d_quality.yaml}
RVIZ_CONFIG=${P5C_RVIZ_CONFIG:-$ROOT/Map/rviz/p5b_explainable_bimodal_demo.rviz}
TS=$(date +%Y%m%d_%H%M%S)
LOG="${P5C_BAG_LOG_DIR:-$ROOT/test-log/${TS}_p5c_real_bag_replay_live_demo}"
mkdir -p "$LOG" "$LOG/ros_logs" "$LOG/ros_cli_logs"
printf '%s\n' "$LOG" > "$ROOT/test-log/.latest_p5c_real_bag_replay_live_demo_dir"
export ROS_LOG_DIR="$LOG/ros_logs"

set +u
source "$ROOT/scripts/env_visual_demo.sh" > "$LOG/env_visual_demo.txt" 2>&1
set -u

if [ -z "$BAG_PATH" ]; then
  echo "P5C_REAL_BAG_REPLAY_LIVE_DEMO=NOT_RUN"
  echo "reason=P5C_BAG_PATH is required."
  echo "usage=P5C_BAG_PATH=/path/to/rosbag2_dir ./scripts/run_p5c_real_bag_replay_live_demo.sh"
  echo "fallback_command=cd $ROOT && ./scripts/run_p5b_explainable_bimodal_live_demo.sh"
  exit 0
fi

P5C_LOG_DIR="$LOG" P5C_BAG_PATH="$BAG_PATH" "$ROOT/scripts/preflight_p5c_pointcloud_input.sh" > "$LOG/preflight_stdout.txt" 2>&1 || true
cat "$LOG/preflight_stdout.txt"
if [ -z "$INPUT_TOPIC" ]; then
  INPUT_TOPIC=$(awk -F= '/^selected_bag_pointcloud_topic=/ {print $2; exit}' "$LOG/p5c_pointcloud_input_preflight_report.md" 2>/dev/null || echo NONE)
fi
if [ -z "$INPUT_TOPIC" ] || [ "$INPUT_TOPIC" = "NONE" ]; then
  echo "P5C_REAL_BAG_REPLAY_LIVE_DEMO=NOT_RUN"
  echo "reason=No sensor_msgs/msg/PointCloud2 topic found in bag; no replay started."
  exit 10
fi

pids=()
cleanup() {
  {
    echo "# P5C Real Bag Replay Cleanup"
    echo "cleanup_time=$(date -Is)"
    echo "tracked_pids=${pids[*]:-NONE}"
  } > "$LOG/p5c_bag_process_cleanup_check.txt"
  for p in "${pids[@]}"; do kill -- "-$p" 2>/dev/null || kill "$p" 2>/dev/null || true; done
  sleep 2
  for p in "${pids[@]}"; do kill -KILL -- "-$p" 2>/dev/null || kill -KILL "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

bag_args=(play "$BAG_PATH" --rate "$BAG_PLAY_RATE")
if [ "$LOOP_BAG" = "1" ]; then bag_args+=(--loop); fi
if [ "$USE_SIM_TIME" = "1" ]; then bag_args+=(--clock); fi
setsid ros2 bag "${bag_args[@]}" > "$LOG/bag_play_runtime.log" 2>&1 &
pids+=($!)
sleep 3

setsid ros2 launch bimodal_map_bringup p1c_real_sensor_pointcloud_input.launch.py \
  e2e_log_dir:="$LOG" sensor_input_mode:=recorded_bag input_topic:="$INPUT_TOPIC" \
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

echo "P5C_REAL_BAG_REPLAY_LIVE_DEMO_STARTED=YES"
echo "bag_path=$BAG_PATH"
echo "selected_bag_pointcloud_topic=$INPUT_TOPIC"
echo "synthetic_publisher_enabled=false"
echo "bridge_output_topic=/bimodal/points"
echo "rviz_status=$rviz_status"
echo "log_dir=$LOG"
echo "status_check_command=cd $ROOT && ./scripts/check_p5b_live_demo_status.sh"
echo "no_real_control_warning=Bag may contain control topics, but this system does not publish /cmd_vel, /mavros/*, /fmu/*, /actuator/*, /offboard_control_mode, or /trajectory_setpoint."
echo "Press Ctrl-C to stop."

while true; do sleep 5; done
