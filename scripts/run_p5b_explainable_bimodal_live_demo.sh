#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
INPUT_TOPIC=${P5B_INPUT_TOPIC:-/points_raw}
SCENE_PROFILE=${P5B_SCENE_PROFILE:-realistic_room_corridor_v1}
MODE_SWITCH_PERIOD=${P5B_MODE_SWITCH_PERIOD_SEC:-75}
AIR_MODE=${P5B_AIR_PLANNER_MODE:-fuel_style_v0}
GROUND_MODE=${P5B_GROUND_PLANNER_MODE:-ground_3d_frontier_v0}
AIR_CONFIG=${P5B_AIR_CONFIG:-$ROOT/Air/config/p3b_air_fuel_quality.yaml}
GROUND_CONFIG=${P5B_GROUND_CONFIG:-$ROOT/Ground/config/p4b_ground_3d_quality.yaml}
RVIZ_CONFIG=${P5B_RVIZ_CONFIG:-$ROOT/Map/rviz/p5b_explainable_bimodal_demo.rviz}
TS=$(date +%Y%m%d_%H%M%S)
LOG="${P5B_LIVE_LOG_DIR:-$ROOT/test-log/${TS}_p5b_explainable_bimodal_live_demo}"
mkdir -p "$LOG" "$LOG/ros_logs" "$LOG/ros_cli_logs"
printf '%s\n' "$LOG" > "$ROOT/test-log/.latest_p5b_explainable_bimodal_live_demo_dir"
export ROS_LOG_DIR="$LOG/ros_logs"

set +u
source "$ROOT/scripts/env_visual_demo.sh" > "$LOG/env_visual_demo.txt" 2>&1
set -u

{
  echo "# P5B Live Demo Static Safety Scan"
  echo "sample_time=$(date -Is)"
  rg -n "create_publisher\\([^\\n]*(/cmd_vel|mavros|/fmu|actuator|offboard_control_mode|trajectory_setpoint)" \
    "$ROOT/Air" "$ROOT/Ground" "$ROOT/Map" 2>/dev/null || true
} > "$LOG/p5b_live_static_safety_scan.txt"
if rg -n "create_publisher\\([^\\n]*(/cmd_vel|mavros|/fmu|actuator|offboard_control_mode|trajectory_setpoint)" \
  "$ROOT/Air" "$ROOT/Ground" "$ROOT/Map" >/tmp/p5b_live_forbidden_publishers.txt 2>/dev/null; then
  cat /tmp/p5b_live_forbidden_publishers.txt
  echo "P5B_LIVE_DEMO_ABORTED=SAFETY_FAIL"
  exit 20
fi

cat > "$LOG/p5b_visual_layer_guide.md" <<'GUIDE'
# P5B Visual Layer Guide

- cyan points: incoming external PointCloud2 after bridge, `/bimodal/points`
- translucent grey/orange objects: synthetic room/corridor world structure, `/bimodal/demo_world_structure_markers`
- green/orange voxel map: built 3D occupied map, `/bimodal/map_3d` and `/bimodal/octomap_occupied_markers`
- translucent orange box: exploration boundary and valid search volume, `/bimodal/exploration_boundary`
- green coverage cubes: explored coverage proxy, `/bimodal/coverage_markers`
- cyan frontier hints: unknown boundary/frontier proxy, `/bimodal/octomap_frontier_markers`
- blue/purple markers: Air candidates and Air selected goal, `/air/candidate_markers`, `/air/selected_goal_marker`
- teal/green markers: Ground 3D frontier candidates, `/ground/frontier_candidates`
- red sphere: current selected bimodal goal, `/bimodal/selected_goal_marker`
- yellow path: current active path chosen by mode mux, `/bimodal/active_path`
- white trail: fake executed path, `/bimodal/executed_path`
- arrow/robot marker: current simulated robot pose, `/bimodal/robot_marker`, `/bimodal/executor_marker`
- text panels: mode, map, Air, Ground, TF and safety explanation, `/bimodal/demo_legend_markers`, `/bimodal/demo_status_text`
GUIDE

pids=()
cleanup() {
  {
    echo "# P5B Live Demo Process Cleanup"
    echo "cleanup_time=$(date -Is)"
    echo "tracked_pids=${pids[*]:-NONE}"
  } > "$LOG/p5b_live_process_cleanup_check.txt"
  for p in "${pids[@]}"; do kill -- "-$p" 2>/dev/null || kill "$p" 2>/dev/null || true; done
  sleep 2
  for p in "${pids[@]}"; do kill -KILL -- "-$p" 2>/dev/null || kill -KILL "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
  remaining=0
  for p in "${pids[@]}"; do kill -0 "$p" 2>/dev/null && remaining=$((remaining + 1)); done
  echo "remaining_owned_process_count=$remaining" >> "$LOG/p5b_live_process_cleanup_check.txt"
}
trap cleanup EXIT INT TERM

if pgrep -af "run_p5a_full_bimodal_live_demo.sh|run_p5b_explainable_bimodal_live_demo.sh" | grep -v "$$" > "$LOG/potential_existing_live_demo_processes.txt"; then
  echo "Existing live-demo wrapper processes were detected; inspect $LOG/potential_existing_live_demo_processes.txt if topics conflict."
fi

setsid ros2 launch bimodal_map_bringup p1c_real_sensor_pointcloud_input.launch.py \
  e2e_log_dir:="$LOG" sensor_input_mode:=external_pointcloud input_topic:="$INPUT_TOPIC" \
  enable_synthetic_pointcloud:=true scene_profile:="$SCENE_PROFILE" \
  enable_explainability_overlay:=true backend_mode:=octomap_style_voxel \
  mode_switch_period_sec:="$MODE_SWITCH_PERIOD" > "$LOG/map_runtime.log" 2>&1 &
pids+=($!)
sleep 5

setsid ros2 launch bimodal_air_bringup air_baseline.launch.py planner_mode:="$AIR_MODE" \
  config_file:="$AIR_CONFIG" > "$LOG/air_runtime.log" 2>&1 &
pids+=($!)

setsid ros2 launch bimodal_ground_bringup ground_baseline.launch.py planner_mode:="$GROUND_MODE" \
  config_file:="$GROUND_CONFIG" > "$LOG/ground_runtime.log" 2>&1 &
pids+=($!)

timeout 45 bash "$ROOT/scripts/check_p5b_live_demo_status.sh" > "$LOG/p5b_live_startup_status.txt" 2>&1 || true
timeout 6 ros2 topic list --no-daemon > "$LOG/p5b_live_topic_snapshot.txt" 2>&1 || true
ros2 topic list 2>/dev/null | grep -E "^/cmd_vel$|^/mavros/|^/fmu/|^/actuator/|^/offboard_control_mode$|^/trajectory_setpoint$" > "$LOG/p5b_live_no_real_control_topic_check.txt" || true
if [ ! -s "$LOG/p5b_live_no_real_control_topic_check.txt" ]; then
  {
    echo "no_real_control_topic=PASS"
    echo "forbidden_topic_detected_count=0"
    echo "forbidden_topic_list=NONE"
  } > "$LOG/p5b_live_no_real_control_topic_check.txt"
fi

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

echo "P5B_EXPLAINABLE_BIMODAL_LIVE_DEMO_STARTED=YES"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-}"
echo "scene_profile=$SCENE_PROFILE"
echo "rviz_status=$rviz_status"
echo "log_dir=$LOG"
echo "second_terminal_diagnostic_command=cd $ROOT && ./scripts/check_p5b_live_demo_status.sh"
echo "rviz_config_path=$RVIZ_CONFIG"
echo "topic_meaning_guide=$LOG/p5b_visual_layer_guide.md"
echo "no_real_control_warning=This demo is sim/visualization only and must not publish /cmd_vel, /mavros/*, /fmu/*, /actuator/*, /offboard_control_mode, or /trajectory_setpoint."
echo "headless_validation_command=cd $ROOT && P5B_DURATION_SEC=180 ./scripts/run_p5b_visual_explainability_validation.sh"
echo "Press Ctrl-C to stop and clean owned ROS processes."

while true; do sleep 5; done
