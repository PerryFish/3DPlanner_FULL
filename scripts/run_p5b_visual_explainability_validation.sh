#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
DURATION=${P5B_DURATION_SEC:-180}
INPUT_TOPIC=${P5B_INPUT_TOPIC:-/points_raw}
SCENE_PROFILE=${P5B_SCENE_PROFILE:-realistic_room_corridor_v1}
MODE_SWITCH_PERIOD=${P5B_MODE_SWITCH_PERIOD_SEC:-75}
AIR_MODE=${P5B_AIR_PLANNER_MODE:-fuel_style_v0}
GROUND_MODE=${P5B_GROUND_PLANNER_MODE:-ground_3d_frontier_v0}
AIR_CONFIG=${P5B_AIR_CONFIG:-$ROOT/Air/config/p3b_air_fuel_quality.yaml}
GROUND_CONFIG=${P5B_GROUND_CONFIG:-$ROOT/Ground/config/p4b_ground_3d_quality.yaml}
RVIZ_CONFIG=${P5B_RVIZ_CONFIG:-$ROOT/Map/rviz/p5b_explainable_bimodal_demo.rviz}
TS=$(date +%Y%m%d_%H%M%S)
LOG="${P5B_LOG_DIR:-$ROOT/test-log/${TS}_p5b_visual_explainability}"
mkdir -p "$LOG" "$LOG/ros_logs" "$LOG/ros_cli_logs" "$LOG/samples" "$LOG/wrapper"
printf '%s\n' "$LOG" > "$ROOT/test-log/.latest_p5b_visual_explainability_dir"
export ROS_LOG_DIR="$LOG/ros_logs"
export P2C_LIVE_DEMO_LOG_DIR="$LOG"

GIT_BRANCH=$(git --git-dir="$ROOT/.git_3dplanner_full" --work-tree="$ROOT" branch --show-current 2>/dev/null || true)
GIT_COMMIT_BEFORE=$(git --git-dir="$ROOT/.git_3dplanner_full" --work-tree="$ROOT" rev-parse HEAD 2>/dev/null || true)

set +u
source "$ROOT/scripts/env_visual_demo.sh" > "$LOG/env_visual_demo.txt" 2>&1
set -u

cat > "$LOG/p5b_visual_tf_audit.md" <<'AUDIT'
# P5B Visual / TF Audit

## p5a_live_demo_behavior

- `scripts/run_p5a_full_bimodal_live_demo.sh` starts Map, Air, Ground, mode mux, fake executor, TF guard, and optional RViz, then stays alive in a `while true` loop until Ctrl-C.
- During a live demo, `/tf`, `/tf_static`, `/bimodal/tf_guard_status`, planner topics, paths, robot marker, and RViz marker topics should continue publishing.

## p5a_validation_cleanup_behavior

- `scripts/run_p5a_full_bimodal_acceptance_validation.sh` delegates to the P4B/P3B validation stack.
- Those validation scripts track launched process groups and clean them on exit. After validation completes, `tf2_echo map base_link` is expected to fail because the TF publisher has been stopped.

## tf_publication_source

- `visual_tf_guard_node` publishes `/tf_static` for `map->odom`, `base_link->camera_link`, and `base_link->lidar_link`.
- It publishes `/tf` dynamically for `odom->base_link`, using `/bimodal/odom` from `fake_path_executor_node` when available.
- It publishes `/bimodal/tf_guard_status` as a diagnostic string.

## tf_check_failure_reason

- If the user ran `tf2_echo` after validation cleanup, `/tf` and `/tf_static` are absent by design.
- If the user ran it while live demo was active, the likely causes are ROS_DOMAIN_ID/RMW mismatch between terminals, live demo startup failure, or `visual_tf_guard_node` not running.
- P5B adds `scripts/check_p5b_live_demo_status.sh` to classify these cases in a second terminal.

## current_visual_layers

- P5A displays `/bimodal/points`, `/bimodal/map_3d`, `/bimodal/coverage_markers`, Air candidates, Ground frontier candidates, active path, executed path, robot/executor markers, and TF.
- These layers are technically present but lack semantic grouping and an in-scene legend.

## current_visual_confusing_points

- The previous synthetic cloud profile creates an abstract moving plane/strip/cube shell, so the built map can look like a regular green block instead of a room/corridor.
- Air/Ground candidates, active path, executed path, and map voxels can overlap without a text explanation.
- RViz may retain last displayed markers visually even after validation nodes are cleaned up, which can mislead manual TF checks.

## proposed_p5b_visual_plan

- Add `realistic_room_corridor_v1` synthetic scene profile with floor, walls, corridor, open area, doorway, boxes, and pillar, revealed progressively.
- Add P5B explainability overlay topics: legend markers, status text, selected goal marker, and exploration state markers.
- Add P5B RViz config with grouped layers and meaningful names.
- Add a live demo script that keeps nodes running and a second-terminal diagnostic script that verifies TF, topics, rates, and safety.
AUDIT

cat > "$LOG/p5b_visual_layer_guide.md" <<'GUIDE'
# P5B Visual Layer Guide

- cyan points: incoming PointCloud2 through `/bimodal/points`
- grey/orange translucent objects: synthetic realistic room/corridor world structure
- green/orange voxels: built shared 3D occupied map and occupied marker overlay
- orange transparent box: exploration boundary
- green coverage markers: explored coverage proxy
- cyan frontier hints: unknown boundary/frontier proxy
- blue/purple markers: Air candidates and selected Air goal
- teal/green markers: Ground 3D frontier candidates
- red sphere: selected active bimodal goal
- yellow path: currently active path selected by mode mux
- white trail: fake executed path
- robot/executor arrow: simulated robot pose and current execution direction
- text panels: current mode, map metrics, Air/Ground status, TF guard state, and safety note
GUIDE

cat > "$LOG/p5b_live_demo_run_guide.md" <<'GUIDE'
# P5B Explainable Live Demo Run Guide

## Live demo

\`\`\`bash
cd /home/nuaa/ZHY/3DPlanner_FULL
./scripts/run_p5b_explainable_bimodal_live_demo.sh
\`\`\`

## Second-terminal diagnostic

\`\`\`bash
cd /home/nuaa/ZHY/3DPlanner_FULL
./scripts/check_p5b_live_demo_status.sh
\`\`\`

## Headless validation

\`\`\`bash
cd /home/nuaa/ZHY/3DPlanner_FULL
P5B_DURATION_SEC=180 ./scripts/run_p5b_visual_explainability_validation.sh
\`\`\`

## RViz only

\`\`\`bash
cd /home/nuaa/ZHY/3DPlanner_FULL
rviz2 -d /home/nuaa/ZHY/3DPlanner_FULL/Map/rviz/p5b_explainable_bimodal_demo.rviz
\`\`\`
GUIDE

{
  echo "# P5B Static Safety Scan"
  echo "sample_time=$(date -Is)"
  rg -n "create_publisher\\([^\\n]*(/cmd_vel|mavros|/fmu|actuator|offboard_control_mode|trajectory_setpoint)" \
    "$ROOT/Air" "$ROOT/Ground" "$ROOT/Map" 2>/dev/null || true
} > "$LOG/p5b_static_safety_scan.txt"
if rg -n "create_publisher\\([^\\n]*(/cmd_vel|mavros|/fmu|actuator|offboard_control_mode|trajectory_setpoint)" \
  "$ROOT/Air" "$ROOT/Ground" "$ROOT/Map" >/tmp/p5b_forbidden_publishers.txt 2>/dev/null; then
  {
    echo "P5B_VISUAL_EXPLAINABILITY_AND_REALISTIC_SCENE_DEMO=FAIL"
    echo "remaining_failure_class=SAFETY_FAIL"
    cat /tmp/p5b_forbidden_publishers.txt
  } > "$LOG/p5b_visual_explainability_summary.md"
  cat "$LOG/p5b_visual_explainability_summary.md"
  exit 20
fi

set +e
bash "$ROOT/scripts/setup_all_workspaces.sh" > "$LOG/build_full.log" 2>&1
build_rc=$?
set -e

{
  echo "# P5B Build Report"
  echo "BUILD_EXIT_CODE=$build_rc"
  if [ "$build_rc" -eq 0 ]; then
    echo "map_build=PASS"
    echo "air_build=PASS"
    echo "ground_build=PASS"
    echo "p5b_visual_build=PASS"
  else
    echo "map_build=FAIL"
    echo "air_build=FAIL"
    echo "ground_build=FAIL"
    echo "p5b_visual_build=FAIL"
  fi
} > "$LOG/build_report.md"

pids=()
cleanup_pass=FAIL
remaining_owned_process_count=0
cleanup_pids() {
  {
    echo "# P5B Process Cleanup Check"
    echo "cleanup_time=$(date -Is)"
    echo "tracked_pids=${pids[*]:-NONE}"
  } > "$LOG/p5b_process_cleanup_check.txt"
  for p in "${pids[@]}"; do kill -- "-$p" 2>/dev/null || kill "$p" 2>/dev/null || true; done
  sleep 2
  for p in "${pids[@]}"; do kill -KILL -- "-$p" 2>/dev/null || kill -KILL "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
  remaining_owned_process_count=0
  for p in "${pids[@]}"; do kill -0 "$p" 2>/dev/null && remaining_owned_process_count=$((remaining_owned_process_count + 1)); done
  if [ "$remaining_owned_process_count" -eq 0 ]; then cleanup_pass=PASS; else cleanup_pass=FAIL; fi
  echo "remaining_owned_process_count=$remaining_owned_process_count" >> "$LOG/p5b_process_cleanup_check.txt"
  echo "cleanup_pass=$cleanup_pass" >> "$LOG/p5b_process_cleanup_check.txt"
}
trap cleanup_pids EXIT
trap 'cleanup_pids; exit 0' INT TERM

capture_topic_once() {
  local topic="$1"
  local out="$2"
  shift 2
  timeout 6 ros2 topic echo "$topic" --once --no-daemon "$@" > "$out" 2>&1 || true
}

capture_string_data() {
  local topic="$1"
  local out="$2"
  timeout 5 ros2 topic echo "$topic" --once --no-daemon --field data >> "$out" 2>&1 || true
}

if [ "$build_rc" -eq 0 ]; then
  timeout 6 ros2 topic list --no-daemon > "$LOG/p5b_topic_snapshot_start.txt" 2>&1 || true
  timeout 6 ros2 node list --no-daemon > "$LOG/p5b_node_snapshot_start.txt" 2>&1 || true
  setsid ros2 launch bimodal_map_bringup p1c_real_sensor_pointcloud_input.launch.py \
    e2e_log_dir:="$LOG/wrapper" sensor_input_mode:=external_pointcloud input_topic:="$INPUT_TOPIC" \
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

  timeout 45 bash "$ROOT/scripts/check_p5b_live_demo_status.sh" > "$LOG/p5b_tf_snapshot.txt" 2>&1 || true
  timeout 6 ros2 topic list --no-daemon > "$LOG/p5b_topic_snapshot_start.txt" 2>&1 || true
  timeout 6 ros2 node list --no-daemon > "$LOG/p5b_node_snapshot_start.txt" 2>&1 || true

  end_time=$((SECONDS + DURATION))
  sample_idx=0
  while [ "$SECONDS" -lt "$end_time" ]; do
    echo "sample_index=$sample_idx sample_time=$(date -Is)" >> "$LOG/p5b_map_metrics.log"
    capture_string_data /bimodal/map_metrics "$LOG/p5b_map_metrics.log"
    echo "sample_index=$sample_idx sample_time=$(date -Is)" >> "$LOG/p5b_air_planner_status.log"
    capture_string_data /air/planner_status "$LOG/p5b_air_planner_status.log"
    echo "sample_index=$sample_idx sample_time=$(date -Is)" >> "$LOG/p5b_ground_planner_status.log"
    capture_string_data /ground/planner_status "$LOG/p5b_ground_planner_status.log"
    echo "sample_index=$sample_idx sample_time=$(date -Is)" >> "$LOG/p5b_active_mode.log"
    capture_string_data /bimodal/active_mode "$LOG/p5b_active_mode.log"
    echo "sample_index=$sample_idx sample_time=$(date -Is)" >> "$LOG/p5b_demo_status_text.log"
    capture_string_data /bimodal/demo_status_string "$LOG/p5b_demo_status_text.log"
    capture_topic_once /bimodal/active_path "$LOG/samples/${sample_idx}_active_path.txt"
    capture_topic_once /bimodal/executed_path "$LOG/samples/${sample_idx}_executed_path.txt"
    capture_topic_once /bimodal/demo_legend_markers "$LOG/samples/${sample_idx}_demo_legend_markers.txt"
    capture_topic_once /bimodal/demo_status_text "$LOG/samples/${sample_idx}_demo_status_text.txt"
    capture_topic_once /bimodal/selected_goal_marker "$LOG/samples/${sample_idx}_selected_goal_marker.txt"
    sample_idx=$((sample_idx + 1))
    sleep 10
  done

  timeout 15 ros2 topic hz /bimodal/points --window 20 > "$LOG/p5b_points_rate.txt" 2>&1 || true
  timeout 6 ros2 topic list --no-daemon > "$LOG/p5b_topic_snapshot_end.txt" 2>&1 || true
  timeout 6 ros2 node list --no-daemon > "$LOG/p5b_node_snapshot_end.txt" 2>&1 || true
  cat "$LOG"/samples/*_active_path.txt > "$LOG/p5b_active_path.log" 2>/dev/null || true
  cat "$LOG"/samples/*_executed_path.txt > "$LOG/p5b_executed_path.log" 2>/dev/null || true
  {
    echo "# P5B Visual Topic Check"
    topics=$(timeout 8 ros2 topic list --no-daemon 2>/dev/null || true)
    ready=PASS
    for topic in /tf /tf_static /bimodal/tf_guard_status /bimodal/points /bimodal/map_3d \
      /bimodal/coverage_markers /bimodal/octomap_occupied_markers /bimodal/octomap_frontier_markers \
      /bimodal/exploration_boundary /bimodal/demo_world_structure_markers /bimodal/demo_legend_markers \
      /bimodal/exploration_state_markers /bimodal/demo_status_text /bimodal/selected_goal_marker \
      /air/candidate_markers /air/selected_goal_marker /ground/frontier_candidates \
      /bimodal/active_mode /bimodal/active_path /bimodal/executed_path /bimodal/robot_marker; do
      if echo "$topics" | grep -qx "$topic"; then echo "$topic=PASS"; else echo "$topic=FAIL"; ready=FAIL; fi
    done
    echo "VISUAL_TOPICS_READY_FOR_RVIZ=$ready"
  } > "$LOG/p5b_visual_topic_check.txt"
  ros2 topic list 2>/dev/null | grep -E "^/cmd_vel$|^/mavros/|^/fmu/|^/actuator/|^/offboard_control_mode$|^/trajectory_setpoint$" > "$LOG/p5b_no_real_control_topic_check.txt" || true
  if [ ! -s "$LOG/p5b_no_real_control_topic_check.txt" ]; then
    {
      echo "no_real_control_topic=PASS"
      echo "forbidden_topic_detected_count=0"
      echo "forbidden_topic_list=NONE"
    } > "$LOG/p5b_no_real_control_topic_check.txt"
  fi
fi

cleanup_pids
trap - EXIT INT TERM

python3 - "$LOG" "$DURATION" "$SCENE_PROFILE" "$INPUT_TOPIC" "$build_rc" "$GIT_BRANCH" "$GIT_COMMIT_BEFORE" "$RVIZ_CONFIG" <<'PY'
import csv
import re
import sys
from pathlib import Path

log = Path(sys.argv[1])
duration = float(sys.argv[2])
scene_profile = sys.argv[3]
input_topic = sys.argv[4]
build_rc = int(sys.argv[5])
git_branch = sys.argv[6]
git_commit_before = sys.argv[7]
rviz_config = Path(sys.argv[8])
root = Path('/home/nuaa/ZHY/3DPlanner_FULL')
wrapper = log / 'wrapper'

def read(path):
    p = Path(path)
    return p.read_text(errors='ignore') if p.exists() else ''

def rel(name):
    return read(log / name)

def rows(name):
    p = wrapper / name
    if not p.exists():
        return []
    with p.open(newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))

def vals(text, key):
    return [m.group(1).strip('",') for m in re.finditer(rf'{re.escape(key)}=([^\s]+)', text)]

def fvals(text, key):
    out = []
    for v in vals(text, key):
        try:
            out.append(float(v.strip('(),')))
        except Exception:
            pass
    return out

def maxv(text, key, default=0.0):
    v = fvals(text, key)
    return max(v) if v else default

def firstv(seq, default=0.0):
    return seq[0] if seq else default

def lastv(seq, default=0.0):
    return seq[-1] if seq else default

def pf(x):
    return 'PASS' if x else 'FAIL'

def avg_rate(text):
    m = re.search(r'average rate:\s*([0-9.]+)', text)
    return float(m.group(1)) if m else 0.0

def cleanup_value(key, default='FAIL'):
    for line in rel('p5b_process_cleanup_check.txt').splitlines():
        if line.startswith(key + '='):
            return line.split('=', 1)[1].strip()
    return default

build = rel('build_report.md')
topic_text = rel('p5b_topic_snapshot_end.txt')
node_text = rel('p5b_node_snapshot_end.txt')
map_text = rel('p5b_map_metrics.log')
air_text = rel('p5b_air_planner_status.log')
ground_text = rel('p5b_ground_planner_status.log')
mode_text = rel('p5b_active_mode.log')
active_path_text = rel('p5b_active_path.log')
executed_path_text = rel('p5b_executed_path.log')
visual_text = rel('p5b_visual_topic_check.txt')
tf_text = rel('p5b_tf_snapshot.txt')
safety_text = rel('p5b_no_real_control_topic_check.txt')
points_rate = avg_rate(rel('p5b_points_rate.txt'))

map_rows = rows('p2e_octomap_map_metrics.csv')
odom_rows = rows('e2e_odom.csv')
path_rows = rows('e2e_paths.csv')
occ_vals = [float(r.get('occupied_voxel_count') or 0.0) for r in map_rows] or fvals(map_text, 'occupied_voxel_count')
cov_vals = [float(r.get('coverage_proxy') or 0.0) for r in map_rows] or fvals(map_text, 'coverage_proxy')
occ_start, occ_end = int(firstv(occ_vals, 0)), int(lastv(occ_vals, 0))
cov_start, cov_end = float(firstv(cov_vals, 0.0)), float(lastv(cov_vals, 0.0))
active_rows = [r for r in path_rows if r.get('source') == 'active']
active_hashes = {f'{r.get("pose_count")}:{round(float(r.get("path_length") or 0.0), 2)}' for r in active_rows}
odom_total = float(odom_rows[-1].get('total_distance') or 0.0) if odom_rows else maxv(rel('p5b_demo_status_text.log'), 'executed_path_length', 0.0)
executed_length = odom_total
if path_rows:
    executed_lengths = [float(r.get('path_length') or 0.0) for r in path_rows if r.get('source') == 'executed']
    if executed_lengths:
        executed_length = max(executed_lengths)

forbidden = [line.strip() for line in safety_text.splitlines() if re.match(r'^(/cmd_vel|/mavros/|/fmu/|/actuator/|/offboard_control_mode$|/trajectory_setpoint$)', line.strip())]

required_visual = [
    '/bimodal/points', '/bimodal/map_3d', '/bimodal/coverage_markers', '/bimodal/demo_world_structure_markers',
    '/bimodal/demo_legend_markers', '/bimodal/demo_status_text', '/bimodal/selected_goal_marker',
    '/air/candidate_markers', '/ground/frontier_candidates', '/bimodal/active_path',
    '/bimodal/executed_path', '/bimodal/robot_marker',
]
missing = [t for t in required_visual if f'{t}=PASS' not in visual_text and t not in topic_text.splitlines()]

map_build = 'PASS' if 'map_build=PASS' in build else 'FAIL'
air_build = 'PASS' if 'air_build=PASS' in build else 'FAIL'
ground_build = 'PASS' if 'ground_build=PASS' in build else 'FAIL'
p5b_build = 'PASS' if build_rc == 0 and 'p5b_visual_build=PASS' in build else 'FAIL'
external_chain = pf('/bimodal/points' in topic_text and points_rate > 0.0)
shared_map = pf('/bimodal/map_3d' in topic_text and occ_end > occ_start and cov_end > cov_start)
air_candidate = int(maxv(air_text, 'air_candidate_count', 0))
air_selected = int(maxv(air_text, 'air_selected_goal_count', 0))
air_fallback = 'YES' if 'air_fallback_active=true' in air_text or 'fallback_active=true' in air_text else 'NO'
ground_candidate = int(maxv(ground_text, 'ground_candidate_count', 0))
ground_selected = int(maxv(ground_text, 'ground_selected_goal_count', 0))
ground_projection = 'PASS' if 'ground_uses_3d_map_projection=true' in ground_text or 'ground_uses_3d_map_projection=PASS' in ground_text else 'FAIL'
ground_2d = 'YES' if 'slam_toolbox' in node_text.lower() else 'NO'
ground_fallback = 'YES' if 'ground_fallback_active=true' in ground_text or 'fallback_active=true' in ground_text else 'NO'
active_mode_switch_count = max(mode_text.count('AIR'), 0) + max(mode_text.count('GROUND'), 0)
active_mode_captured = pf('AIR' in mode_text or 'GROUND' in mode_text)
active_path_captured = pf(bool(active_rows) or 'poses:' in active_path_text)
executed_path_captured = pf(odom_total > 0.0 or 'poses:' in executed_path_text)
full_chain = pf(active_mode_captured == 'PASS' and active_path_captured == 'PASS' and executed_path_captured == 'PASS' and odom_total > 0.0)
no_real = pf(not forbidden)
cleanup_pass = cleanup_value('cleanup_pass')
remaining_owned = cleanup_value('remaining_owned_process_count', '0')

tf_topic_exists = 'YES' if 'tf_topic_exists=YES' in tf_text or '/tf=PASS' in visual_text else 'NO'
tf_static_exists = 'YES' if 'tf_static_topic_exists=YES' in tf_text or '/tf_static=PASS' in visual_text else 'NO'
tf_echo = 'PASS' if 'tf_echo_map_base_link=PASS' in tf_text or 'TF2_ECHO_MAP_BASE_LINK=PASS' in tf_text else 'FAIL'
tf_guard_status = 'PASS' if 'tf_guard_status=PASS' in tf_text or '/bimodal/tf_guard_status=PASS' in visual_text else 'FAIL'
tf_failure_reason = 'NONE'
for line in tf_text.splitlines():
    if line.startswith('failure_reason='):
        tf_failure_reason = line.split('=', 1)[1].strip()
p5b_tf = pf(tf_topic_exists == 'YES' and tf_static_exists == 'YES' and tf_echo == 'PASS' and tf_guard_status == 'PASS')
rviz_fixed = p5b_tf
visual_ready = pf(not missing and 'VISUAL_TOPICS_READY_FOR_RVIZ=PASS' in visual_text)
tf_tree_ready = p5b_tf

legend = 'PASS' if '/bimodal/demo_legend_markers=PASS' in visual_text or '/bimodal/demo_legend_markers' in topic_text else 'FAIL'
status_text = 'PASS' if '/bimodal/demo_status_text=PASS' in visual_text or '/bimodal/demo_status_text' in topic_text else 'FAIL'
selected_goal_marker = 'PASS' if '/bimodal/selected_goal_marker=PASS' in visual_text or '/bimodal/selected_goal_marker' in topic_text else 'FAIL'
final_rviz_config = pf(rviz_config.exists())
live_script = pf((root / 'scripts/run_p5b_explainable_bimodal_live_demo.sh').exists())
status_script = pf((root / 'scripts/check_p5b_live_demo_status.sh').exists())
layer_guide = pf((log / 'p5b_visual_layer_guide.md').exists())
visual_explainability = pf(final_rviz_config == 'PASS' and live_script == 'PASS' and status_script == 'PASS' and layer_guide == 'PASS' and legend == 'PASS' and status_text == 'PASS' and selected_goal_marker == 'PASS' and not missing)
realistic_scene = pf(scene_profile in ('realistic_room_corridor_v1', 'warehouse_obstacles_v1') and '/bimodal/demo_world_structure_markers' in topic_text and points_rate > 0.0 and shared_map == 'PASS')
air_reg = pf(air_candidate > 0 and air_selected >= 5 and air_fallback == 'NO')
ground_reg = pf(ground_candidate > 0 and ground_selected >= 5 and ground_projection == 'PASS' and ground_2d == 'NO' and ground_fallback == 'NO')
p5a_reg = pf(external_chain == 'PASS' and shared_map == 'PASS' and air_reg == 'PASS' and ground_reg == 'PASS' and full_chain == 'PASS')
p4b_reg = ground_reg
p3b_reg = air_reg

result = 'PASS'
failure = 'NONE'
reason = 'No remaining P5B failure.'
checks = [map_build, air_build, ground_build, p5b_build, p5a_reg, p4b_reg, p3b_reg, visual_explainability, realistic_scene, p5b_tf, full_chain, shared_map, no_real, cleanup_pass]
if any(v != 'PASS' for v in checks):
    result = 'FAIL'
    if no_real != 'PASS':
        failure, reason = 'SAFETY_FAIL', 'Forbidden control topic detected.'
    elif p5b_build != 'PASS':
        failure, reason = 'BUILD_FAIL', 'Build failed.'
    elif visual_explainability != 'PASS':
        failure, reason = 'VISUAL_EXPLAINABILITY_FAIL', 'P5B visual explanation topics/config are incomplete.'
    elif realistic_scene != 'PASS':
        failure, reason = 'REALISTIC_SCENE_FAIL', 'Realistic scene profile or world structure marker was not validated.'
    elif p5b_tf != 'PASS':
        failure, reason = 'TF_LIVE_CHECK_FAIL', tf_failure_reason
    elif shared_map != 'PASS':
        failure, reason = 'MAP_NOT_GROWING', 'Shared 3D map did not grow.'
    elif air_reg != 'PASS':
        failure, reason = 'AIR_REGRESSION', 'Air P3B regression failed.'
    elif ground_reg != 'PASS':
        failure, reason = 'GROUND_REGRESSION', 'Ground P4B regression failed.'
    elif full_chain != 'PASS':
        failure, reason = 'BIMODAL_CHAIN_FAIL', 'Active/executed path chain failed.'
    elif cleanup_pass != 'PASS':
        failure, reason = 'CLEANUP_FAIL', 'Owned process cleanup failed.'
    else:
        failure, reason = 'UNKNOWN_FAIL', 'Unknown P5B validation failure.'

summary = {
    'project_stage': 'P5_FINAL_BIMODAL_EXPLORATION_DEMO',
    'current_substage': 'P5B_VISUAL_EXPLAINABILITY_AND_REALISTIC_SCENE_DEMO',
    'work_done': 'Added realistic synthetic scene profile, explainability overlay markers, P5B RViz config, live demo/status scripts, validation, guides, and debug package',
    'work_not_done': 'No real control, no GitHub push, no Air/Ground algorithm rewrite, no 2D SLAM/Nav2/nvblox/RTAB-Map',
    'root': str(root),
    'log_dir': str(log),
    'git_branch': git_branch,
    'git_commit_before': git_commit_before,
    'git_commit_after': 'NONE',
    'map_build': map_build,
    'air_build': air_build,
    'ground_build': ground_build,
    'p5b_visual_build': p5b_build,
    'p5a_regression_guard': p5a_reg,
    'p4b_regression_guard': p4b_reg,
    'p3b_regression_guard': p3b_reg,
    'scene_profile': scene_profile,
    'realistic_synthetic_scene': realistic_scene,
    'selected_input_mode': 'external_pointcloud',
    'selected_input_topic': input_topic,
    'bridge_output_topic': '/bimodal/points',
    'map_backend_mode': 'octomap_style_voxel',
    'duration_sec': f'{duration:.3f}',
    'external_pointcloud_chain': external_chain,
    'bimodal_points_captured': pf(points_rate > 0.0),
    'bimodal_points_rate_avg_hz': f'{points_rate:.3f}',
    'shared_3d_map_chain': shared_map,
    'occupied_voxel_count_start': str(occ_start),
    'occupied_voxel_count_end': str(occ_end),
    'occupied_voxel_count_delta': str(occ_end - occ_start),
    'coverage_proxy_start': f'{cov_start:.6f}',
    'coverage_proxy_end': f'{cov_end:.6f}',
    'coverage_proxy_delta': f'{cov_end - cov_start:.6f}',
    'air_p3b_regression': air_reg,
    'air_candidate_count': str(air_candidate),
    'air_selected_goal_count': str(air_selected),
    'air_fallback_active': air_fallback,
    'ground_p4b_regression': ground_reg,
    'ground_candidate_count': str(ground_candidate),
    'ground_selected_goal_count': str(ground_selected),
    'ground_uses_3d_map_projection': ground_projection,
    'ground_2d_slam_dependency_detected': ground_2d,
    'ground_fallback_active': ground_fallback,
    'active_mode_captured': active_mode_captured,
    'active_mode_switch_count': str(active_mode_switch_count),
    'active_path_captured': active_path_captured,
    'active_path_unique_hash_count': str(len(active_hashes)),
    'executed_path_captured': executed_path_captured,
    'executed_path_length_m': f'{executed_length:.3f}',
    'odom_total_distance_m': f'{odom_total:.3f}',
    'p5b_visual_explainability': visual_explainability,
    'final_rviz_config_created': final_rviz_config,
    'final_live_demo_script_created': live_script,
    'live_status_check_script_created': status_script,
    'visual_layer_guide_created': layer_guide,
    'demo_legend_markers': legend,
    'demo_status_text': status_text,
    'selected_goal_marker': selected_goal_marker,
    'visual_topic_missing': ','.join(missing) if missing else 'NONE',
    'p5b_live_tf_check': p5b_tf,
    'tf_topic_exists': tf_topic_exists,
    'tf_static_topic_exists': tf_static_exists,
    'tf_echo_map_base_link': tf_echo,
    'tf_guard_status': tf_guard_status,
    'tf_check_failure_reason': tf_failure_reason,
    'rviz_fixed_frame_ready': rviz_fixed,
    'visual_topics_ready': visual_ready,
    'tf_tree_ready': tf_tree_ready,
    'no_real_control_topic': no_real,
    'forbidden_topic_detected_count': str(len(forbidden)),
    'forbidden_topic_list': ','.join(forbidden) if forbidden else 'NONE',
    'cleanup_pass': cleanup_pass,
    'remaining_owned_process_count': remaining_owned,
    'P5B_VISUAL_EXPLAINABILITY_AND_REALISTIC_SCENE_DEMO': result,
    'changed_files': 'PENDING',
    'local_commit_created': 'PENDING',
    'main_result_summary': 'P5B explainable realistic visual demo validation completed.' if result == 'PASS' else 'P5B explainable visual demo validation failed.',
    'remaining_failure_class': failure,
    'remaining_failure_reason': reason,
    'summary_report': str(log / 'p5b_visual_explainability_summary.md'),
    'visual_layer_guide': str(log / 'p5b_visual_layer_guide.md'),
    'live_demo_run_guide': str(log / 'p5b_live_demo_run_guide.md'),
    'debug_package': str(root / 'latest_p5b_visual_explainability_package.tar.gz'),
    'final_live_demo_command': f'cd {root} && ./scripts/run_p5b_explainable_bimodal_live_demo.sh',
    'live_status_check_command': f'cd {root} && ./scripts/check_p5b_live_demo_status.sh',
    'headless_validation_command': f'cd {root} && P5B_DURATION_SEC=180 ./scripts/run_p5b_visual_explainability_validation.sh',
    'rviz_only_command': f'cd {root} && rviz2 -d {rviz_config}',
    'recommended_next_prompt_type': 'GITHUB_BACKUP_P5B' if result == 'PASS' else 'FIX_P5B_VISUAL_EXPLAINABILITY',
    'next_stage_explanation': 'P5B visual baseline is explainable enough to back up.' if result == 'PASS' else 'Fix the P5B failure before backup or deployment prep.',
}

block = 'B3D_P5B_VISUAL_EXPLAINABILITY_SUMMARY\n' + '\n'.join(f'{k}={v}' for k, v in summary.items())
(log / 'p5b_visual_explainability_summary.md').write_text(block + '\n', encoding='utf-8')
print(block)
PY

tar -czf "$ROOT/latest_p5b_visual_explainability_package.tar.gz" -C "$(dirname "$LOG")" "$(basename "$LOG")"
