#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
DURATION=${P3B_DURATION_SEC:-300}
INPUT_TOPIC=${P3B_INPUT_TOPIC:-/points_raw}
AIR_MODE=${P3B_AIR_PLANNER_MODE:-fuel_style_v0}
GROUND_MODE=${P3B_GROUND_PLANNER_MODE:-ground_3d_frontier_v0}
MODE_SWITCH_PERIOD=${P3B_MODE_SWITCH_PERIOD_SEC:-75}
AIR_CONFIG=${P3B_AIR_CONFIG:-$ROOT/Air/config/p3b_air_fuel_quality.yaml}
GROUND_CONFIG=${P3B_GROUND_CONFIG:-$ROOT/Ground/config/p4a_ground_3d_frontier_v0.yaml}
SKIP_BUILD=0

while [ "$#" -gt 0 ]; do
  case "$1" in
    --duration) DURATION="$2"; shift 2 ;;
    --input-topic) INPUT_TOPIC="$2"; shift 2 ;;
    --air-planner-mode) AIR_MODE="$2"; shift 2 ;;
    --ground-planner-mode) GROUND_MODE="$2"; shift 2 ;;
    --air-config) AIR_CONFIG="$2"; shift 2 ;;
    --ground-config) GROUND_CONFIG="$2"; shift 2 ;;
    --mode-switch-period) MODE_SWITCH_PERIOD="$2"; shift 2 ;;
    --skip-build) SKIP_BUILD=1; shift ;;
    *) echo "unknown_arg=$1" >&2; exit 2 ;;
  esac
done

TS=$(date +%Y%m%d_%H%M%S)
LOG="${P3B_LOG_DIR:-$ROOT/test-log/${TS}_p3b_air_fuel_quality}"
mkdir -p "$LOG" "$LOG/ros_logs" "$LOG/ros_cli_logs" "$LOG/wrapper/samples"
printf '%s\n' "$LOG" > "$ROOT/test-log/.latest_p3b_air_fuel_quality_dir"
export P2C_LIVE_DEMO_LOG_DIR="$LOG/wrapper"
export ROS_LOG_DIR="$LOG/ros_logs"

GIT_BRANCH=$(git --git-dir="$ROOT/.git_3dplanner_full" --work-tree="$ROOT" branch --show-current 2>/dev/null || true)
GIT_COMMIT_BEFORE=$(git --git-dir="$ROOT/.git_3dplanner_full" --work-tree="$ROOT" rev-parse HEAD 2>/dev/null || true)

set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$ROOT/scripts/env_visual_demo.sh" > "$LOG/env_visual_demo.txt" 2>&1 || true
set -u

if [ ! -f "$LOG/p3b_air_fuel_quality_audit.md" ]; then
  cp "$ROOT/test-log/20260702_103026_p3b_air_fuel_quality/p3b_air_fuel_quality_audit.md" "$LOG/p3b_air_fuel_quality_audit.md" 2>/dev/null || true
fi

{
  echo "# P3B Static Safety Scan"
  echo
  rg -n "create_publisher\\([^\\n]*(/cmd_vel|mavros|/fmu|actuator|offboard_control_mode|trajectory_setpoint)" "$ROOT/Air" "$ROOT/Ground" "$ROOT/Map" 2>/dev/null || true
} > "$LOG/p3b_static_safety_scan.txt"

if [ "$SKIP_BUILD" -eq 0 ]; then
  set +e
  bash "$ROOT/scripts/setup_all_workspaces.sh" > "$LOG/build_full.log" 2>&1
  build_rc=$?
  set -e
else
  build_rc=0
  echo "SKIP_BUILD=YES" > "$LOG/build_full.log"
fi

{
  echo "# P3B Build Report"
  echo
  echo "BUILD_EXIT_CODE=$build_rc"
  if [ "$build_rc" -eq 0 ]; then
    echo "map_build=PASS"
    echo "air_build=PASS"
    echo "ground_build=PASS"
    echo "p3b_air_quality_build=PASS"
  else
    echo "map_build=FAIL"
    echo "air_build=FAIL"
    echo "ground_build=FAIL"
    echo "p3b_air_quality_build=FAIL"
  fi
} > "$LOG/build_report.md"

pids=()
launched_process_count=0
cleanup_pass=FAIL
remaining_owned_process_count=0

cleanup_pids() {
  local report="$LOG/p3b_process_cleanup_check.txt"
  {
    echo "# P3B Process Cleanup Check"
    echo
    echo "cleanup_time=$(date -Is)"
    echo "launched_process_count=$launched_process_count"
    echo "tracked_pids=${pids[*]:-NONE}"
  } > "$report"
  for p in "${pids[@]}"; do kill -- "-$p" 2>/dev/null || kill "$p" 2>/dev/null || true; done
  sleep 2
  for p in "${pids[@]}"; do kill -KILL -- "-$p" 2>/dev/null || kill -KILL "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
  remaining_owned_process_count=0
  for p in "${pids[@]}"; do
    if kill -0 "$p" 2>/dev/null; then remaining_owned_process_count=$((remaining_owned_process_count + 1)); fi
  done
  if [ "$remaining_owned_process_count" -eq 0 ]; then cleanup_pass=PASS; else cleanup_pass=FAIL; fi
  {
    echo "cleaned_process_count=$((launched_process_count - remaining_owned_process_count))"
    echo "remaining_owned_process_count=$remaining_owned_process_count"
    echo "cleanup_pass=$cleanup_pass"
    echo
    ps -eo pid,ppid,pgid,stat,cmd | grep -E "bimodal_|ros2 launch|rviz2" | grep -v grep || true
  } >> "$report"
  pids=()
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

quick_visual_topics_check() {
  local report="$1"
  local topics
  local ready=PASS
  topics=$(timeout 8 ros2 topic list --no-daemon 2>/dev/null || true)
  {
    echo "# P3B Visual Topics Report"
    echo
    echo "sample_time=$(date -Is)"
    for topic in /tf /tf_static /bimodal/tf_guard_status /bimodal/points /bimodal/map_3d /bimodal/esdf \
      /bimodal/coverage_markers /bimodal/map_status_marker /bimodal/robot_marker \
      /air/candidate_markers /air/selected_goal_marker /ground/frontier_candidates \
      /bimodal/active_mode /bimodal/active_path /bimodal/executed_path /bimodal/executor_marker; do
      if echo "$topics" | grep -qx "$topic"; then echo "$topic=PASS"; else echo "$topic=FAIL"; ready=FAIL; fi
    done
    echo "VISUAL_TOPICS_READY_FOR_RVIZ=$ready"
  } > "$report"
}

launch_validation() {
  local phase_dir="$LOG/wrapper"
  setsid ros2 launch bimodal_map_bringup p1c_real_sensor_pointcloud_input.launch.py \
    e2e_log_dir:="$phase_dir" sensor_input_mode:=external_pointcloud input_topic:="$INPUT_TOPIC" \
    enable_synthetic_pointcloud:=true backend_mode:=octomap_style_voxel \
    mode_switch_period_sec:="$MODE_SWITCH_PERIOD" > "$phase_dir/map_runtime.log" 2>&1 &
  pids+=($!)
  launched_process_count=$((launched_process_count + 1))
  sleep 5
  setsid ros2 launch bimodal_air_bringup air_baseline.launch.py planner_mode:="$AIR_MODE" \
    config_file:="$AIR_CONFIG" > "$phase_dir/air_runtime.log" 2>&1 &
  pids+=($!)
  launched_process_count=$((launched_process_count + 1))
  setsid ros2 launch bimodal_ground_bringup ground_baseline.launch.py planner_mode:="$GROUND_MODE" \
    config_file:="$GROUND_CONFIG" > "$phase_dir/ground_runtime.log" 2>&1 &
  pids+=($!)
  launched_process_count=$((launched_process_count + 1))

  timeout 45 bash "$ROOT/scripts/check_rviz_tf_ready.sh" --wait 20 > "$phase_dir/tf_snapshot.txt" 2>&1 || true
  quick_visual_topics_check "$phase_dir/visual_topics_report.md"
  timeout 6 ros2 topic list --no-daemon > "$phase_dir/topic_snapshot_start.txt" 2>&1 || true
  timeout 6 ros2 node list --no-daemon > "$phase_dir/node_snapshot_start.txt" 2>&1 || true
  capture_topic_once /bimodal/points "$phase_dir/bimodal_points_once.txt" --field header
  capture_topic_once /bimodal/map_3d "$phase_dir/map3d_once.txt" --field header
  capture_topic_once /air/planner_status "$phase_dir/air_status_once.txt" --field data
  capture_topic_once /ground/planner_status "$phase_dir/ground_status_once.txt" --field data
  capture_topic_once /bimodal/active_path "$phase_dir/active_path_once.txt"
  capture_topic_once /bimodal/executed_path "$phase_dir/executed_path_once.txt"

  end_time=$((SECONDS + DURATION))
  sample_idx=0
  while [ "$SECONDS" -lt "$end_time" ]; do
    echo "sample_index=$sample_idx sample_time=$(date -Is)" >> "$phase_dir/map_metrics.log"
    capture_string_data /bimodal/map_metrics "$phase_dir/map_metrics.log"
    echo "sample_index=$sample_idx sample_time=$(date -Is)" >> "$phase_dir/sensor_input_status.log"
    capture_string_data /bimodal/sensor_input_status "$phase_dir/sensor_input_status.log"
    echo "sample_index=$sample_idx sample_time=$(date -Is)" >> "$phase_dir/air_planner_status.log"
    capture_string_data /air/planner_status "$phase_dir/air_planner_status.log"
    echo "sample_index=$sample_idx sample_time=$(date -Is)" >> "$phase_dir/ground_planner_status.log"
    capture_string_data /ground/planner_status "$phase_dir/ground_planner_status.log"
    echo "sample_index=$sample_idx sample_time=$(date -Is)" >> "$phase_dir/active_mode.log"
    capture_string_data /bimodal/active_mode "$phase_dir/active_mode.log"
    capture_topic_once /bimodal/active_path "$phase_dir/samples/${sample_idx}_active_path.txt"
    capture_topic_once /bimodal/executed_path "$phase_dir/samples/${sample_idx}_executed_path.txt"
    capture_topic_once /bimodal/fake_executor_status "$phase_dir/samples/${sample_idx}_executor_status.txt" --field data
    sample_idx=$((sample_idx + 1))
    sleep 10
  done

  timeout 15 ros2 topic hz /bimodal/points --window 20 > "$phase_dir/points_rate.txt" 2>&1 || true
  timeout 6 ros2 topic list --no-daemon > "$phase_dir/topic_snapshot_end.txt" 2>&1 || true
  timeout 6 ros2 node list --no-daemon > "$phase_dir/node_snapshot_end.txt" 2>&1 || true
  cat "$phase_dir"/samples/*_active_path.txt > "$phase_dir/active_path.log" 2>/dev/null || true
  cat "$phase_dir"/samples/*_executed_path.txt > "$phase_dir/executed_path.log" 2>/dev/null || true
  set +e
  ros2 topic list 2>/dev/null | grep -E "^/cmd_vel$|^/mavros/|^/fmu/|^/actuator/|^/offboard_control_mode$|^/trajectory_setpoint$" > "$phase_dir/no_real_control_topic_check.txt"
  set -e
  cleanup_pids
}

if [ "$build_rc" -eq 0 ]; then
  launch_validation
else
  echo "Build failed; validation not launched." > "$LOG/wrapper/no_real_control_topic_check.txt"
fi

cp "$LOG/wrapper/topic_snapshot_start.txt" "$LOG/p3b_topic_snapshot_start.txt" 2>/dev/null || true
cp "$LOG/wrapper/topic_snapshot_end.txt" "$LOG/p3b_topic_snapshot_end.txt" 2>/dev/null || true
cp "$LOG/wrapper/node_snapshot_start.txt" "$LOG/p3b_node_snapshot_start.txt" 2>/dev/null || true
cp "$LOG/wrapper/node_snapshot_end.txt" "$LOG/p3b_node_snapshot_end.txt" 2>/dev/null || true
cp "$LOG/wrapper/tf_snapshot.txt" "$LOG/p3b_tf_snapshot.txt" 2>/dev/null || true
cp "$LOG/wrapper/air_planner_status.log" "$LOG/p3b_air_planner_status.log" 2>/dev/null || true
cp "$LOG/wrapper/ground_planner_status.log" "$LOG/p3b_ground_planner_status.log" 2>/dev/null || true
{ echo "# P3B Planner Wrapper Status"; cat "$LOG/p3b_air_planner_status.log" "$LOG/p3b_ground_planner_status.log" 2>/dev/null || true; } > "$LOG/p3b_planner_wrapper_status.log"
cp "$LOG/wrapper/map_metrics.log" "$LOG/p3b_map_metrics.log" 2>/dev/null || true
cp "$LOG/wrapper/active_path.log" "$LOG/p3b_active_path.log" 2>/dev/null || true
cp "$LOG/wrapper/executed_path.log" "$LOG/p3b_executed_path.log" 2>/dev/null || true
cp "$LOG/wrapper/no_real_control_topic_check.txt" "$LOG/p3b_no_real_control_topic_check.txt" 2>/dev/null || true

python3 - "$LOG" "$DURATION" "$AIR_MODE" "$GROUND_MODE" "$INPUT_TOPIC" "$build_rc" "$GIT_BRANCH" "$GIT_COMMIT_BEFORE" <<'PY'
import csv
import re
import sys
from pathlib import Path

log = Path(sys.argv[1])
duration = float(sys.argv[2])
air_mode_req = sys.argv[3]
ground_mode_req = sys.argv[4]
input_topic = sys.argv[5]
build_rc = int(sys.argv[6])
git_branch = sys.argv[7]
git_commit_before = sys.argv[8]
wrapper = log / 'wrapper'

def read(path):
    p = Path(path)
    return p.read_text(errors='ignore') if p.exists() else ''

def rel(path):
    return read(log / path)

def wrel(path):
    return read(wrapper / path)

def rows(path):
    p = wrapper / path
    if not p.exists():
        return []
    with p.open(newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))

def rv(text, key, default='FAIL'):
    for line in text.splitlines():
        if line.startswith(key + '='):
            return line.split('=', 1)[1].strip()
    return default

def vals(text, key):
    return [m.group(1).strip('",') for m in re.finditer(rf'{re.escape(key)}=(\S+)', text)]

def fvals(text, key):
    out = []
    for v in vals(text, key):
        try:
            out.append(float(v.strip('(),')))
        except Exception:
            pass
    return out

def last(text, key, default=0.0):
    v = fvals(text, key)
    return v[-1] if v else default

def first(text, key, default=0.0):
    v = fvals(text, key)
    return v[0] if v else default

def maxv(text, key, default=0.0):
    v = fvals(text, key)
    return max(v) if v else default

def avgv(text, key, default=0.0):
    v = fvals(text, key)
    return sum(v) / len(v) if v else default

def pf(x):
    return 'PASS' if x else 'FAIL'

def avg_rate(text):
    m = re.search(r'average rate: ([0-9.]+)', text)
    return float(m.group(1)) if m else 0.0

build = rel('build_report.md')
air_text = wrel('air_planner_status.log')
ground_text = wrel('ground_planner_status.log')
map_text = wrel('map_metrics.log')
sensor_text = wrel('sensor_input_status.log')
tf_text = wrel('tf_snapshot.txt')
visual_text = wrel('visual_topics_report.md')
node_text = wrel('node_snapshot_end.txt')
topic_text = wrel('topic_snapshot_end.txt')
active_path_text = wrel('active_path.log')
executed_path_text = wrel('executed_path.log')
mode_text = wrel('active_mode.log')
forbidden_text = wrel('no_real_control_topic_check.txt').strip()
cleanup_text = rel('p3b_process_cleanup_check.txt')

map_rows = rows('p2e_octomap_map_metrics.csv')
odom_rows = rows('e2e_odom.csv')
path_rows = rows('e2e_paths.csv')
mode_rows = rows('e2e_mode.csv')

occ_vals = [float(r.get('occupied_voxel_count') or 0.0) for r in map_rows] or fvals(map_text, 'occupied_voxel_count')
cov_vals = [float(r.get('coverage_proxy') or 0.0) for r in map_rows] or fvals(map_text, 'coverage_proxy')
occ_start = int(occ_vals[0]) if occ_vals else 0
occ_end = int(occ_vals[-1]) if occ_vals else 0
cov_start = float(cov_vals[0]) if cov_vals else 0.0
cov_end = float(cov_vals[-1]) if cov_vals else 0.0

external_delta = max(0, int(last(sensor_text, 'external_cloud_received_count', 0)) - int(first(sensor_text, 'external_cloud_received_count', 0)))
output_delta = max(0, int(last(sensor_text, 'output_cloud_published_count', 0)) - int(first(sensor_text, 'output_cloud_published_count', 0)))
points_rate = avg_rate(wrel('points_rate.txt')) or output_delta / max(duration, 1.0)

active_rows = [r for r in path_rows if r.get('source') == 'active']
active_hashes = [f'{r.get("pose_count")}:{round(float(r.get("path_length") or 0.0), 2)}' for r in active_rows]
active_unique_hash_count = len(set(active_hashes))
same_hash_max = 0.0
if active_rows:
    last_hash = None
    run_start = None
    last_time = None
    for r, h in zip(active_rows, active_hashes):
        ts = float(r.get('timestamp') or 0.0)
        if h != last_hash:
            if last_hash is not None and last_time is not None and run_start is not None:
                same_hash_max = max(same_hash_max, last_time - run_start)
            last_hash = h
            run_start = ts
        last_time = ts
    if last_time is not None and run_start is not None:
        same_hash_max = max(same_hash_max, last_time - run_start)

executed_path_count = executed_path_text.count('poses:') or len(odom_rows)
odom_total = max([float(r.get('total_distance') or 0.0) for r in odom_rows] or [0.0])
executed_length = odom_total
mode_values = [r.get('mode', '').strip() for r in mode_rows if r.get('mode', '').strip()]
mode_switch_count = sum(1 for a, b in zip(mode_values, mode_values[1:]) if a != b)

air_candidate_count = int(maxv(air_text, 'air_candidate_count', maxv(air_text, 'candidate_count', 0)))
air_valid_candidate_count = int(maxv(air_text, 'air_valid_candidate_count', maxv(air_text, 'valid_candidate_count', 0)))
air_candidate_sector_count = int(maxv(air_text, 'air_candidate_sector_count', 0))
air_candidate_z_level_count = int(maxv(air_text, 'air_candidate_z_level_count', 0))
air_selected_goal_count = int(maxv(air_text, 'air_selected_goal_count', 0))
air_goal_retire_count = int(maxv(air_text, 'air_goal_retire_count', 0))
air_low_gain_retire_count = int(maxv(air_text, 'air_low_gain_retire_count', 0))
air_stale_goal_retire_count = int(maxv(air_text, 'air_stale_goal_retire_count', 0))
air_path_feasible_count = int(maxv(air_text, 'air_path_feasible_count', 0))
air_path_infeasible_count = int(maxv(air_text, 'air_path_infeasible_count', 0))
air_collision_reject_count = int(maxv(air_text, 'air_collision_reject_count', 0))
air_z_reject_count = int(maxv(air_text, 'air_z_reject_count', 0))
air_clearance_reject_count = int(maxv(air_text, 'air_clearance_reject_count', 0))
air_boundary_reject_count = int(maxv(air_text, 'air_boundary_reject_count', 0))
air_goal_blacklist_count = int(maxv(air_text, 'air_goal_blacklist_count', 0))
air_repeat_goal_ratio = maxv(air_text, 'air_repeat_goal_ratio', 1.0)
air_frontier_gain_max = maxv(air_text, 'air_frontier_gain_max', maxv(air_text, 'octomap_frontier_gain', 0.0))
air_frontier_gain_avg = avgv(air_text, 'air_frontier_gain_avg', 0.0)
air_unknown_boundary_gain_max = maxv(air_text, 'air_unknown_boundary_gain_max', maxv(air_text, 'unknown_boundary_gain', 0.0))
air_unknown_boundary_gain_avg = avgv(air_text, 'air_unknown_boundary_gain_avg', 0.0)
air_expected_coverage_gain_max = maxv(air_text, 'air_expected_coverage_gain_max', maxv(air_text, 'coverage_gain_estimate', 0.0))
air_expected_coverage_gain_avg = avgv(air_text, 'air_expected_coverage_gain_avg', 0.0)
air_novelty_gain_max = maxv(air_text, 'air_novelty_gain_max', 0.0)
air_novelty_gain_avg = avgv(air_text, 'air_novelty_gain_avg', 0.0)
air_path_length_avg = maxv(air_text, 'air_path_length_avg', 0.0)
air_endpoint_to_goal_distance_avg = maxv(air_text, 'air_endpoint_to_goal_distance_avg', 0.0)
air_selected_goal_age_max_sec = maxv(air_text, 'air_selected_goal_age_max_sec', 0.0)
air_stale_path_max_sec = maxv(air_text, 'air_stale_path_max_sec', 0.0)
air_fallback_active = 'YES' if 'fallback_active=true' in air_text else 'NO'
air_failure_reason = vals(air_text, 'failure_reason')[-1] if vals(air_text, 'failure_reason') else 'NONE'
air_planner_mode = vals(air_text, 'air_planner_mode')[-1] if vals(air_text, 'air_planner_mode') else air_mode_req
air_quality_profile = vals(air_text, 'air_quality_profile')[-1] if vals(air_text, 'air_quality_profile') else 'UNKNOWN'

ground_candidate_count = int(maxv(ground_text, 'ground_candidate_count', maxv(ground_text, 'candidate_count', 0)))
ground_selected_goal_count = int(maxv(ground_text, 'ground_selected_goal_count', 0))
ground_fallback_active = 'YES' if 'fallback_active=true' in ground_text else 'NO'
ground_planner_mode = vals(ground_text, 'ground_planner_mode')[-1] if vals(ground_text, 'ground_planner_mode') else ground_mode_req

forbidden = [line for line in forbidden_text.splitlines() if line.strip()]
map_build = rv(build, 'map_build')
air_build = rv(build, 'air_build')
ground_build = rv(build, 'ground_build')
p3b_air_quality_build = rv(build, 'p3b_air_quality_build')

external_chain = pf(external_delta > 0 and output_delta > 0 and points_rate > 0)
bimodal_points = pf('/bimodal/points' in topic_text or output_delta > 0)
map_metrics_captured = pf(bool(map_rows) or 'coverage_proxy=' in map_text)
octomap_running = pf('octomap_pointcloud_backend_node' in node_text)
occ_inc = pf(occ_end > occ_start)
cov_inc = pf(cov_end > cov_start)
map_growth = pf(map_metrics_captured == 'PASS' and occ_inc == 'PASS' and cov_inc == 'PASS')
coverage_gain_per_meter = (cov_end - cov_start) / odom_total if odom_total > 0.0 else 0.0

active_mode_captured = pf(bool(mode_values) or bool(mode_text.strip()))
active_path_captured = pf(bool(active_rows) or 'poses:' in active_path_text)
executed_path_captured = pf(executed_path_count > 0 or 'poses:' in executed_path_text)
integrated_chain = pf(active_mode_captured == 'PASS' and active_path_captured == 'PASS' and executed_path_captured == 'PASS' and odom_total > 0.0 and executed_length > 0.0 and active_unique_hash_count > 0)
rviz_ready = rv(tf_text, 'RVIZ_FIXED_FRAME_READY')
visual_ready = rv(visual_text, 'VISUAL_TOPICS_READY_FOR_RVIZ')
tf_ready = pf('TF2_ECHO_MAP_BASE_LINK=PASS' in tf_text and 'TF_STATIC_MESSAGE_CAPTURED=PASS' in tf_text)
no_control = pf(not forbidden)
cleanup_pass = rv(cleanup_text, 'cleanup_pass')
remaining_owned = int(rv(cleanup_text, 'remaining_owned_process_count', '0') or 0)

p1c_guard = pf(external_chain == 'PASS' and bimodal_points == 'PASS')
p2f_guard = pf(p1c_guard == 'PASS' and map_growth == 'PASS' and integrated_chain == 'PASS')
p3a_p4a_guard = pf(p2f_guard == 'PASS' and ground_candidate_count > 0 and ground_selected_goal_count >= 1)
air_quality = pf(air_planner_mode == 'fuel_style_v0' and air_quality_profile == 'p3b_optimized' and air_candidate_count > 0 and air_valid_candidate_count > 0 and air_selected_goal_count >= 5 and air_path_feasible_count > 0 and air_repeat_goal_ratio <= 0.3 and air_fallback_active == 'NO')
ground_regression = pf(ground_planner_mode == 'ground_3d_frontier_v0' and ground_candidate_count > 0 and ground_selected_goal_count >= 1 and ground_fallback_active == 'NO')
air_node_running = pf('air_exploration_stub_node' in node_text)

required = [map_build, air_build, ground_build, p3b_air_quality_build,
            p3a_p4a_guard, p2f_guard, p1c_guard, external_chain, map_growth,
            integrated_chain, ground_regression, rviz_ready, visual_ready, tf_ready,
            no_control, cleanup_pass]
result = 'PASS' if all(v == 'PASS' for v in required) and air_quality == 'PASS' else 'FAIL'
if result == 'PASS' and (air_selected_goal_count <= 5 or active_unique_hash_count <= 5):
    result = 'PASS_PARTIAL'
    air_quality = 'PASS_PARTIAL'

failure = 'NONE'
reason = 'No remaining P3B failure.'
if result == 'FAIL':
    if no_control != 'PASS':
        failure, reason = 'SAFETY_FAIL', 'Forbidden real control topic appeared.'
    elif any(v != 'PASS' for v in [map_build, air_build, ground_build, p3b_air_quality_build]):
        failure, reason = 'BUILD_FAIL', 'Build failed.'
    elif air_candidate_count <= 0:
        failure, reason = 'AIR_NO_CANDIDATE', 'Air wrapper produced no candidates.'
    elif air_valid_candidate_count <= 0:
        failure, reason = 'AIR_NO_VALID_CANDIDATE', 'Air wrapper produced no valid candidates.'
    elif air_path_feasible_count <= 0:
        failure, reason = 'AIR_PATH_INFEASIBLE', 'Air wrapper produced no feasible paths.'
    elif map_growth != 'PASS':
        failure, reason = 'MAP_NOT_GROWING', 'Map metrics did not grow.'
    elif ground_regression != 'PASS':
        failure, reason = 'GROUND_REGRESSION', 'Ground wrapper regression failed.'
    elif active_path_captured != 'PASS':
        failure, reason = 'ACTIVE_PATH_FAIL', 'Active path was not captured.'
    elif executed_path_captured != 'PASS':
        failure, reason = 'EXECUTED_PATH_FAIL', 'Executed path was not captured.'
    elif tf_ready != 'PASS':
        failure, reason = 'TF_REGRESSION', 'TF readiness failed.'
    elif visual_ready != 'PASS':
        failure, reason = 'RVIZ_TOPIC_REGRESSION', 'Visual topics were not ready.'
    elif cleanup_pass != 'PASS':
        failure, reason = 'CLEANUP_FAIL', 'Owned processes were not cleaned.'
    else:
        failure, reason = 'AIR_QUALITY_REGRESSION', 'P3B Air quality acceptance thresholds were not met.'
elif result == 'PASS_PARTIAL':
    reason = 'No remaining hard P3B failure; quality is PASS_PARTIAL because selected-goal or active-path diversity did not exceed the P3A target.'

fields = {
    'project_stage': 'P3_AIR_REAL_PLANNER_QUALITY_OPTIMIZATION',
    'current_substage': 'P3B_AIR_FUEL_WRAPPER_QUALITY_OPTIMIZATION',
    'work_done': 'Optimized Air fuel_style_v0 candidate sampling, scoring, path feasibility diagnostics, and goal lifecycle while validating the external PointCloud2 chain',
    'work_not_done': 'No real control, no full FUEL/TARE import, no Ground quality rewrite, no nvblox, no RTAB-Map, no real octomap_server',
    'root': '/home/nuaa/ZHY/3DPlanner_FULL',
    'log_dir': str(log),
    'git_branch': git_branch,
    'git_commit_before': git_commit_before,
    'git_commit_after': 'NONE',
    'map_build': map_build,
    'air_build': air_build,
    'ground_build': ground_build,
    'p3b_air_quality_build': p3b_air_quality_build,
    'p3a_p4a_regression_guard': p3a_p4a_guard,
    'p2f_regression_guard': p2f_guard,
    'p1c_regression_guard': p1c_guard,
    'selected_input_mode': 'external_pointcloud',
    'selected_input_topic': input_topic,
    'bridge_output_topic': '/bimodal/points',
    'air_planner_mode': air_planner_mode,
    'air_quality_profile': air_quality_profile,
    'p3b_air_fuel_quality': air_quality,
    'air_wrapper_node_running': air_node_running,
    'air_candidate_count': str(air_candidate_count),
    'air_valid_candidate_count': str(air_valid_candidate_count),
    'air_candidate_sector_count': str(air_candidate_sector_count),
    'air_candidate_z_level_count': str(air_candidate_z_level_count),
    'air_selected_goal_count': str(air_selected_goal_count),
    'air_goal_retire_count': str(air_goal_retire_count),
    'air_low_gain_retire_count': str(air_low_gain_retire_count),
    'air_stale_goal_retire_count': str(air_stale_goal_retire_count),
    'air_goal_blacklist_count': str(air_goal_blacklist_count),
    'air_repeat_goal_ratio': f'{air_repeat_goal_ratio:.3f}',
    'air_path_feasible_count': str(air_path_feasible_count),
    'air_path_infeasible_count': str(air_path_infeasible_count),
    'air_collision_reject_count': str(air_collision_reject_count),
    'air_z_reject_count': str(air_z_reject_count),
    'air_clearance_reject_count': str(air_clearance_reject_count),
    'air_boundary_reject_count': str(air_boundary_reject_count),
    'air_frontier_gain_max': f'{air_frontier_gain_max:.3f}',
    'air_frontier_gain_avg': f'{air_frontier_gain_avg:.3f}',
    'air_unknown_boundary_gain_max': f'{air_unknown_boundary_gain_max:.3f}',
    'air_unknown_boundary_gain_avg': f'{air_unknown_boundary_gain_avg:.3f}',
    'air_expected_coverage_gain_max': f'{air_expected_coverage_gain_max:.3f}',
    'air_expected_coverage_gain_avg': f'{air_expected_coverage_gain_avg:.3f}',
    'air_novelty_gain_max': f'{air_novelty_gain_max:.3f}',
    'air_novelty_gain_avg': f'{air_novelty_gain_avg:.3f}',
    'air_path_length_avg': f'{air_path_length_avg:.3f}',
    'air_endpoint_to_goal_distance_avg': f'{air_endpoint_to_goal_distance_avg:.3f}',
    'air_selected_goal_age_max_sec': f'{air_selected_goal_age_max_sec:.3f}',
    'air_stale_path_max_sec': f'{air_stale_path_max_sec:.3f}',
    'air_fallback_active': air_fallback_active,
    'air_failure_reason': air_failure_reason,
    'ground_wrapper_regression': ground_regression,
    'ground_planner_mode': ground_planner_mode,
    'ground_candidate_count': str(ground_candidate_count),
    'ground_selected_goal_count': str(ground_selected_goal_count),
    'ground_fallback_active': ground_fallback_active,
    'duration_sec': f'{duration:.3f}',
    'external_pointcloud_chain': external_chain,
    'bimodal_points_captured': bimodal_points,
    'bimodal_points_rate_avg_hz': f'{points_rate:.3f}',
    'octomap_backend_node_running': octomap_running,
    'map_metrics_captured': map_metrics_captured,
    'occupied_voxel_count_start': str(occ_start),
    'occupied_voxel_count_end': str(occ_end),
    'occupied_voxel_count_delta': str(occ_end - occ_start),
    'occupied_voxel_count_increased': occ_inc,
    'coverage_proxy_start': f'{cov_start:.6f}',
    'coverage_proxy_end': f'{cov_end:.6f}',
    'coverage_proxy_delta': f'{cov_end - cov_start:.6f}',
    'coverage_proxy_increased': cov_inc,
    'coverage_gain_per_meter': f'{coverage_gain_per_meter:.9f}',
    'map_growth_with_p3b_air': map_growth,
    'active_mode_captured': active_mode_captured,
    'active_mode_switch_count': str(mode_switch_count),
    'active_path_captured': active_path_captured,
    'active_path_count': str(len(active_rows)),
    'active_path_update_count': str(len(active_rows)),
    'active_path_unique_hash_count': str(active_unique_hash_count),
    'active_path_same_hash_max_duration_sec': f'{same_hash_max:.3f}' if active_rows else 'UNKNOWN',
    'executed_path_captured': executed_path_captured,
    'executed_path_count': str(executed_path_count),
    'executed_path_length_m': f'{executed_length:.3f}',
    'odom_total_distance_m': f'{odom_total:.3f}',
    'integrated_p3b_chain': integrated_chain,
    'rviz_fixed_frame_ready': rviz_ready,
    'visual_topics_ready': visual_ready,
    'tf_tree_ready': tf_ready,
    'no_real_control_topic': no_control,
    'forbidden_topic_detected_count': str(len(forbidden)),
    'forbidden_topic_list': ','.join(forbidden) if forbidden else 'NONE',
    'cleanup_pass': cleanup_pass,
    'remaining_owned_process_count': str(remaining_owned),
    'P3B_AIR_FUEL_WRAPPER_QUALITY_OPTIMIZATION': result,
    'changed_files': 'PENDING',
    'local_commit_created': 'PENDING',
    'main_result_summary': 'P3B Air FUEL-style quality validation completed.' if result != 'FAIL' else 'P3B Air FUEL-style quality validation failed.',
    'remaining_failure_class': failure,
    'remaining_failure_reason': reason,
    'summary_report': str(log / 'p3b_air_fuel_quality_summary.md'),
    'debug_package': '/home/nuaa/ZHY/3DPlanner_FULL/latest_p3b_air_fuel_quality_package.tar.gz',
    'recommended_next_prompt_type': 'P4B_GROUND_3D_WRAPPER_QUALITY_OPTIMIZATION' if result != 'FAIL' else 'FIX_P3B_AIR_FUEL_QUALITY',
    'next_stage_explanation': 'Air wrapper quality is improved or safely bounded; next optimize the Ground 3D wrapper while preserving this shared-map baseline.' if result != 'FAIL' else 'Fix the P3B Air quality failure before moving to Ground quality or full demo.',
}

with (log / 'p3b_air_fuel_quality_summary.md').open('w', encoding='utf-8') as f:
    f.write('# P3B Air FUEL Wrapper Quality Summary\n\n')
    f.write('当前阶段：P3B_AIR_FUEL_WRAPPER_QUALITY_OPTIMIZATION。\n\n')
    f.write('本轮完成：Air fuel_style_v0 增加 P3B optimized profile，多半径/多方向/z 分层候选，可解释 score 分项，保守碰撞检查与 goal lifecycle retirement；Ground 保持 P4A regression。\n\n')
    f.write('本轮未做：未接真实飞控，未发布真实控制 topic，未大改 Ground，未直接导入完整 FUEL/TARE 模块。\n\n')
    f.write(f'Air quality：{air_quality}，candidate={air_candidate_count}，valid={air_valid_candidate_count}，selected_goal={air_selected_goal_count}，feasible_path={air_path_feasible_count}，repeat_ratio={air_repeat_goal_ratio:.3f}。\n\n')
    f.write(f'Path diversity：active_path_unique_hash_count={active_unique_hash_count}，active_path_same_hash_max_duration_sec={same_hash_max:.3f}。\n\n')
    f.write(f'Map growth：occupied {occ_start} -> {occ_end}，coverage {cov_start:.6f} -> {cov_end:.6f}，coverage_gain_per_meter={coverage_gain_per_meter:.9f}。\n\n')
    f.write(f'Safety：no_real_control_topic={no_control}，forbidden_topic_detected_count={len(forbidden)}。\n\n')
    f.write(f'Final：P3B_AIR_FUEL_WRAPPER_QUALITY_OPTIMIZATION={result}，remaining_failure_class={failure}。\n')

with (log / 'p3b_acceptance_report.md').open('w', encoding='utf-8') as f:
    f.write('# P3B Acceptance Report\n\n')
    for k, v in fields.items():
        f.write(f'{k}={v}\n')

print('B3D_P3B_AIR_FUEL_WRAPPER_QUALITY_SUMMARY')
for k, v in fields.items():
    print(f'{k}={v}')
PY

tar -czf "$ROOT/latest_p3b_air_fuel_quality_package.tar.gz" -C "$(dirname "$LOG")" "$(basename "$LOG")"
cat "$LOG/p3b_acceptance_report.md"
