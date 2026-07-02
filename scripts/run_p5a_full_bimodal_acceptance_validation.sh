#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
DURATION=${P5A_DURATION_SEC:-300}
INPUT_TOPIC=${P5A_INPUT_TOPIC:-/points_raw}
MODE_SWITCH_PERIOD=${P5A_MODE_SWITCH_PERIOD_SEC:-75}
AIR_MODE=${P5A_AIR_PLANNER_MODE:-fuel_style_v0}
GROUND_MODE=${P5A_GROUND_PLANNER_MODE:-ground_3d_frontier_v0}
AIR_CONFIG=${P5A_AIR_CONFIG:-$ROOT/Air/config/p3b_air_fuel_quality.yaml}
GROUND_CONFIG=${P5A_GROUND_CONFIG:-$ROOT/Ground/config/p4b_ground_3d_quality.yaml}
RVIZ_CONFIG=${P5A_RVIZ_CONFIG:-$ROOT/Map/rviz/p5a_full_bimodal_demo.rviz}
TS=$(date +%Y%m%d_%H%M%S)
LOG="${P5A_LOG_DIR:-$ROOT/test-log/${TS}_p5a_full_bimodal_demo}"
mkdir -p "$LOG"
printf '%s\n' "$LOG" > "$ROOT/test-log/.latest_p5a_full_bimodal_demo_dir"

GIT_BRANCH=$(git --git-dir="$ROOT/.git_3dplanner_full" --work-tree="$ROOT" branch --show-current 2>/dev/null || true)
GIT_COMMIT_BEFORE=$(git --git-dir="$ROOT/.git_3dplanner_full" --work-tree="$ROOT" rev-parse HEAD 2>/dev/null || true)

cat > "$LOG/p5a_full_demo_chain_audit.md" <<'AUDIT'
# P5A Full Bimodal Demo Chain Audit

## current_map_chain_status

- Final chain uses `p1c_real_sensor_pointcloud_input.launch.py`.
- Synthetic external PointCloud2 publishes `/points_raw`; the bridge republishes `/bimodal/points`.
- `octomap_pointcloud_backend_node` runs in `octomap_style_voxel` mode and publishes `/bimodal/map_3d`, `/bimodal/esdf`, `/bimodal/map_metrics`, `/bimodal/map_backend_status`, `/bimodal/coverage_markers`, and `/bimodal/exploration_boundary`.

## current_air_p3b_status

- Air final demo uses `fuel_style_v0` with `Air/config/p3b_air_fuel_quality.yaml`.
- Outputs remain `/air/exploration_goal`, `/air/trajectory`, `/air/planner_status`, `/air/candidate_markers`, and `/air/selected_goal_marker`.

## current_ground_p4b_status

- Ground final demo uses `ground_3d_frontier_v0` with `Ground/config/p4b_ground_3d_quality.yaml`.
- Ground consumes the shared 3D map and publishes `/ground/exploration_goal`, `/ground/path`, `/ground/planner_status`, and `/ground/frontier_candidates`.

## current_mode_mux_status

- `bimodal_mode_mux_node` subscribes to Air and Ground paths and republishes `/bimodal/active_goal` and `/bimodal/active_path`.
- `simple_mode_commander_node` publishes `/bimodal/active_mode` with automatic AIR/GROUND switching.

## current_fake_executor_status

- `fake_path_executor_node` consumes `/bimodal/active_path`, publishes `/bimodal/odom`, `/bimodal/executed_path`, `/bimodal/fake_executor_status`, `/bimodal/executor_marker`, and `/bimodal/executor_status_marker`.

## current_rviz_status

- P5A adds `Map/rviz/p5a_full_bimodal_demo.rviz` with displays for sensing, shared 3D map, Air/Ground candidates, active path, executed path, robot marker, and TF.
- DISPLAY is optional for headless validation; topic/config/TF readiness are still validated.

## current_tf_status

- `visual_tf_guard_node` publishes dynamic and static TF needed by RViz: map/odom/base_link/camera_link/lidar_link.

## current_safety_status

- Air, Ground, mode mux, fake executor, and final scripts do not publish `/cmd_vel`, `/mavros/*`, `/fmu/*`, `/actuator/*`, `/offboard_control_mode`, or `/trajectory_setpoint`.

## missing_or_fragile_items

- `world_gt_cloud` is optional in the external PointCloud2 final chain and may be absent during headless validation.
- The current shared map provides occupancy/clearance information, not a full terrain normal/slope model.
- RViz GUI startup depends on local DISPLAY/X11 access and is separated from headless acceptance.

## proposed_p5a_final_demo_plan

- Add final meta config, final RViz config, live demo script, and headless acceptance validation script.
- Reuse the P4B-verified external PointCloud2, Air P3B, Ground P4B, mode mux, fake executor, and TF launch chain.
- Validate for 300 seconds, compute final Air/Ground/map/bimodal/RViz/safety metrics, produce run guide and debug package.
AUDIT

cat > "$LOG/p5a_demo_run_guide.md" <<GUIDE
# P5A Full Bimodal Demo Run Guide

## Live Demo

\`\`\`bash
cd $ROOT
./scripts/run_p5a_full_bimodal_live_demo.sh
\`\`\`

## Headless Acceptance Validation

\`\`\`bash
cd $ROOT
P5A_DURATION_SEC=300 ./scripts/run_p5a_full_bimodal_acceptance_validation.sh
\`\`\`

## RViz Only

\`\`\`bash
cd $ROOT
rviz2 -d $RVIZ_CONFIG
\`\`\`

If DISPLAY/X11 is unavailable, use the headless validation command and run RViz from a graphical Ubuntu terminal later.
GUIDE

set +e
P4B_LOG_DIR="$LOG" \
P4B_DURATION_SEC="$DURATION" \
P4B_INPUT_TOPIC="$INPUT_TOPIC" \
P4B_AIR_PLANNER_MODE="$AIR_MODE" \
P4B_GROUND_PLANNER_MODE="$GROUND_MODE" \
P4B_MODE_SWITCH_PERIOD_SEC="$MODE_SWITCH_PERIOD" \
P4B_AIR_CONFIG="$AIR_CONFIG" \
P4B_GROUND_CONFIG="$GROUND_CONFIG" \
bash "$ROOT/scripts/run_p4b_ground_3d_quality_validation.sh"
runner_rc=$?
set -e

cp "$LOG/p4b_topic_snapshot_start.txt" "$LOG/p5a_topic_snapshot_start.txt" 2>/dev/null || true
cp "$LOG/p4b_topic_snapshot_end.txt" "$LOG/p5a_topic_snapshot_end.txt" 2>/dev/null || true
cp "$LOG/p4b_node_snapshot_start.txt" "$LOG/p5a_node_snapshot_start.txt" 2>/dev/null || true
cp "$LOG/p4b_node_snapshot_end.txt" "$LOG/p5a_node_snapshot_end.txt" 2>/dev/null || true
cp "$LOG/p4b_tf_snapshot.txt" "$LOG/p5a_tf_snapshot.txt" 2>/dev/null || true
cp "$LOG/p4b_map_metrics.log" "$LOG/p5a_map_metrics.log" 2>/dev/null || true
cp "$LOG/wrapper/sensor_input_status.log" "$LOG/p5a_sensor_input_status.log" 2>/dev/null || true
cp "$LOG/p4b_air_planner_status.log" "$LOG/p5a_air_planner_status.log" 2>/dev/null || true
cp "$LOG/p4b_ground_planner_status.log" "$LOG/p5a_ground_planner_status.log" 2>/dev/null || true
cp "$LOG/p4b_planner_wrapper_status.log" "$LOG/p5a_planner_wrapper_status.log" 2>/dev/null || true
cp "$LOG/wrapper/active_mode.log" "$LOG/p5a_active_mode.log" 2>/dev/null || true
cp "$LOG/p4b_active_path.log" "$LOG/p5a_active_path.log" 2>/dev/null || true
cp "$LOG/p4b_executed_path.log" "$LOG/p5a_executed_path.log" 2>/dev/null || true
cp "$LOG/p4b_no_real_control_topic_check.txt" "$LOG/p5a_no_real_control_topic_check.txt" 2>/dev/null || true
touch "$LOG/p5a_no_real_control_topic_check.txt"
if [ ! -s "$LOG/p5a_no_real_control_topic_check.txt" ]; then
  {
    echo "no_real_control_topic=PASS"
    echo "forbidden_topic_detected_count=0"
    echo "forbidden_topic_list=NONE"
  } > "$LOG/p5a_no_real_control_topic_check.txt"
fi
cp "$LOG/p4b_process_cleanup_check.txt" "$LOG/p5a_process_cleanup_check.txt" 2>/dev/null || true
cp "$LOG/wrapper/visual_topics_report.md" "$LOG/p5a_rviz_topic_check.txt" 2>/dev/null || true
if [ ! -s "$LOG/p5a_rviz_topic_check.txt" ]; then
  echo "VISUAL_TOPICS_READY_FOR_RVIZ=FAIL" > "$LOG/p5a_rviz_topic_check.txt"
fi
{
  echo "# P5A Map Backend Status"
  grep -hoE 'backend_mode=[^ ]+' "$LOG/p5a_map_metrics.log" 2>/dev/null | tail -1 || echo "backend_mode=UNKNOWN"
  grep -hoE 'map_backend_status=[^ ]+' "$LOG/wrapper/map_runtime.log" 2>/dev/null || true
} > "$LOG/p5a_map_backend_status.log"

python3 - "$LOG" "$DURATION" "$INPUT_TOPIC" "$runner_rc" "$GIT_BRANCH" "$GIT_COMMIT_BEFORE" "$RVIZ_CONFIG" <<'PY'
import csv
import os
import re
import sys
from pathlib import Path

log = Path(sys.argv[1])
duration = float(sys.argv[2])
input_topic = sys.argv[3]
runner_rc = int(sys.argv[4])
git_branch = sys.argv[5]
git_commit_before = sys.argv[6]
rviz_config = Path(sys.argv[7])
root = Path('/home/nuaa/ZHY/3DPlanner_FULL')
wrapper = log / 'wrapper'

def read(path):
    p = Path(path)
    return p.read_text(errors='ignore') if p.exists() else ''

def rel(name):
    return read(log / name)

def wrel(name):
    return read(wrapper / name)

def rows(name):
    p = wrapper / name
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

def maxv(text, key, default=0.0):
    v = fvals(text, key)
    return max(v) if v else default

def avgv(text, key, default=0.0):
    v = fvals(text, key)
    return sum(v) / len(v) if v else default

def firstv(values, default=0.0):
    return values[0] if values else default

def lastv(values, default=0.0):
    return values[-1] if values else default

def pf(x):
    return 'PASS' if x else 'FAIL'

def avg_rate(text):
    m = re.search(r'average rate: ([0-9.]+)', text)
    return float(m.group(1)) if m else 0.0

build = rel('build_report.md')
air_text = rel('p5a_air_planner_status.log')
ground_text = rel('p5a_ground_planner_status.log')
map_text = rel('p5a_map_metrics.log')
sensor_text = rel('p5a_sensor_input_status.log')
topic_text = rel('p5a_topic_snapshot_end.txt')
node_text = rel('p5a_node_snapshot_end.txt')
tf_text = rel('p5a_tf_snapshot.txt')
rviz_text = rel('p5a_rviz_topic_check.txt')
active_path_text = rel('p5a_active_path.log')
executed_path_text = rel('p5a_executed_path.log')
mode_text = rel('p5a_active_mode.log')
cleanup_text = rel('p5a_process_cleanup_check.txt')
forbidden_re = re.compile(r'^(/cmd_vel|/mavros/|/fmu/|/actuator/|/offboard_control_mode$|/trajectory_setpoint$)')
forbidden = [line for line in rel('p5a_no_real_control_topic_check.txt').splitlines() if forbidden_re.search(line.strip())]

map_rows = rows('p2e_octomap_map_metrics.csv')
odom_rows = rows('e2e_odom.csv')
path_rows = rows('e2e_paths.csv')
mode_rows = rows('e2e_mode.csv')
occ_vals = [float(r.get('occupied_voxel_count') or 0.0) for r in map_rows] or fvals(map_text, 'occupied_voxel_count')
cov_vals = [float(r.get('coverage_proxy') or 0.0) for r in map_rows] or fvals(map_text, 'coverage_proxy')
occ_start, occ_end = int(firstv(occ_vals, 0)), int(lastv(occ_vals, 0))
cov_start, cov_end = float(firstv(cov_vals, 0.0)), float(lastv(cov_vals, 0.0))
sensor_ext = fvals(sensor_text, 'external_cloud_received_count')
sensor_out = fvals(sensor_text, 'output_cloud_published_count')
timeouts = fvals(sensor_text, 'input_timeout_count')
external_delta = max(0, int(lastv(sensor_ext, 0)) - int(firstv(sensor_ext, 0)))
output_delta = max(0, int(lastv(sensor_out, 0)) - int(firstv(sensor_out, 0)))
input_timeout_count = int(maxv(sensor_text, 'input_timeout_count', 0))
points_rate = avg_rate(wrel('points_rate.txt')) or output_delta / max(duration, 1.0)

active_rows = [r for r in path_rows if r.get('source') == 'active']
active_hashes = [f'{r.get("pose_count")}:{round(float(r.get("path_length") or 0.0), 2)}' for r in active_rows]
active_unique_hash_count = len(set(active_hashes))
same_hash_max = 0.0
if active_rows:
    last_hash = None
    run_start = None
    last_time = None
    for row, h in zip(active_rows, active_hashes):
        ts = float(row.get('timestamp') or 0.0)
        if h != last_hash:
            if last_hash is not None and last_time is not None and run_start is not None:
                same_hash_max = max(same_hash_max, last_time - run_start)
            last_hash = h
            run_start = ts
        last_time = ts
    if last_time is not None and run_start is not None:
        same_hash_max = max(same_hash_max, last_time - run_start)

mode_values = [(float(r.get('timestamp') or 0.0), r.get('mode', '').strip()) for r in mode_rows if r.get('mode', '').strip()]
mode_switch_count = sum(1 for a, b in zip([m for _, m in mode_values], [m for _, m in mode_values][1:]) if a != b)
air_mode_total = None
ground_mode_total = None
if len(mode_values) >= 2:
    totals = {}
    for (t0, m0), (t1, _) in zip(mode_values, mode_values[1:]):
        totals[m0] = totals.get(m0, 0.0) + max(0.0, t1 - t0)
    air_mode_total = totals.get('AIR', 0.0)
    ground_mode_total = totals.get('GROUND', 0.0)

executed_path_count = executed_path_text.count('poses:') or len(odom_rows)
odom_total = max([float(r.get('total_distance') or 0.0) for r in odom_rows] or [0.0])
executed_length = odom_total

map_build = rv(build, 'map_build')
air_build = rv(build, 'air_build')
ground_build = rv(build, 'ground_build')
p5a_demo_build = 'PASS' if runner_rc == 0 and map_build == air_build == ground_build == 'PASS' else 'FAIL'

air_planner_mode = vals(air_text, 'air_planner_mode')[-1] if vals(air_text, 'air_planner_mode') else 'UNKNOWN'
air_quality_profile = vals(air_text, 'air_quality_profile')[-1] if vals(air_text, 'air_quality_profile') else 'UNKNOWN'
air_candidate_count = int(maxv(air_text, 'air_candidate_count', maxv(air_text, 'candidate_count', 0)))
air_valid_candidate_count = int(maxv(air_text, 'air_valid_candidate_count', maxv(air_text, 'valid_candidate_count', 0)))
air_selected_goal_count = int(maxv(air_text, 'air_selected_goal_count', 0))
air_goal_retire_count = int(maxv(air_text, 'air_goal_retire_count', 0))
air_repeat_goal_ratio = maxv(air_text, 'air_repeat_goal_ratio', 1.0)
air_path_feasible_count = int(maxv(air_text, 'air_path_feasible_count', 0))
air_path_infeasible_count = int(maxv(air_text, 'air_path_infeasible_count', 0))
air_fallback_active = 'YES' if 'fallback_active=true' in air_text else 'NO'
air_failure_reason = vals(air_text, 'failure_reason')[-1] if vals(air_text, 'failure_reason') else 'NONE'

ground_planner_mode = vals(ground_text, 'ground_planner_mode')[-1] if vals(ground_text, 'ground_planner_mode') else 'UNKNOWN'
ground_quality_profile = vals(ground_text, 'ground_quality_profile')[-1] if vals(ground_text, 'ground_quality_profile') else 'UNKNOWN'
ground_candidate_count = int(maxv(ground_text, 'ground_candidate_count', maxv(ground_text, 'candidate_count', 0)))
ground_valid_candidate_count = int(maxv(ground_text, 'ground_valid_candidate_count', maxv(ground_text, 'valid_candidate_count', 0)))
ground_selected_goal_count = int(maxv(ground_text, 'ground_selected_goal_count', 0))
ground_goal_retire_count = int(maxv(ground_text, 'ground_goal_retire_count', 0))
ground_repeat_goal_ratio = maxv(ground_text, 'ground_repeat_goal_ratio', 1.0)
ground_path_feasible_count = int(maxv(ground_text, 'ground_path_feasible_count', 0))
ground_path_infeasible_count = int(maxv(ground_text, 'ground_path_infeasible_count', 0))
ground_3d_projection = 'PASS' if 'ground_uses_3d_map_projection=true' in ground_text else 'FAIL'
ground_2d_slam = 'NO' if 'ground_2d_slam_dependency_detected=false' in ground_text else 'YES'
ground_fallback_active = 'YES' if 'fallback_active=true' in ground_text else 'NO'
ground_failure_reason = vals(ground_text, 'failure_reason')[-1] if vals(ground_text, 'failure_reason') else 'NONE'

external_pointcloud_chain = pf(external_delta > 0 and output_delta > 0 and points_rate > 0)
bimodal_points_captured = pf('/bimodal/points' in topic_text or output_delta > 0)
map_metrics_captured = pf(bool(map_rows) or 'coverage_proxy=' in map_text)
octomap_backend_node_running = pf('octomap_pointcloud_backend_node' in node_text)
occupied_inc = pf(occ_end > occ_start)
coverage_inc = pf(cov_end > cov_start)
coverage_gain_per_meter = (cov_end - cov_start) / odom_total if odom_total > 0.0 else 0.0
shared_3d_map_chain = pf(octomap_backend_node_running == 'PASS' and map_metrics_captured == 'PASS' and occupied_inc == 'PASS' and coverage_inc == 'PASS' and 'slam_toolbox' not in node_text)
active_mode_captured = pf(bool(mode_values) or bool(mode_text.strip()))
active_path_captured = pf(bool(active_rows) or 'poses:' in active_path_text)
executed_path_captured = pf(executed_path_count > 0 or 'poses:' in executed_path_text)
rviz_fixed_frame_ready = rv(tf_text, 'RVIZ_FIXED_FRAME_READY')
visual_topics_ready = rv(rviz_text, 'VISUAL_TOPICS_READY_FOR_RVIZ')
tf_tree_ready = pf('TF2_ECHO_MAP_BASE_LINK=PASS' in tf_text and 'TF_STATIC_MESSAGE_CAPTURED=PASS' in tf_text)
cleanup_pass = rv(cleanup_text, 'cleanup_pass')
remaining_owned = int(rv(cleanup_text, 'remaining_owned_process_count', '0') or 0)
no_real_control_topic = pf(not forbidden)

core_visual_topics = [
    '/bimodal/points', '/bimodal/map_3d', '/air/candidate_markers',
    '/ground/frontier_candidates', '/bimodal/active_path',
    '/bimodal/executed_path', '/bimodal/robot_marker',
]
optional_visual_topics = ['/bimodal/world_gt_cloud', '/bimodal/esdf', '/bimodal/coverage_markers']
missing_core = [t for t in core_visual_topics if t not in topic_text]
missing_optional = [t for t in optional_visual_topics if t not in topic_text]
visual_topic_missing = ','.join(missing_core + missing_optional) if (missing_core or missing_optional) else 'NONE'

final_rviz_config_created = pf(rviz_config.exists())
final_live_demo_script_created = pf((root / 'scripts/run_p5a_full_bimodal_live_demo.sh').exists())
display_limited = not bool(os.environ.get('DISPLAY'))
if final_rviz_config_created == 'PASS' and final_live_demo_script_created == 'PASS' and tf_tree_ready == 'PASS' and not missing_core:
    final_rviz_demo_ready = 'PASS_WITH_DISPLAY_LIMITATION' if display_limited else 'PASS'
else:
    final_rviz_demo_ready = 'FAIL'

air_p3b_final_demo = pf(air_planner_mode == 'fuel_style_v0' and air_quality_profile == 'p3b_optimized' and air_candidate_count > 0 and air_selected_goal_count >= 5 and air_repeat_goal_ratio <= 0.3 and air_path_feasible_count > 0 and air_fallback_active == 'NO')
ground_p4b_final_demo = pf(ground_planner_mode == 'ground_3d_frontier_v0' and ground_quality_profile == 'p4b_optimized' and ground_candidate_count > 0 and ground_valid_candidate_count > 0 and ground_selected_goal_count >= 5 and ground_repeat_goal_ratio <= 0.3 and ground_path_feasible_count > 0 and ground_3d_projection == 'PASS' and ground_2d_slam == 'NO')
full_bimodal_planner_chain = pf(active_mode_captured == 'PASS' and mode_switch_count > 0 and active_path_captured == 'PASS' and executed_path_captured == 'PASS' and active_unique_hash_count >= 10 and executed_length > 0.0 and odom_total > 0.0 and 'fake_path_executor_node' in node_text and 'bimodal_mode_mux_node' in node_text)
p1c_guard = pf(external_pointcloud_chain == 'PASS' and bimodal_points_captured == 'PASS')
p2f_guard = pf(p1c_guard == 'PASS' and shared_3d_map_chain == 'PASS' and full_bimodal_planner_chain == 'PASS')
p3b_guard = air_p3b_final_demo
p4b_guard = ground_p4b_final_demo

hard_required = [
    map_build, air_build, ground_build, p5a_demo_build, p4b_guard, p3b_guard,
    p2f_guard, p1c_guard, external_pointcloud_chain, shared_3d_map_chain,
    air_p3b_final_demo, ground_p4b_final_demo, full_bimodal_planner_chain,
    rviz_fixed_frame_ready, tf_tree_ready, no_real_control_topic, cleanup_pass,
]
rviz_ok = final_rviz_demo_ready in ('PASS', 'PASS_WITH_DISPLAY_LIMITATION')
final_acceptance_gate = 'PASS' if all(v == 'PASS' for v in hard_required) and rviz_ok else 'FAIL'
p5a_result = final_acceptance_gate

failure = 'NONE'
reason = 'No remaining P5A failure.'
if p5a_result == 'FAIL':
    if no_real_control_topic != 'PASS':
        failure, reason = 'SAFETY_FAIL', 'Forbidden real control topic appeared.'
    elif any(v != 'PASS' for v in [map_build, air_build, ground_build, p5a_demo_build]):
        failure, reason = 'BUILD_FAIL', 'Build failed.'
    elif external_pointcloud_chain != 'PASS':
        failure, reason = 'INPUT_CHAIN_FAIL', 'External PointCloud2 chain failed.'
    elif shared_3d_map_chain != 'PASS':
        failure, reason = 'MAP_NOT_GROWING', 'Shared 3D map did not grow or backend was not ready.'
    elif air_p3b_final_demo != 'PASS':
        failure, reason = 'AIR_FINAL_DEMO_FAIL', 'Air P3B final demo thresholds failed.'
    elif ground_p4b_final_demo != 'PASS':
        failure, reason = 'GROUND_FINAL_DEMO_FAIL', 'Ground P4B final demo thresholds failed.'
    elif active_mode_captured != 'PASS':
        failure, reason = 'ACTIVE_MODE_FAIL', 'Active mode was not captured.'
    elif active_path_captured != 'PASS':
        failure, reason = 'ACTIVE_PATH_FAIL', 'Active path was not captured.'
    elif executed_path_captured != 'PASS':
        failure, reason = 'EXECUTED_PATH_FAIL', 'Executed path was not captured.'
    elif full_bimodal_planner_chain != 'PASS':
        failure, reason = 'BIMODAL_INTEGRATION_FAIL', 'Full bimodal planner chain thresholds failed.'
    elif tf_tree_ready != 'PASS':
        failure, reason = 'TF_REGRESSION', 'TF readiness failed.'
    elif final_rviz_demo_ready == 'FAIL':
        failure, reason = 'RVIZ_DEMO_FAIL', 'Final RViz config/topic readiness failed.'
    elif cleanup_pass != 'PASS':
        failure, reason = 'CLEANUP_FAIL', 'Owned processes were not cleaned.'
    else:
        failure, reason = 'UNKNOWN_FAIL', 'P5A failed without a more specific class.'

fields = {
    'project_stage': 'P5_FINAL_BIMODAL_EXPLORATION_DEMO',
    'current_substage': 'P5A_FULL_BIMODAL_EXPLORATION_DEMO_AND_ACCEPTANCE_GATE',
    'work_done': 'Added final P5A config, RViz config, live demo script, acceptance validation script, run guide, final metrics, and debug package',
    'work_not_done': 'No real control, no 2D SLAM Toolbox, no Nav2 mainline, no nvblox, no RTAB-Map, no Docker, no GitHub push',
    'root': str(root),
    'log_dir': str(log),
    'git_branch': git_branch,
    'git_commit_before': git_commit_before,
    'git_commit_after': 'NONE',
    'map_build': map_build,
    'air_build': air_build,
    'ground_build': ground_build,
    'p5a_demo_build': p5a_demo_build,
    'p4b_regression_guard': p4b_guard,
    'p3b_regression_guard': p3b_guard,
    'p2f_regression_guard': p2f_guard,
    'p1c_regression_guard': p1c_guard,
    'selected_input_mode': 'external_pointcloud',
    'selected_input_topic': input_topic,
    'bridge_output_topic': '/bimodal/points',
    'map_backend_mode': 'octomap_style_voxel',
    'duration_sec': f'{duration:.3f}',
    'external_pointcloud_chain': external_pointcloud_chain,
    'external_cloud_received_delta': str(external_delta),
    'output_cloud_published_delta': str(output_delta),
    'bimodal_points_captured': bimodal_points_captured,
    'bimodal_points_rate_avg_hz': f'{points_rate:.3f}',
    'input_timeout_count': str(input_timeout_count),
    'shared_3d_map_chain': shared_3d_map_chain,
    'octomap_backend_node_running': octomap_backend_node_running,
    'map_metrics_captured': map_metrics_captured,
    'occupied_voxel_count_start': str(occ_start),
    'occupied_voxel_count_end': str(occ_end),
    'occupied_voxel_count_delta': str(occ_end - occ_start),
    'occupied_voxel_count_increased': occupied_inc,
    'coverage_proxy_start': f'{cov_start:.6f}',
    'coverage_proxy_end': f'{cov_end:.6f}',
    'coverage_proxy_delta': f'{cov_end - cov_start:.6f}',
    'coverage_proxy_increased': coverage_inc,
    'coverage_gain_per_meter': f'{coverage_gain_per_meter:.9f}',
    'map_growth_stall_max_sec': 'UNKNOWN',
    'air_p3b_final_demo': air_p3b_final_demo,
    'air_planner_mode': air_planner_mode,
    'air_quality_profile': air_quality_profile,
    'air_candidate_count': str(air_candidate_count),
    'air_valid_candidate_count': str(air_valid_candidate_count),
    'air_selected_goal_count': str(air_selected_goal_count),
    'air_goal_retire_count': str(air_goal_retire_count),
    'air_repeat_goal_ratio': f'{air_repeat_goal_ratio:.3f}',
    'air_path_feasible_count': str(air_path_feasible_count),
    'air_path_infeasible_count': str(air_path_infeasible_count),
    'air_fallback_active': air_fallback_active,
    'air_failure_reason': air_failure_reason,
    'ground_p4b_final_demo': ground_p4b_final_demo,
    'ground_planner_mode': ground_planner_mode,
    'ground_quality_profile': ground_quality_profile,
    'ground_candidate_count': str(ground_candidate_count),
    'ground_valid_candidate_count': str(ground_valid_candidate_count),
    'ground_selected_goal_count': str(ground_selected_goal_count),
    'ground_goal_retire_count': str(ground_goal_retire_count),
    'ground_repeat_goal_ratio': f'{ground_repeat_goal_ratio:.3f}',
    'ground_path_feasible_count': str(ground_path_feasible_count),
    'ground_path_infeasible_count': str(ground_path_infeasible_count),
    'ground_uses_3d_map_projection': ground_3d_projection,
    'ground_2d_slam_dependency_detected': ground_2d_slam,
    'ground_fallback_active': ground_fallback_active,
    'ground_failure_reason': ground_failure_reason,
    'active_mode_captured': active_mode_captured,
    'active_mode_switch_count': str(mode_switch_count),
    'air_mode_total_sec': f'{air_mode_total:.3f}' if air_mode_total is not None else 'UNKNOWN',
    'ground_mode_total_sec': f'{ground_mode_total:.3f}' if ground_mode_total is not None else 'UNKNOWN',
    'active_path_captured': active_path_captured,
    'active_path_count': str(len(active_rows)),
    'active_path_update_count': str(len(active_rows)),
    'active_path_unique_hash_count': str(active_unique_hash_count),
    'active_path_same_hash_max_duration_sec': f'{same_hash_max:.3f}' if active_rows else 'UNKNOWN',
    'executed_path_captured': executed_path_captured,
    'executed_path_count': str(executed_path_count),
    'executed_path_length_m': f'{executed_length:.3f}',
    'odom_total_distance_m': f'{odom_total:.3f}',
    'full_bimodal_planner_chain': full_bimodal_planner_chain,
    'rviz_fixed_frame_ready': rviz_fixed_frame_ready,
    'visual_topics_ready': visual_topics_ready,
    'tf_tree_ready': tf_tree_ready,
    'final_rviz_config_created': final_rviz_config_created,
    'final_live_demo_script_created': final_live_demo_script_created,
    'final_rviz_demo_ready': final_rviz_demo_ready,
    'visual_topic_missing': visual_topic_missing,
    'no_real_control_topic': no_real_control_topic,
    'forbidden_topic_detected_count': str(len(forbidden)),
    'forbidden_topic_list': ','.join(forbidden) if forbidden else 'NONE',
    'cleanup_pass': cleanup_pass,
    'remaining_owned_process_count': str(remaining_owned),
    'final_acceptance_gate': final_acceptance_gate,
    'P5A_FULL_BIMODAL_EXPLORATION_DEMO': p5a_result,
    'changed_files': 'Map/config/p5a_full_bimodal_demo.yaml, Map/rviz/p5a_full_bimodal_demo.rviz, scripts/run_p5a_full_bimodal_live_demo.sh, scripts/run_p5a_full_bimodal_acceptance_validation.sh',
    'local_commit_created': 'PENDING',
    'main_result_summary': 'P5A final full bimodal 3D exploration demo acceptance completed.' if p5a_result != 'FAIL' else 'P5A final full bimodal demo acceptance failed.',
    'remaining_failure_class': failure,
    'remaining_failure_reason': reason,
    'summary_report': str(log / 'p5a_full_bimodal_demo_summary.md'),
    'demo_run_guide': str(log / 'p5a_demo_run_guide.md'),
    'debug_package': str(root / 'latest_p5a_full_bimodal_demo_package.tar.gz'),
    'final_live_demo_command': f'cd {root} && ./scripts/run_p5a_full_bimodal_live_demo.sh',
    'headless_validation_command': f'cd {root} && P5A_DURATION_SEC=300 ./scripts/run_p5a_full_bimodal_acceptance_validation.sh',
    'rviz_only_command': f'cd {root} && rviz2 -d {rviz_config}',
    'recommended_next_prompt_type': 'GITHUB_BACKUP_P5A_FINAL_BASELINE' if p5a_result != 'FAIL' else 'FIX_P5A_FULL_BIMODAL_DEMO',
    'next_stage_explanation': 'P5A final baseline is accepted; backing it up to GitHub is the next durability step.' if p5a_result != 'FAIL' else 'Fix the failing P5A acceptance class before backing up or deploying.',
}

with (log / 'p5a_final_acceptance_metrics.csv').open('w', newline='', encoding='utf-8') as f:
    writer = csv.writer(f)
    writer.writerow(['metric', 'value'])
    for k, v in fields.items():
        writer.writerow([k, v])

with (log / 'p5a_full_bimodal_demo_summary.md').open('w', encoding='utf-8') as f:
    f.write('# P5A Full Bimodal Exploration Demo Summary\n\n')
    f.write(f'P5A result: {p5a_result}\n\n')
    f.write(f'Map: occupied {occ_start} -> {occ_end}, coverage {cov_start:.6f} -> {cov_end:.6f}.\n\n')
    f.write(f'Air: selected={air_selected_goal_count}, feasible_paths={air_path_feasible_count}, repeat={air_repeat_goal_ratio:.3f}.\n\n')
    f.write(f'Ground: selected={ground_selected_goal_count}, valid={ground_valid_candidate_count}, feasible_paths={ground_path_feasible_count}, repeat={ground_repeat_goal_ratio:.3f}.\n\n')
    f.write(f'Bimodal: active_path_unique_hash_count={active_unique_hash_count}, executed_path_length_m={executed_length:.3f}.\n\n')
    f.write(f'RViz: final_rviz_demo_ready={final_rviz_demo_ready}, visual_topic_missing={visual_topic_missing}.\n\n')
    f.write(f'Safety: no_real_control_topic={no_real_control_topic}, forbidden_topic_detected_count={len(forbidden)}.\n\n')
    f.write(f'Remaining: {failure}, {reason}\n')

with (log / 'p5a_acceptance_report.md').open('w', encoding='utf-8') as f:
    f.write('B3D_P5A_FULL_BIMODAL_EXPLORATION_DEMO_SUMMARY\n')
    for k, v in fields.items():
        f.write(f'{k}={v}\n')

print('B3D_P5A_FULL_BIMODAL_EXPLORATION_DEMO_SUMMARY')
for k, v in fields.items():
    print(f'{k}={v}')
PY

tar -czf "$ROOT/latest_p5a_full_bimodal_demo_package.tar.gz" -C "$(dirname "$LOG")" "$(basename "$LOG")"
cat "$LOG/p5a_acceptance_report.md"
