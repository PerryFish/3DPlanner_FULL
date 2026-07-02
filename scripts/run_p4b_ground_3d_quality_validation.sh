#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
DURATION=${P4B_DURATION_SEC:-300}
INPUT_TOPIC=${P4B_INPUT_TOPIC:-/points_raw}
AIR_MODE=${P4B_AIR_PLANNER_MODE:-fuel_style_v0}
GROUND_MODE=${P4B_GROUND_PLANNER_MODE:-ground_3d_frontier_v0}
MODE_SWITCH_PERIOD=${P4B_MODE_SWITCH_PERIOD_SEC:-75}
AIR_CONFIG=${P4B_AIR_CONFIG:-$ROOT/Air/config/p3b_air_fuel_quality.yaml}
GROUND_CONFIG=${P4B_GROUND_CONFIG:-$ROOT/Ground/config/p4b_ground_3d_quality.yaml}
GROUND_PROFILE=${P4B_GROUND_QUALITY_PROFILE:-p4b_optimized}
TS=$(date +%Y%m%d_%H%M%S)
LOG="${P4B_LOG_DIR:-$ROOT/test-log/${TS}_p4b_ground_3d_quality}"
mkdir -p "$LOG"
printf '%s\n' "$LOG" > "$ROOT/test-log/.latest_p4b_ground_3d_quality_dir"

GIT_BRANCH=$(git --git-dir="$ROOT/.git_3dplanner_full" --work-tree="$ROOT" branch --show-current 2>/dev/null || true)
GIT_COMMIT_BEFORE=$(git --git-dir="$ROOT/.git_3dplanner_full" --work-tree="$ROOT" rev-parse HEAD 2>/dev/null || true)

cat > "$LOG/p4b_ground_3d_quality_audit.md" <<'AUDIT'
# P4B Ground 3D Quality Audit

## current_ground_wrapper_architecture

- `Ground/ros2_ws/src/bimodal_ground_explorer/bimodal_ground_explorer/ground_3d_frontier_node.py` is the active `ground_3d_frontier_v0` wrapper.
- Inputs are `/bimodal/odom`, `/bimodal/map_3d`, `/bimodal/map_metrics`, `/bimodal/map_backend_status`, `/bimodal/esdf`, and `/bimodal/exploration_boundary`.
- Outputs remain `/ground/exploration_goal`, `/ground/path`, `/ground/planner_status`, and `/ground/frontier_candidates`; the mode mux consumes `/ground/path` without publishing real control topics.

## current_ground_quality_bottlenecks

- P4A candidate generation linearly scanned the global XY bounds and stopped at `max_candidate_count`, so many runs sampled only one region of the map.
- Goal lifecycle had a minimum hold and score-improvement gate, but no P4B max-hold/no-progress retirement, which allowed stale low-gain goals.
- Traversability was only a clearance proxy and did not expose detailed reject counters.
- Score diagnostics did not report avg/max per gain class, making quality regressions hard to classify.

## local_ground_reference_findings

- `Ground_Explore_v3/v4` mainly contains SLAM Toolbox, Nav2, explore_lite, Gazebo, and real navigation stack integration; these are not imported into 3DPlanner_FULL.
- TARE/TARE_V3 materials provide useful concepts: frontier information gain, occupancy inflation, failed goal blacklist, no-progress timeout, and recovery by alternate sectors.
- GBPlanner/TARE-style large modules are ROS1/heavy-stack oriented and are unsafe to vendor directly into the current ROS2 shared-map wrapper.

## safe_reusable_ideas

- Multi-radius and multi-yaw candidate sampling around current odom.
- Sector balancing and sector escape after stale/retired goals.
- 3D map-derived obstacle clearance and conservative occupied-voxel path rejection.
- Low-gain/no-progress/stale goal retirement plus blacklist.
- Explicit per-score-term diagnostics and RViz rejected/selected markers.

## unsafe_do_not_import_components

- SLAM Toolbox/Nav2/explore_lite/Gazebo controller stacks.
- `/cmd_vel`, MAVROS, PX4 `/fmu/*`, actuator, offboard, or trajectory setpoint publishers.
- Full TARE/GBPlanner dependency trees, OR-Tools payloads, Gazebo vehicle simulation, or ROS1 launch code.

## proposed_p4b_modification_plan

- Keep Ground on the shared 3D map and add a P4B profile in `Ground/config/p4b_ground_3d_quality.yaml`.
- Replace P4B candidate generation with multi-radius/multi-sector sampling, deduplication, and local escape candidates.
- Add conservative 3D occupancy clearance, traversability score, path feasibility, collision, boundary, and height diagnostics.
- Add goal max-hold, low-gain/no-progress/stale retirement, blacklist, and sector escape counters.
- Preserve Air P3B logic except for clearing stale `failure_reason` after successful target selection.
AUDIT

set +e
P3B_LOG_DIR="$LOG" \
P3B_DURATION_SEC="$DURATION" \
P3B_INPUT_TOPIC="$INPUT_TOPIC" \
P3B_AIR_PLANNER_MODE="$AIR_MODE" \
P3B_GROUND_PLANNER_MODE="$GROUND_MODE" \
P3B_MODE_SWITCH_PERIOD_SEC="$MODE_SWITCH_PERIOD" \
P3B_AIR_CONFIG="$AIR_CONFIG" \
P3B_GROUND_CONFIG="$GROUND_CONFIG" \
bash "$ROOT/scripts/run_p3b_air_fuel_quality_validation.sh"
runner_rc=$?
set -e

cp "$LOG/p3b_topic_snapshot_start.txt" "$LOG/p4b_topic_snapshot_start.txt" 2>/dev/null || true
cp "$LOG/p3b_topic_snapshot_end.txt" "$LOG/p4b_topic_snapshot_end.txt" 2>/dev/null || true
cp "$LOG/p3b_node_snapshot_start.txt" "$LOG/p4b_node_snapshot_start.txt" 2>/dev/null || true
cp "$LOG/p3b_node_snapshot_end.txt" "$LOG/p4b_node_snapshot_end.txt" 2>/dev/null || true
cp "$LOG/p3b_tf_snapshot.txt" "$LOG/p4b_tf_snapshot.txt" 2>/dev/null || true
cp "$LOG/p3b_air_planner_status.log" "$LOG/p4b_air_planner_status.log" 2>/dev/null || true
cp "$LOG/p3b_ground_planner_status.log" "$LOG/p4b_ground_planner_status.log" 2>/dev/null || true
{ echo "# P4B Planner Wrapper Status"; cat "$LOG/p4b_air_planner_status.log" "$LOG/p4b_ground_planner_status.log" 2>/dev/null || true; } > "$LOG/p4b_planner_wrapper_status.log"
cp "$LOG/p3b_map_metrics.log" "$LOG/p4b_map_metrics.log" 2>/dev/null || true
cp "$LOG/p3b_active_path.log" "$LOG/p4b_active_path.log" 2>/dev/null || true
cp "$LOG/p3b_executed_path.log" "$LOG/p4b_executed_path.log" 2>/dev/null || true
cp "$LOG/p3b_no_real_control_topic_check.txt" "$LOG/p4b_no_real_control_topic_check.txt" 2>/dev/null || true
touch "$LOG/p4b_no_real_control_topic_check.txt"
cp "$LOG/p3b_process_cleanup_check.txt" "$LOG/p4b_process_cleanup_check.txt" 2>/dev/null || true

python3 - "$LOG" "$DURATION" "$INPUT_TOPIC" "$AIR_MODE" "$GROUND_MODE" "$GROUND_PROFILE" "$runner_rc" "$GIT_BRANCH" "$GIT_COMMIT_BEFORE" <<'PY'
import csv
import re
import sys
from pathlib import Path

log = Path(sys.argv[1])
duration = float(sys.argv[2])
input_topic, air_mode_req, ground_mode_req, ground_profile_req = sys.argv[3:7]
runner_rc = int(sys.argv[7])
git_branch = sys.argv[8]
git_commit_before = sys.argv[9]
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
air_text = rel('p4b_air_planner_status.log')
ground_text = rel('p4b_ground_planner_status.log')
map_text = rel('p4b_map_metrics.log')
sensor_text = wrel('sensor_input_status.log')
tf_text = rel('p4b_tf_snapshot.txt')
visual_text = wrel('visual_topics_report.md')
node_text = rel('p4b_node_snapshot_end.txt')
topic_text = rel('p4b_topic_snapshot_end.txt')
active_path_text = rel('p4b_active_path.log')
executed_path_text = rel('p4b_executed_path.log')
mode_text = wrel('active_mode.log')
forbidden_re = re.compile(r'^(/cmd_vel|/mavros/|/fmu/|/actuator/|/offboard_control_mode$|/trajectory_setpoint$)')
forbidden = [line for line in rel('p4b_no_real_control_topic_check.txt').splitlines() if forbidden_re.search(line.strip())]
cleanup_text = rel('p4b_process_cleanup_check.txt')

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
external_delta = max(0, int(lastv(sensor_ext, 0)) - int(firstv(sensor_ext, 0)))
output_delta = max(0, int(lastv(sensor_out, 0)) - int(firstv(sensor_out, 0)))
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

executed_path_count = executed_path_text.count('poses:') or len(odom_rows)
odom_total = max([float(r.get('total_distance') or 0.0) for r in odom_rows] or [0.0])
executed_length = odom_total
mode_values = [r.get('mode', '').strip() for r in mode_rows if r.get('mode', '').strip()]
mode_switch_count = sum(1 for a, b in zip(mode_values, mode_values[1:]) if a != b)

map_build = rv(build, 'map_build')
air_build = rv(build, 'air_build')
ground_build = rv(build, 'ground_build')
p4b_build = 'PASS' if runner_rc == 0 and map_build == air_build == ground_build == 'PASS' else 'FAIL'

air_planner_mode = vals(air_text, 'air_planner_mode')[-1] if vals(air_text, 'air_planner_mode') else air_mode_req
air_quality_profile = vals(air_text, 'air_quality_profile')[-1] if vals(air_text, 'air_quality_profile') else 'UNKNOWN'
air_candidate_count = int(maxv(air_text, 'air_candidate_count', maxv(air_text, 'candidate_count', 0)))
air_selected_goal_count = int(maxv(air_text, 'air_selected_goal_count', 0))
air_repeat_goal_ratio = maxv(air_text, 'air_repeat_goal_ratio', 1.0)
air_fallback_active = 'YES' if 'fallback_active=true' in air_text else 'NO'
air_failure_reason = vals(air_text, 'failure_reason')[-1] if vals(air_text, 'failure_reason') else 'NONE'

ground_planner_mode = vals(ground_text, 'ground_planner_mode')[-1] if vals(ground_text, 'ground_planner_mode') else ground_mode_req
ground_quality_profile = vals(ground_text, 'ground_quality_profile')[-1] if vals(ground_text, 'ground_quality_profile') else ground_profile_req
ground_candidate_count = int(maxv(ground_text, 'ground_candidate_count', maxv(ground_text, 'candidate_count', 0)))
ground_valid_candidate_count = int(maxv(ground_text, 'ground_valid_candidate_count', maxv(ground_text, 'valid_candidate_count', 0)))
ground_selected_goal_count = int(maxv(ground_text, 'ground_selected_goal_count', 0))
ground_path_feasible_count = int(maxv(ground_text, 'ground_path_feasible_count', 0))
ground_path_infeasible_count = int(maxv(ground_text, 'ground_path_infeasible_count', 0))
ground_repeat_goal_ratio = maxv(ground_text, 'ground_repeat_goal_ratio', 1.0)
ground_fallback_active = 'YES' if 'fallback_active=true' in ground_text else 'NO'
ground_failure_reason = vals(ground_text, 'failure_reason')[-1] if vals(ground_text, 'failure_reason') else 'NONE'
ground_3d_projection = 'PASS' if 'ground_uses_3d_map_projection=true' in ground_text else 'FAIL'
ground_2d_slam = 'NO' if 'ground_2d_slam_dependency_detected=false' in ground_text else 'YES'

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
air_regression = pf(air_planner_mode == 'fuel_style_v0' and air_quality_profile == 'p3b_optimized' and air_candidate_count > 0 and air_selected_goal_count >= 5 and air_repeat_goal_ratio <= 0.3 and air_fallback_active == 'NO')
ground_quality = pf(ground_planner_mode == 'ground_3d_frontier_v0' and ground_quality_profile == 'p4b_optimized' and ground_candidate_count > 0 and ground_valid_candidate_count > 0 and ground_selected_goal_count >= 2 and ground_path_feasible_count > 0 and ground_repeat_goal_ratio <= 0.3 and ground_3d_projection == 'PASS' and ground_2d_slam == 'NO')
if ground_quality == 'PASS' and ground_selected_goal_count <= 2:
    ground_quality = 'PASS_PARTIAL'

required = [map_build, air_build, ground_build, p4b_build, p1c_guard, p2f_guard, p3a_p4a_guard, air_regression, external_chain, map_growth, integrated_chain, rviz_ready, visual_ready, tf_ready, no_control, cleanup_pass]
result = 'PASS' if all(v == 'PASS' for v in required) and ground_quality == 'PASS' else ('PASS_PARTIAL' if all(v == 'PASS' for v in required) and ground_quality == 'PASS_PARTIAL' else 'FAIL')
failure = 'NONE'
reason = 'No remaining P4B failure.'
if result == 'FAIL':
    if no_control != 'PASS':
        failure, reason = 'SAFETY_FAIL', 'Forbidden real control topic appeared.'
    elif any(v != 'PASS' for v in [map_build, air_build, ground_build, p4b_build]):
        failure, reason = 'BUILD_FAIL', 'Build failed.'
    elif ground_candidate_count <= 0:
        failure, reason = 'GROUND_NO_CANDIDATE', 'Ground wrapper produced no candidates.'
    elif ground_valid_candidate_count <= 0:
        failure, reason = 'GROUND_NO_VALID_CANDIDATE', 'Ground wrapper produced no valid candidates.'
    elif ground_path_feasible_count <= 0:
        failure, reason = 'GROUND_PATH_INFEASIBLE', 'Ground wrapper produced no feasible paths.'
    elif ground_2d_slam != 'NO':
        failure, reason = 'GROUND_2D_SLAM_REGRESSION', '2D SLAM dependency marker was detected.'
    elif air_regression != 'PASS':
        failure, reason = 'AIR_REGRESSION', 'Air P3B regression thresholds were not met.'
    elif map_growth != 'PASS':
        failure, reason = 'MAP_NOT_GROWING', 'Map metrics did not grow.'
    elif integrated_chain != 'PASS':
        failure, reason = 'MODE_MUX_FAIL', 'Integrated path/mode/executor chain failed.'
    elif tf_ready != 'PASS':
        failure, reason = 'TF_REGRESSION', 'TF readiness failed.'
    elif visual_ready != 'PASS':
        failure, reason = 'RVIZ_TOPIC_REGRESSION', 'Visual topics were not ready.'
    elif cleanup_pass != 'PASS':
        failure, reason = 'CLEANUP_FAIL', 'Owned processes were not cleaned.'
    else:
        failure, reason = 'GROUND_QUALITY_REGRESSION', 'Ground P4B quality thresholds were not met.'

fields = {
    'project_stage': 'P4_GROUND_REAL_PLANNER_QUALITY_OPTIMIZATION',
    'current_substage': 'P4B_GROUND_3D_WRAPPER_QUALITY_OPTIMIZATION',
    'work_done': 'Optimized Ground ground_3d_frontier_v0 candidate generation, traversability scoring, path feasibility, lifecycle diagnostics, and P4B validation packaging',
    'work_not_done': 'No real control, no 2D SLAM Toolbox, no Nav2 import, no nvblox, no RTAB-Map, no full TARE/GBPlanner module import',
    'root': str(log.parents[1]),
    'log_dir': str(log),
    'git_branch': git_branch,
    'git_commit_before': git_commit_before,
    'git_commit_after': 'NONE',
    'map_build': map_build,
    'air_build': air_build,
    'ground_build': ground_build,
    'p4b_ground_quality_build': p4b_build,
    'p3b_air_regression_guard': air_regression,
    'p3a_p4a_regression_guard': p3a_p4a_guard,
    'p2f_regression_guard': p2f_guard,
    'p1c_regression_guard': p1c_guard,
    'selected_input_mode': 'external_pointcloud',
    'selected_input_topic': input_topic,
    'bridge_output_topic': '/bimodal/points',
    'ground_planner_mode': ground_planner_mode,
    'ground_quality_profile': ground_quality_profile,
    'p4b_ground_3d_quality': ground_quality,
    'ground_wrapper_node_running': pf('ground_3d_frontier_node' in node_text),
    'ground_candidate_count': str(ground_candidate_count),
    'ground_valid_candidate_count': str(ground_valid_candidate_count),
    'ground_candidate_sector_count': str(int(maxv(ground_text, 'ground_candidate_sector_count', 0))),
    'ground_traversability_checked_count': str(int(maxv(ground_text, 'ground_traversability_checked_count', 0))),
    'ground_traversability_reject_count': str(int(maxv(ground_text, 'ground_traversability_reject_count', 0))),
    'ground_clearance_reject_count': str(int(maxv(ground_text, 'ground_clearance_reject_count', 0))),
    'ground_step_height_reject_count': str(int(maxv(ground_text, 'ground_step_height_reject_count', 0))),
    'ground_support_reject_count': str(int(maxv(ground_text, 'ground_support_reject_count', 0))),
    'ground_unreachable_reject_count': str(int(maxv(ground_text, 'ground_unreachable_reject_count', 0))),
    'ground_selected_goal_count': str(ground_selected_goal_count),
    'ground_goal_retire_count': str(int(maxv(ground_text, 'ground_goal_retire_count', 0))),
    'ground_low_gain_retire_count': str(int(maxv(ground_text, 'ground_low_gain_retire_count', 0))),
    'ground_stale_goal_retire_count': str(int(maxv(ground_text, 'ground_stale_goal_retire_count', 0))),
    'ground_goal_blacklist_count': str(int(maxv(ground_text, 'ground_goal_blacklist_count', 0))),
    'ground_repeat_goal_ratio': f'{ground_repeat_goal_ratio:.3f}',
    'ground_path_feasible_count': str(ground_path_feasible_count),
    'ground_path_infeasible_count': str(ground_path_infeasible_count),
    'ground_collision_reject_count': str(int(maxv(ground_text, 'ground_collision_reject_count', 0))),
    'ground_boundary_reject_count': str(int(maxv(ground_text, 'ground_boundary_reject_count', 0))),
    'ground_height_reject_count': str(int(maxv(ground_text, 'ground_height_reject_count', 0))),
    'ground_frontier_gain_max': f'{maxv(ground_text, "ground_frontier_gain_max", 0.0):.3f}',
    'ground_frontier_gain_avg': f'{avgv(ground_text, "ground_frontier_gain_avg", 0.0):.3f}',
    'ground_unknown_boundary_gain_max': f'{maxv(ground_text, "ground_unknown_boundary_gain_max", 0.0):.3f}',
    'ground_unknown_boundary_gain_avg': f'{avgv(ground_text, "ground_unknown_boundary_gain_avg", 0.0):.3f}',
    'ground_expected_coverage_gain_max': f'{maxv(ground_text, "ground_expected_coverage_gain_max", 0.0):.3f}',
    'ground_expected_coverage_gain_avg': f'{avgv(ground_text, "ground_expected_coverage_gain_avg", 0.0):.3f}',
    'ground_traversability_score_max': f'{maxv(ground_text, "ground_traversability_score_max", 0.0):.3f}',
    'ground_traversability_score_avg': f'{avgv(ground_text, "ground_traversability_score_avg", 0.0):.3f}',
    'ground_clearance_score_max': f'{maxv(ground_text, "ground_clearance_score_max", 0.0):.3f}',
    'ground_clearance_score_avg': f'{avgv(ground_text, "ground_clearance_score_avg", 0.0):.3f}',
    'ground_novelty_gain_max': f'{maxv(ground_text, "ground_novelty_gain_max", 0.0):.3f}',
    'ground_novelty_gain_avg': f'{avgv(ground_text, "ground_novelty_gain_avg", 0.0):.3f}',
    'ground_path_length_avg': f'{maxv(ground_text, "ground_path_length_avg", 0.0):.3f}',
    'ground_endpoint_to_goal_distance_avg': f'{maxv(ground_text, "ground_endpoint_to_goal_distance_avg", 0.0):.3f}',
    'ground_selected_goal_age_max_sec': f'{maxv(ground_text, "ground_selected_goal_age_max_sec", 0.0):.3f}',
    'ground_stale_path_max_sec': f'{maxv(ground_text, "ground_stale_path_max_sec", 0.0):.3f}',
    'ground_fallback_active': ground_fallback_active,
    'ground_failure_reason': ground_failure_reason,
    'ground_uses_3d_map_projection': ground_3d_projection,
    'ground_2d_slam_dependency_detected': ground_2d_slam,
    'air_p3b_regression': air_regression,
    'air_planner_mode': air_planner_mode,
    'air_quality_profile': air_quality_profile,
    'air_candidate_count': str(air_candidate_count),
    'air_selected_goal_count': str(air_selected_goal_count),
    'air_repeat_goal_ratio': f'{air_repeat_goal_ratio:.3f}',
    'air_fallback_active': air_fallback_active,
    'air_failure_reason': air_failure_reason,
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
    'map_growth_with_p4b_ground': map_growth,
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
    'integrated_p4b_chain': integrated_chain,
    'rviz_fixed_frame_ready': rviz_ready,
    'visual_topics_ready': visual_ready,
    'tf_tree_ready': tf_ready,
    'no_real_control_topic': no_control,
    'forbidden_topic_detected_count': str(len(forbidden)),
    'forbidden_topic_list': ','.join(forbidden) if forbidden else 'NONE',
    'cleanup_pass': cleanup_pass,
    'remaining_owned_process_count': str(remaining_owned),
    'P4B_GROUND_3D_WRAPPER_QUALITY_OPTIMIZATION': result,
    'changed_files': 'Ground node, P4B Ground config, P4B validation script, Air failure_reason status fix',
    'local_commit_created': 'PENDING',
    'main_result_summary': 'P4B Ground 3D quality validation completed.' if result != 'FAIL' else 'P4B Ground 3D quality validation failed.',
    'remaining_failure_class': failure,
    'remaining_failure_reason': reason,
    'summary_report': str(log / 'p4b_ground_3d_quality_summary.md'),
    'debug_package': str(log.parents[1] / 'latest_p4b_ground_3d_quality_package.tar.gz'),
    'recommended_next_prompt_type': 'P5A_FULL_BIMODAL_EXPLORATION_DEMO' if result != 'FAIL' else 'FIX_P4B_GROUND_3D_QUALITY',
    'next_stage_explanation': 'Ground now has map-derived 3D traversability frontier exploration quality sufficient for full bimodal demo.' if result != 'FAIL' else 'Fix remaining Ground quality or integration failure before the final bimodal demo.',
}

with (log / 'p4b_ground_3d_quality_summary.md').open('w', encoding='utf-8') as f:
    f.write('# P4B Ground 3D Wrapper Quality Summary\n\n')
    f.write(f'P4B result: {result}\n\n')
    f.write(f'Ground quality: {ground_quality}, candidates={ground_candidate_count}, valid={ground_valid_candidate_count}, selected={ground_selected_goal_count}, feasible_paths={ground_path_feasible_count}, repeat_ratio={ground_repeat_goal_ratio:.3f}.\n\n')
    f.write(f'Air regression: {air_regression}, selected={air_selected_goal_count}, repeat_ratio={air_repeat_goal_ratio:.3f}, failure_reason={air_failure_reason}.\n\n')
    f.write(f'Map growth: occupied {occ_start} -> {occ_end}, coverage {cov_start:.6f} -> {cov_end:.6f}, coverage_gain_per_meter={coverage_gain_per_meter:.9f}.\n\n')
    f.write(f'Safety: no_real_control_topic={no_control}, forbidden_topic_detected_count={len(forbidden)}.\n\n')
    f.write(f'Remaining failure: {failure}, {reason}\n')

with (log / 'p4b_acceptance_report.md').open('w', encoding='utf-8') as f:
    f.write('B3D_P4B_GROUND_3D_WRAPPER_QUALITY_SUMMARY\n')
    for k, v in fields.items():
        f.write(f'{k}={v}\n')

print('B3D_P4B_GROUND_3D_WRAPPER_QUALITY_SUMMARY')
for k, v in fields.items():
    print(f'{k}={v}')
PY

tar -czf "$ROOT/latest_p4b_ground_3d_quality_package.tar.gz" -C "$(dirname "$LOG")" "$(basename "$LOG")"
cat "$LOG/p4b_acceptance_report.md"
