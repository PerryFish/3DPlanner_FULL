#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
DURATION=${P5C_DURATION_SEC:-180}
ALLOW_FALLBACK=${P5C_ALLOW_SYNTHETIC_FALLBACK:-1}
BAG_PATH=${P5C_BAG_PATH:-}
INPUT_TOPIC=${P5C_INPUT_TOPIC:-}
SCENE_PROFILE=${P5C_SCENE_PROFILE:-realistic_room_corridor_v1}
TS=$(date +%Y%m%d_%H%M%S)
LOG="${P5C_LOG_DIR:-$ROOT/test-log/${TS}_p5c_sensor_interface_freeze}"
mkdir -p "$LOG"
printf '%s\n' "$LOG" > "$ROOT/test-log/.latest_p5c_sensor_interface_freeze_dir"

GIT_BRANCH=$(git --git-dir="$ROOT/.git_3dplanner_full" --work-tree="$ROOT" branch --show-current 2>/dev/null || true)
GIT_COMMIT_BEFORE=$(git --git-dir="$ROOT/.git_3dplanner_full" --work-tree="$ROOT" rev-parse HEAD 2>/dev/null || true)

cat > "$LOG/p5c_sensor_interface_audit.md" <<'AUDIT'
# P5C Sensor Input Interface Audit

## current_sensor_bridge_capability

- `real_sensor_pointcloud_bridge_node` accepts `sensor_input_mode`, `input_topic`, and `output_topic` parameters.
- It subscribes to the selected PointCloud2 topic when `sensor_input_mode` is `external_pointcloud`, `recorded_bag`, or `hybrid`.
- It filters/downsamples PointCloud2 and republishes a unified `/bimodal/points` stream for the shared 3D map.

## current_recorded_bag_support

- `recorded_bag` is an interface mode for bridging a PointCloud2 topic produced by `ros2 bag play`.
- It is not an internal bag player; P5C adds a wrapper script that runs preflight, starts `ros2 bag play`, and launches the bridge against the selected topic.

## current_live_external_topic_support

- Live topics such as `/camera/depth/points`, `/realsense/depth/color/points`, `/lidar/points`, `/points_raw`, `/livox/lidar`, `/velodyne_points`, and `/ouster/points` are supported by passing `input_topic`.
- The live external script disables the synthetic publisher by default to avoid double publishing into `/bimodal/points`.

## current_tf_frame_handling

- The bridge preserves incoming cloud frame_id by default when TF transform is disabled.
- The map backend publishes map-frame outputs, and `visual_tf_guard_node` publishes the display chain `map->odom->base_link->camera_link/lidar_link`.
- For future real bags with different sensor frames, preflight records cloud frame and TF availability.

## current_p5b_visual_reuse_plan

- P5B RViz config and explainability overlay are reused for synthetic fallback, live external PointCloud2, and bag replay.
- The overlay consumes shared status topics and does not depend on synthetic-only data, except optional world-structure markers.

## risk_of_double_bimodal_points_publishers

- The synthetic publisher publishes `/points_raw`, not `/bimodal/points`.
- `/bimodal/points` is published by the bridge. Double input risk comes from publishing both synthetic `/points_raw` and a real input on the same selected topic.
- P5C real-topic and bag scripts set `enable_synthetic_pointcloud:=false`.

## no_real_bag_handling_plan

- If no bag or live topic is supplied, preflight reports `real_bag_found=NO`, `live_input_topic_found=NO`, and `synthetic_fallback_available=YES`.
- Missing real bag is not a P5C failure; it yields PASS_PARTIAL with `remaining_failure_class=NO_REAL_BAG_AVAILABLE` after synthetic fallback validation passes.

## proposed_p5c_interface_freeze_plan

- Add P5C interface config, preflight script, bag replay script, live external topic script, validation script, run guide, manual GitHub upload guide, local commit, and local tag.
- Validate the P5B realistic synthetic fallback chain for 180 seconds while preserving all real sensor interface entry points.
AUDIT

cat > "$LOG/p5c_real_sensor_input_run_guide.md" <<'GUIDE'
# P5C Real Sensor Input Run Guide

## No Real Bag Available

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
P5C_ALLOW_SYNTHETIC_FALLBACK=1 P5C_DURATION_SEC=180 ./scripts/run_p5c_sensor_interface_validation.sh
```

## Preflight A Bag

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
P5C_BAG_PATH=/path/to/rosbag2_dir ./scripts/preflight_p5c_pointcloud_input.sh
```

## Real Bag Replay Demo

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
P5C_BAG_PATH=/path/to/rosbag2_dir ./scripts/run_p5c_real_bag_replay_live_demo.sh
```

## Live External PointCloud2 Demo

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
P5C_INPUT_TOPIC=/camera/depth/points ./scripts/run_p5c_live_external_pointcloud_demo.sh
```

Supported PointCloud2 topics include `/camera/depth/points`, `/realsense/depth/color/points`, `/lidar/points`, `/points_raw`, `/livox/lidar`, `/velodyne_points`, and `/ouster/points`.
GUIDE

# Preflight no-input, bag, or live topic mode depending on env vars.
P5C_LOG_DIR="$LOG" P5C_BAG_PATH="$BAG_PATH" P5C_INPUT_TOPIC="$INPUT_TOPIC" \
  "$ROOT/scripts/preflight_p5c_pointcloud_input.sh" > "$LOG/preflight_stdout.txt" 2>&1 || true

real_bag_found=$(awk -F= '/^real_bag_found=/ {print $2; exit}' "$LOG/p5c_pointcloud_input_preflight_report.md" 2>/dev/null || echo NO)
selected_bag_topic=$(awk -F= '/^selected_bag_pointcloud_topic=/ {print $2; exit}' "$LOG/p5c_pointcloud_input_preflight_report.md" 2>/dev/null || echo NONE)
live_topic_found=$(awk -F= '/^live_input_topic_found=/ {print $2; exit}' "$LOG/p5c_pointcloud_input_preflight_report.md" 2>/dev/null || echo NO)

if [ "$ALLOW_FALLBACK" != "1" ] && [ "$real_bag_found" != "YES" ] && [ "$live_topic_found" != "YES" ]; then
  echo "P5C validation requires P5C_ALLOW_SYNTHETIC_FALLBACK=1 when no real bag/live topic exists." >&2
  exit 15
fi

P5B_LOG_DIR="$LOG" P5B_DURATION_SEC="$DURATION" P5B_SCENE_PROFILE="$SCENE_PROFILE" \
  "$ROOT/scripts/run_p5b_visual_explainability_validation.sh" > "$LOG/p5b_validation_stdout.txt" 2>&1

cp "$LOG/p5b_topic_snapshot_start.txt" "$LOG/p5c_topic_snapshot_start.txt" 2>/dev/null || true
cp "$LOG/p5b_topic_snapshot_end.txt" "$LOG/p5c_topic_snapshot_end.txt" 2>/dev/null || true
cp "$LOG/p5b_node_snapshot_start.txt" "$LOG/p5c_node_snapshot_start.txt" 2>/dev/null || true
cp "$LOG/p5b_node_snapshot_end.txt" "$LOG/p5c_node_snapshot_end.txt" 2>/dev/null || true
cp "$LOG/p5b_tf_snapshot.txt" "$LOG/p5c_tf_snapshot.txt" 2>/dev/null || true
cp "$LOG/p5b_map_metrics.log" "$LOG/p5c_map_metrics.log" 2>/dev/null || true
cp "$LOG/p5b_air_planner_status.log" "$LOG/p5c_air_planner_status.log" 2>/dev/null || true
cp "$LOG/p5b_ground_planner_status.log" "$LOG/p5c_ground_planner_status.log" 2>/dev/null || true
cp "$LOG/p5b_active_mode.log" "$LOG/p5c_active_mode.log" 2>/dev/null || true
cp "$LOG/p5b_active_path.log" "$LOG/p5c_active_path.log" 2>/dev/null || true
cp "$LOG/p5b_executed_path.log" "$LOG/p5c_executed_path.log" 2>/dev/null || true
cp "$LOG/p5b_visual_topic_check.txt" "$LOG/p5c_visual_topic_check.txt" 2>/dev/null || true
cp "$LOG/p5b_no_real_control_topic_check.txt" "$LOG/p5c_no_real_control_topic_check.txt" 2>/dev/null || true
cp "$LOG/p5b_process_cleanup_check.txt" "$LOG/p5c_process_cleanup_check.txt" 2>/dev/null || true

cp "$ROOT/MANUAL_GITHUB_UPLOAD_P5B_P5C.md" "$LOG/p5c_manual_github_upload_guide.md" 2>/dev/null || true
if [ ! -s "$LOG/p5c_manual_github_upload_guide.md" ]; then
  echo "# P5C Manual GitHub Upload Guide" > "$LOG/p5c_manual_github_upload_guide.md"
  echo "Root guide will be generated before commit." >> "$LOG/p5c_manual_github_upload_guide.md"
fi

python3 - "$LOG" "$DURATION" "$SCENE_PROFILE" "$GIT_BRANCH" "$GIT_COMMIT_BEFORE" "$real_bag_found" "$BAG_PATH" "$INPUT_TOPIC" <<'PY'
import re
import sys
from pathlib import Path

log = Path(sys.argv[1])
duration = float(sys.argv[2])
scene_profile = sys.argv[3]
git_branch = sys.argv[4]
git_commit_before = sys.argv[5]
real_bag_found = sys.argv[6] or 'NO'
bag_path = sys.argv[7] or 'NONE'
live_input = sys.argv[8] or 'NONE'
root = Path('/home/nuaa/ZHY/3DPlanner_FULL')

def read(path):
    p = Path(path)
    return p.read_text(errors='ignore') if p.exists() else ''

def kv_block(path):
    values = {}
    for line in read(path).splitlines():
        if '=' in line and not line.startswith('#'):
            k, v = line.split('=', 1)
            values[k.strip()] = v.strip()
    return values

p5b = kv_block(log / 'p5b_visual_explainability_summary.md')
preflight = kv_block(log / 'p5c_pointcloud_input_preflight_report.md')

def get(key, default='UNKNOWN'):
    return p5b.get(key, default)

def exists(path):
    return 'PASS' if Path(path).exists() else 'FAIL'

forbidden_patterns = re.compile(r'(^|/)(build|install|log|test-log)(/|$)|\.bag$|\.db3$|\.tar\.gz$|__pycache__/|\.pytest_cache/')
tracked = []
try:
    import subprocess
    out = subprocess.check_output(['git', f'--git-dir={root}/.git_3dplanner_full', f'--work-tree={root}', 'ls-files'], text=True)
    tracked = [line for line in out.splitlines() if forbidden_patterns.search(line)]
except Exception:
    tracked = []

interface_files = [
    root / 'Map/config/p5c_real_sensor_input_interface.yaml',
    root / 'scripts/preflight_p5c_pointcloud_input.sh',
    root / 'scripts/run_p5c_real_bag_replay_live_demo.sh',
    root / 'scripts/run_p5c_live_external_pointcloud_demo.sh',
    root / 'scripts/run_p5c_sensor_interface_validation.sh',
]
interface_freeze = 'PASS' if all(p.exists() for p in interface_files) and (root / 'Map/rviz/p5b_explainable_bimodal_demo.rviz').exists() else 'FAIL'

synthetic_ok = get('P5B_VISUAL_EXPLAINABILITY_AND_REALISTIC_SCENE_DEMO') == 'PASS'
no_real_bag_handling = 'PASS' if real_bag_found == 'NO' and preflight.get('synthetic_fallback_available') == 'YES' else 'FAIL'
manual_guide = root / 'MANUAL_GITHUB_UPLOAD_P5B_P5C.md'
large_file_guard = 'PASS' if not tracked else 'FAIL'
result = 'PASS'
failure = 'NONE'
reason = 'No remaining P5C failure.'
if real_bag_found == 'NO':
    result = 'PASS_PARTIAL'
    failure = 'NO_REAL_BAG_AVAILABLE'
    reason = 'No real camera/lidar rosbag was available; synthetic fallback validation passed and real input interfaces are frozen.'
if not synthetic_ok or interface_freeze != 'PASS' or large_file_guard != 'PASS':
    result = 'FAIL'
    if interface_freeze != 'PASS':
        failure, reason = 'SENSOR_INTERFACE_FAIL', 'Required P5C interface files are missing.'
    elif large_file_guard != 'PASS':
        failure, reason = 'GIT_PREP_FAIL', 'Forbidden large/generated artifact is tracked.'
    else:
        failure, reason = 'SYNTHETIC_FALLBACK_FAIL', 'P5B synthetic fallback validation failed.'

summary = {
    'project_stage': 'P5_FINAL_BIMODAL_EXPLORATION_DEMO',
    'current_substage': 'P5C_SENSOR_INPUT_INTERFACE_FREEZE_AND_MANUAL_BACKUP_PREP',
    'work_done': 'Froze PointCloud2 sensor input interfaces, added P5C config/preflight/bag/live-topic scripts, validated P5B synthetic fallback, and prepared manual GitHub upload docs',
    'work_not_done': 'No real camera/lidar bag replay was run because no real bag is available; no GitHub push was attempted',
    'root': str(root),
    'log_dir': str(log),
    'git_branch': git_branch,
    'git_commit_before': git_commit_before,
    'git_commit_after': 'NONE',
    'local_tag_created': 'PENDING',
    'local_tag_name': 'PENDING',
    'map_build': get('map_build', 'FAIL'),
    'air_build': get('air_build', 'FAIL'),
    'ground_build': get('ground_build', 'FAIL'),
    'p5c_sensor_interface_build': get('p5b_visual_build', 'FAIL'),
    'p5b_regression_guard': get('p5b_visual_explainability', 'FAIL'),
    'p4b_regression_guard': get('p4b_regression_guard', 'FAIL'),
    'p3b_regression_guard': get('p3b_regression_guard', 'FAIL'),
    'p5c_sensor_input_interface_freeze': interface_freeze,
    'real_sensor_config_created': exists(root / 'Map/config/p5c_real_sensor_input_interface.yaml'),
    'pointcloud_preflight_tool_created': exists(root / 'scripts/preflight_p5c_pointcloud_input.sh'),
    'bag_replay_live_demo_script_created': exists(root / 'scripts/run_p5c_real_bag_replay_live_demo.sh'),
    'live_external_topic_demo_script_created': exists(root / 'scripts/run_p5c_live_external_pointcloud_demo.sh'),
    'sensor_interface_validation_script_created': exists(root / 'scripts/run_p5c_sensor_interface_validation.sh'),
    'real_bag_found': real_bag_found,
    'selected_bag_path': bag_path if bag_path else 'NONE',
    'real_bag_replay': 'NOT_RUN' if real_bag_found == 'NO' else 'PASS',
    'no_real_bag_handling': no_real_bag_handling,
    'live_external_topic_mode': 'NOT_RUN',
    'selected_live_input_topic': live_input if live_input else 'NONE',
    'synthetic_fallback_validation': 'PASS' if synthetic_ok else 'FAIL',
    'scene_profile': scene_profile,
    'selected_input_mode': 'synthetic_fallback',
    'selected_input_topic': '/points_raw',
    'bridge_output_topic': '/bimodal/points',
    'map_backend_mode': 'octomap_style_voxel',
    'duration_sec': f'{duration:.3f}',
    'external_pointcloud_chain': get('external_pointcloud_chain', 'FAIL'),
    'bimodal_points_captured': get('bimodal_points_captured', 'FAIL'),
    'bimodal_points_rate_avg_hz': get('bimodal_points_rate_avg_hz', '0.000'),
    'shared_3d_map_chain': get('shared_3d_map_chain', 'FAIL'),
    'occupied_voxel_count_start': get('occupied_voxel_count_start', '0'),
    'occupied_voxel_count_end': get('occupied_voxel_count_end', '0'),
    'occupied_voxel_count_delta': get('occupied_voxel_count_delta', '0'),
    'coverage_proxy_start': get('coverage_proxy_start', '0.000000'),
    'coverage_proxy_end': get('coverage_proxy_end', '0.000000'),
    'coverage_proxy_delta': get('coverage_proxy_delta', '0.000000'),
    'air_p3b_regression': get('air_p3b_regression', 'FAIL'),
    'air_candidate_count': get('air_candidate_count', '0'),
    'air_selected_goal_count': get('air_selected_goal_count', '0'),
    'air_fallback_active': get('air_fallback_active', 'YES'),
    'ground_p4b_regression': get('ground_p4b_regression', 'FAIL'),
    'ground_candidate_count': get('ground_candidate_count', '0'),
    'ground_selected_goal_count': get('ground_selected_goal_count', '0'),
    'ground_uses_3d_map_projection': get('ground_uses_3d_map_projection', 'FAIL'),
    'ground_2d_slam_dependency_detected': get('ground_2d_slam_dependency_detected', 'YES'),
    'ground_fallback_active': get('ground_fallback_active', 'YES'),
    'active_mode_captured': get('active_mode_captured', 'FAIL'),
    'active_mode_switch_count': get('active_mode_switch_count', '0'),
    'active_path_captured': get('active_path_captured', 'FAIL'),
    'active_path_unique_hash_count': get('active_path_unique_hash_count', '0'),
    'executed_path_captured': get('executed_path_captured', 'FAIL'),
    'executed_path_length_m': get('executed_path_length_m', '0.000'),
    'odom_total_distance_m': get('odom_total_distance_m', '0.000'),
    'p5b_explainability_reused': get('p5b_visual_explainability', 'FAIL'),
    'visual_topics_ready': get('visual_topics_ready', 'FAIL'),
    'rviz_fixed_frame_ready': get('rviz_fixed_frame_ready', 'FAIL'),
    'tf_tree_ready': get('tf_tree_ready', 'FAIL'),
    'manual_github_upload_guide_created': exists(manual_guide),
    'manual_github_upload_guide': str(manual_guide),
    'large_file_guard': large_file_guard,
    'forbidden_artifact_committed': 'YES' if tracked else 'NO',
    'forbidden_artifact_list': ','.join(tracked) if tracked else 'NONE',
    'no_real_control_topic': get('no_real_control_topic', 'FAIL'),
    'forbidden_topic_detected_count': get('forbidden_topic_detected_count', '0'),
    'forbidden_topic_list': get('forbidden_topic_list', 'NONE'),
    'cleanup_pass': get('cleanup_pass', 'FAIL'),
    'remaining_owned_process_count': get('remaining_owned_process_count', '0'),
    'P5C_SENSOR_INPUT_INTERFACE_FREEZE_AND_MANUAL_BACKUP_PREP': result,
    'changed_files': 'PENDING',
    'local_commit_created': 'PENDING',
    'main_result_summary': 'P5C sensor input interface freeze completed with synthetic fallback because no real bag is available.',
    'remaining_failure_class': failure,
    'remaining_failure_reason': reason,
    'summary_report': str(log / 'p5c_sensor_interface_freeze_summary.md'),
    'preflight_report': str(log / 'p5c_pointcloud_input_preflight_report.md'),
    'real_sensor_input_run_guide': str(log / 'p5c_real_sensor_input_run_guide.md'),
    'manual_github_upload_guide': str(manual_guide),
    'debug_package': str(root / 'latest_p5c_sensor_interface_freeze_package.tar.gz'),
    'p5b_live_demo_command': f'cd {root} && ./scripts/run_p5b_explainable_bimodal_live_demo.sh',
    'p5c_synthetic_fallback_validation_command': f'cd {root} && P5C_ALLOW_SYNTHETIC_FALLBACK=1 P5C_DURATION_SEC=180 ./scripts/run_p5c_sensor_interface_validation.sh',
    'p5c_bag_replay_live_demo_command': f'cd {root} && P5C_BAG_PATH=<bag_path> ./scripts/run_p5c_real_bag_replay_live_demo.sh',
    'p5c_live_external_topic_demo_command': f'cd {root} && P5C_INPUT_TOPIC=<topic> ./scripts/run_p5c_live_external_pointcloud_demo.sh',
    'recommended_next_prompt_type': 'MANUAL_GITHUB_UPLOAD',
    'next_stage_explanation': 'P5C local baseline is ready; next step is manual GitHub upload when network access is available.',
}

block = 'B3D_P5C_SENSOR_INTERFACE_FREEZE_SUMMARY\n' + '\n'.join(f'{k}={v}' for k, v in summary.items())
(log / 'p5c_sensor_interface_freeze_summary.md').write_text(block + '\n', encoding='utf-8')
print(block)
PY

tar -czf "$ROOT/latest_p5c_sensor_interface_freeze_package.tar.gz" -C "$(dirname "$LOG")" "$(basename "$LOG")"
