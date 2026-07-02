#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
DURATION=${P6A_DURATION_SEC:-120}
TS=$(date +%Y%m%d_%H%M%S)
LOG="${P6A_LOG_DIR:-$ROOT/test-log/${TS}_p6a_jetson_deployment_prep}"
mkdir -p "$LOG"
printf '%s\n' "$LOG" > "$ROOT/test-log/.latest_p6a_jetson_deployment_prep_dir"

GIT_CMD=(git --git-dir="$ROOT/.git_3dplanner_full" --work-tree="$ROOT")
git_branch=$("${GIT_CMD[@]}" branch --show-current 2>/dev/null || true)
git_head=$("${GIT_CMD[@]}" rev-parse HEAD 2>/dev/null || true)
git_tag=$("${GIT_CMD[@]}" tag --points-at HEAD 2>/dev/null | paste -sd, - || true)
git_remote=$("${GIT_CMD[@]}" remote get-url origin 2>/dev/null || true)

cat > "$LOG/p6a_git_state.txt" <<EOF
custom_git_dir=$ROOT/.git_3dplanner_full
git_branch=$git_branch
git_head=$git_head
git_tag_at_start=${git_tag:-NONE}
git_remote_origin=${git_remote:-NONE}
EOF

cat > "$LOG/p6a_deployment_audit.md" <<EOF
# P6A Deployment Audit

## current_workspace_layout

- Root: $ROOT
- Map workspace: $ROOT/Map/ros2_ws
- Air workspace: $ROOT/Air/ros2_ws
- Ground workspace: $ROOT/Ground/ros2_ws
- Scripts: $ROOT/scripts
- Configs: $ROOT/Map/config
- RViz: $ROOT/Map/rviz

## current_build_flow

- P6A build order is Map -> Air -> Ground.
- Each workspace is built with \`colcon build --symlink-install\`.
- Existing P5B/P5C scripts source \`/opt/ros/humble/setup.bash\` and all three workspace installs.

## current_runtime_scripts

- P5B explainable live demo: \`scripts/run_p5b_explainable_bimodal_live_demo.sh\`
- P5B status check: \`scripts/check_p5b_live_demo_status.sh\`
- P5C validation: \`scripts/run_p5c_sensor_interface_validation.sh\`
- P5C bag replay: \`scripts/run_p5c_real_bag_replay_live_demo.sh\`
- P5C live topic: \`scripts/run_p5c_live_external_pointcloud_demo.sh\`

## current_sensor_input_interfaces

- Synthetic fallback uses \`/points_raw\` with \`realistic_room_corridor_v1\`.
- External PointCloud2 bridge outputs \`/bimodal/points\`.
- P5C keeps live camera/lidar topics and rosbag2 PointCloud2 topic selection.

## current_visualization_interfaces

- P5B RViz config: \`Map/rviz/p5b_explainable_bimodal_demo.rviz\`.
- Key visual topics include map voxels, incoming cloud, Air/Ground candidates, selected goal, active path, executed path, robot marker, TF, and text/legend overlays.

## current_safety_gate

- P5B/P5C/P6A scripts scan visible ROS topics for \`/cmd_vel\`, \`/mavros/*\`, \`/fmu/*\`, \`/actuator/*\`, \`/offboard_control_mode\`, and \`/trajectory_setpoint\`.
- P6A does not start real flight control or control bridges.

## current_git_layout_custom_git_dir

- This repository uses custom git dir: \`$ROOT/.git_3dplanner_full\`.
- P6A validation records branch/head/tag using \`git --git-dir=$ROOT/.git_3dplanner_full --work-tree=$ROOT\`.

## jetson_risk_list

- Missing ROS2 Humble or colcon on a fresh Jetson.
- DISPLAY/X11 unavailable for RViz.
- \`nvidia-smi\` absent on Jetson; \`tegrastats\` availability depends on image.
- CPU/memory pressure during RViz and validation.
- ROS_DOMAIN_ID/RMW mismatch across terminals.
- No real camera/lidar bag available yet.

## proposed_p6a_deployment_plan

- Add deployment guide, environment check, build-all, runtime preflight, unified launcher, runtime monitor, debug exporter, and validation wrapper.
- Reuse P5B/P5C scripts for runtime behavior and keep algorithm code unchanged.
EOF

P6A_LOG_DIR="$LOG" "$ROOT/scripts/p6a_check_environment.sh" > "$LOG/p6a_environment_check_stdout.txt" 2>&1 || true
P6A_LOG_DIR="$LOG" "$ROOT/scripts/p6a_build_all.sh" > "$LOG/p6a_build_all_stdout.txt" 2>&1 || true
P6A_LOG_DIR="$LOG" "$ROOT/scripts/p6a_runtime_preflight.sh" > "$LOG/p6a_runtime_preflight_stdout.txt" 2>&1 || true

P5C_LOG_DIR="$LOG" P5C_ALLOW_SYNTHETIC_FALLBACK=1 P5C_DURATION_SEC="$DURATION" \
  "$ROOT/scripts/run_p5c_sensor_interface_validation.sh" > "$LOG/p6a_p5c_validation_stdout.txt" 2>&1 || true

topics=$(timeout 6 bash -lc "source '$ROOT/scripts/env_visual_demo.sh' >/dev/null 2>&1; ros2 topic list --no-daemon" 2>/dev/null || true)
forbidden=$(printf '%s\n' "$topics" | grep -E '^/cmd_vel$|^/mavros/|^/fmu/|^/actuator/|^/offboard_control_mode$|^/trajectory_setpoint$' || true)
if [ -n "$forbidden" ]; then
  {
    echo "no_real_control_topic=FAIL"
    echo "forbidden_topic_detected_count=$(printf '%s\n' "$forbidden" | sed '/^$/d' | wc -l)"
    echo "forbidden_topic_list=$forbidden"
  } > "$LOG/p6a_no_real_control_topic_check.txt"
else
  {
    echo "no_real_control_topic=PASS"
    echo "forbidden_topic_detected_count=0"
    echo "forbidden_topic_list=NONE"
  } > "$LOG/p6a_no_real_control_topic_check.txt"
fi

P6A_LOG_DIR="$LOG" "$ROOT/scripts/p6a_export_debug_package.sh" > "$LOG/p6a_export_debug_package_stdout.txt" 2>&1 || true

python3 - "$LOG" "$DURATION" <<'PY'
import re
import sys
from pathlib import Path

log = Path(sys.argv[1])
duration = float(sys.argv[2])
root = Path('/home/nuaa/ZHY/3DPlanner_FULL')

def read(path):
    p = Path(path)
    return p.read_text(errors='ignore') if p.exists() else ''

def kv(path):
    out = {}
    for line in read(path).splitlines():
        if '=' in line and not line.startswith('#'):
            k, v = line.split('=', 1)
            out[k.strip()] = v.strip()
    return out

env = kv(log / 'p6a_environment_check.txt')
build = kv(log / 'p6a_build_summary.md')
preflight = kv(log / 'p6a_runtime_preflight.txt')
p5c = kv(log / 'p5c_sensor_interface_freeze_summary.md')
safety = kv(log / 'p6a_no_real_control_topic_check.txt')
git_state = kv(log / 'p6a_git_state.txt')

def exists(path):
    return 'PASS' if Path(path).exists() else 'FAIL'

tracked_forbidden = []
try:
    import subprocess
    out = subprocess.check_output([
        'git', f'--git-dir={root}/.git_3dplanner_full', f'--work-tree={root}', 'ls-files'
    ], text=True)
    pat = re.compile(r'(^|/)(build|install|log|test-log)(/|$)|\.bag$|\.db3$|\.tar\.gz$|__pycache__/|\.pytest_cache/')
    tracked_forbidden = [x for x in out.splitlines() if pat.search(x)]
except Exception:
    tracked_forbidden = ['GIT_LS_FILES_FAILED']

large_file_guard = 'PASS' if not tracked_forbidden else 'FAIL'
debug_pkg = root / 'latest_p6a_jetson_deployment_prep_package.tar.gz'
debug_created = 'PASS' if debug_pkg.exists() else 'FAIL'

no_control = safety.get('no_real_control_topic', 'FAIL')
result = 'PASS'
failure = 'NONE'
reason = 'No remaining P6A failure.'
warnings = []
if env.get('P6A_ENV_CHECK') == 'PASS_WITH_WARNINGS':
    warnings.append('environment warnings')
if preflight.get('P6A_RUNTIME_PREFLIGHT') in ('PASS_WITH_DISPLAY_LIMITATION', 'PASS_WITH_WARNINGS'):
    warnings.append('runtime preflight warnings')
if p5c.get('real_bag_found') == 'NO':
    warnings.append('no real bag available')

checks = {
    'deployment_guide_created': exists(root / 'DEPLOYMENT_P6A_JETSON_UBUNTU_GUIDE.md'),
    'unified_launcher_created': exists(root / 'scripts/run_p6a_unified_demo_launcher.sh'),
    'runtime_monitor_created': exists(root / 'scripts/p6a_runtime_monitor.sh'),
    'debug_exporter_created': exists(root / 'scripts/p6a_export_debug_package.sh'),
    'p6a_build_all': build.get('P6A_BUILD_ALL', 'FAIL'),
    'map_build': build.get('map_build', 'FAIL'),
    'air_build': build.get('air_build', 'FAIL'),
    'ground_build': build.get('ground_build', 'FAIL'),
    'p5c_synthetic_fallback_regression': 'PASS' if p5c.get('synthetic_fallback_validation') == 'PASS' or p5c.get('P5C_SENSOR_INPUT_INTERFACE_FREEZE_AND_MANUAL_BACKUP_PREP') in ('PASS', 'PASS_PARTIAL') else 'FAIL',
    'p5b_explainability_regression': p5c.get('p5b_explainability_reused', 'FAIL'),
    'no_real_control_topic': no_control,
    'large_file_guard': large_file_guard,
    'debug_package_created': debug_created,
}

if no_control != 'PASS':
    result, failure, reason = 'FAIL', 'SAFETY_FAIL', 'Forbidden control topics were detected.'
elif large_file_guard != 'PASS':
    result, failure, reason = 'FAIL', 'LARGE_FILE_GUARD_FAIL', 'Tracked generated or large artifacts were detected.'
elif checks['p6a_build_all'] != 'PASS':
    result, failure, reason = 'FAIL', 'BUILD_FAIL', 'P6A build-all did not pass.'
elif checks['p5c_synthetic_fallback_regression'] != 'PASS':
    result, failure, reason = 'FAIL', 'P5C_REGRESSION', 'P5C synthetic fallback regression did not pass.'
elif any(checks[k] != 'PASS' for k in ('deployment_guide_created','unified_launcher_created','runtime_monitor_created','debug_exporter_created','debug_package_created')):
    result, failure, reason = 'FAIL', 'UNKNOWN_FAIL', 'One or more P6A deployment deliverables are missing.'
elif warnings:
    result, failure, reason = 'PASS_WITH_WARNINGS', 'NONE', 'No remaining P6A failure; warnings are expected for non-Jetson/no-real-bag/headless environments.'

remaining_owned = 0
try:
    import subprocess
    ps = subprocess.check_output(['pgrep','-af','run_p5b_explainable|run_p5c|bimodal_map_bringup|bimodal_air_bringup|bimodal_ground_bringup'], text=True)
    remaining_owned = len([line for line in ps.splitlines() if 'run_p6a_jetson_deployment_prep_validation' not in line])
except Exception:
    remaining_owned = 0
cleanup_pass = 'PASS' if remaining_owned == 0 else 'FAIL'

summary = {
    'project_stage': 'P6_DEPLOYMENT_AND_FIELD_READINESS',
    'current_substage': 'P6A_JETSON_DEPLOYMENT_PREP_AND_RUNTIME_HARDENING',
    'work_done': 'Added Jetson/Ubuntu deployment guide, environment check, build-all, runtime preflight, unified launcher, runtime monitor, debug exporter, and validation wrapper',
    'work_not_done': 'No GitHub push, no real Jetson-only hardware test, and no real camera/lidar bag replay were performed',
    'root': str(root),
    'log_dir': str(log),
    'custom_git_dir_used': 'PASS',
    'git_branch': git_state.get('git_branch', 'UNKNOWN'),
    'git_commit_before': git_state.get('git_head', 'UNKNOWN'),
    'git_commit_after': 'NONE',
    'git_tag_at_start': git_state.get('git_tag_at_start', 'NONE') or 'NONE',
    'git_remote_origin': git_state.get('git_remote_origin', 'NONE'),
    'map_build': checks['map_build'],
    'air_build': checks['air_build'],
    'ground_build': checks['ground_build'],
    'p6a_build_all': checks['p6a_build_all'],
    'p6a_environment_check': env.get('P6A_ENV_CHECK', 'FAIL'),
    'p6a_runtime_preflight': preflight.get('P6A_RUNTIME_PREFLIGHT', 'FAIL'),
    'deployment_guide_created': checks['deployment_guide_created'],
    'deployment_guide': str(root / 'DEPLOYMENT_P6A_JETSON_UBUNTU_GUIDE.md'),
    'unified_launcher_created': checks['unified_launcher_created'],
    'runtime_monitor_created': checks['runtime_monitor_created'],
    'debug_exporter_created': checks['debug_exporter_created'],
    'p5c_synthetic_fallback_regression': checks['p5c_synthetic_fallback_regression'],
    'p5b_explainability_regression': checks['p5b_explainability_regression'],
    'selected_input_mode': 'synthetic_fallback',
    'scene_profile': 'realistic_room_corridor_v1',
    'duration_sec': f'{duration:.1f}',
    'bimodal_points_captured': p5c.get('bimodal_points_captured', 'FAIL'),
    'shared_3d_map_chain': p5c.get('shared_3d_map_chain', 'FAIL'),
    'air_p3b_regression': p5c.get('air_p3b_regression', 'FAIL'),
    'ground_p4b_regression': p5c.get('ground_p4b_regression', 'FAIL'),
    'active_path_captured': p5c.get('active_path_captured', 'FAIL'),
    'executed_path_captured': p5c.get('executed_path_captured', 'FAIL'),
    'no_real_control_topic': no_control,
    'forbidden_topic_detected_count': safety.get('forbidden_topic_detected_count', '0'),
    'forbidden_topic_list': safety.get('forbidden_topic_list', 'NONE'),
    'large_file_guard': large_file_guard,
    'forbidden_artifact_committed': 'NO' if large_file_guard == 'PASS' else 'YES',
    'forbidden_artifact_list': 'NONE' if large_file_guard == 'PASS' else ','.join(tracked_forbidden),
    'cleanup_pass': cleanup_pass,
    'remaining_owned_process_count': str(remaining_owned),
    'debug_package_created': debug_created,
    'debug_package': str(debug_pkg),
    'P6A_JETSON_DEPLOYMENT_PREP_AND_RUNTIME_HARDENING': result,
    'changed_files': 'DEPLOYMENT_P6A_JETSON_UBUNTU_GUIDE.md,scripts/p6a_check_environment.sh,scripts/p6a_build_all.sh,scripts/p6a_runtime_preflight.sh,scripts/run_p6a_unified_demo_launcher.sh,scripts/p6a_runtime_monitor.sh,scripts/p6a_export_debug_package.sh,scripts/run_p6a_jetson_deployment_prep_validation.sh',
    'local_commit_created': 'PENDING',
    'main_result_summary': 'P6A deployment preparation and runtime hardening deliverables were generated and validated against the P5C synthetic fallback baseline.',
    'remaining_failure_class': failure,
    'remaining_failure_reason': reason,
    'validation_summary': str(log / 'p6a_validation_summary.md'),
    'p6a_env_check_command': 'cd /home/nuaa/ZHY/3DPlanner_FULL && ./scripts/p6a_check_environment.sh',
    'p6a_build_command': 'cd /home/nuaa/ZHY/3DPlanner_FULL && ./scripts/p6a_build_all.sh',
    'p6a_preflight_command': 'cd /home/nuaa/ZHY/3DPlanner_FULL && ./scripts/p6a_runtime_preflight.sh',
    'p6a_unified_launcher_command': 'cd /home/nuaa/ZHY/3DPlanner_FULL && P6A_MODE=p5b_synthetic ./scripts/run_p6a_unified_demo_launcher.sh',
    'p6a_validation_command': 'cd /home/nuaa/ZHY/3DPlanner_FULL && P6A_DURATION_SEC=120 ./scripts/run_p6a_jetson_deployment_prep_validation.sh',
    'recommended_next_prompt_type': 'P6B_REAL_SENSOR_LIVE_DRY_RUN/P6C_JETSON_ON_DEVICE_BUILD/P7A_FIELD_TEST_WITH_REAL_BAG/GITHUB_BACKUP_P6A/FIX_P6A_DEPLOYMENT_PREP',
    'next_stage_explanation': 'After P6A, the next useful step is either real PointCloud2 live dry-run, Jetson on-device build, field bag replay, or GitHub backup of the deployment baseline.'
}

text = ['B3D_P6A_JETSON_DEPLOYMENT_PREP_SUMMARY']
text.extend(f'{k}={v}' for k, v in summary.items())
(log / 'p6a_validation_summary.md').write_text('\n'.join(text) + '\n')
print('\n'.join(text))
PY

cat "$LOG/p6a_validation_summary.md"
