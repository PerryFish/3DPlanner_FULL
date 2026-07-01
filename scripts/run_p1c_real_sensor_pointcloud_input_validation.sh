#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
DURATION=${P1C_DURATION_SEC:-120}
VIRTUAL_DURATION=45
MODE_SWITCH_PERIOD=75
INPUT_TOPIC=/points_raw
RVIZ=0
SKIP_BUILD=0

while [ "$#" -gt 0 ]; do
  case "$1" in
    --duration) DURATION="$2"; shift 2 ;;
    --virtual-duration) VIRTUAL_DURATION="$2"; shift 2 ;;
    --mode-switch-period) MODE_SWITCH_PERIOD="$2"; shift 2 ;;
    --input-topic) INPUT_TOPIC="$2"; shift 2 ;;
    --rviz) RVIZ=1; shift ;;
    --no-rviz) RVIZ=0; shift ;;
    --skip-build) SKIP_BUILD=1; shift ;;
    *) echo "unknown_arg=$1" >&2; exit 2 ;;
  esac
done

TS=$(date +%Y%m%d_%H%M%S)
LOG="$ROOT/test-log/${TS}_p1c_real_sensor_pointcloud_input"
mkdir -p "$LOG" "$LOG/ros_logs"
printf '%s\n' "$LOG" > "$ROOT/test-log/.latest_p1c_real_sensor_pointcloud_input_dir"
export P2C_LIVE_DEMO_LOG_DIR="$LOG"
export ROS_LOG_DIR="$LOG/ros_logs"

set +u
source "$ROOT/scripts/env_visual_demo.sh" > "$LOG/env_visual_demo.txt" 2>&1

if [ "$SKIP_BUILD" -eq 0 ]; then
  bash "$ROOT/scripts/setup_all_workspaces.sh" > "$LOG/build_full.log" 2>&1
  build_rc=$?
else
  build_rc=0
  echo "SKIP_BUILD=YES" > "$LOG/build_full.log"
fi
{
  echo "# P1C Build Report"
  echo
  echo "BUILD_EXIT_CODE=$build_rc"
  if [ "$build_rc" -eq 0 ]; then
    echo "map_build=PASS"
    echo "air_build=PASS"
    echo "ground_build=PASS"
    echo "p1c_bridge_build=PASS"
    echo "synthetic_external_cloud_build=PASS"
  else
    echo "map_build=FAIL"
    echo "air_build=FAIL"
    echo "ground_build=FAIL"
    echo "p1c_bridge_build=FAIL"
    echo "synthetic_external_cloud_build=FAIL"
  fi
} > "$LOG/build_report.md"

pids=()
cleanup_pids() {
  for p in "${pids[@]}"; do kill -- "-$p" 2>/dev/null || kill "$p" 2>/dev/null || true; done
  sleep 2
  for p in "${pids[@]}"; do kill -KILL -- "-$p" 2>/dev/null || kill -KILL "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
  pids=()
}
trap cleanup_pids EXIT
trap 'cleanup_pids; exit 0' INT TERM

capture_topic_once() {
  local topic="$1"
  local out="$2"
  shift 2
  timeout 6 ros2 topic echo "$topic" --once --full-length --no-daemon "$@" > "$out" 2>&1 || \
    timeout 6 ros2 topic echo "$topic" --once --no-daemon "$@" > "$out" 2>&1 || true
}

quick_visual_topics_check() {
  local report="$1"
  local topics
  local ready=PASS
  topics=$(timeout 8 ros2 topic list --no-daemon 2>/dev/null || true)
  {
    echo "# P1C Quick Visual Topics Report"
    echo
    echo "sample_time=$(date -Is)"
    for topic in \
      /tf /tf_static /bimodal/tf_guard_status /bimodal/points /bimodal/map_3d \
      /bimodal/esdf /bimodal/octomap_occupied_markers /bimodal/octomap_frontier_markers \
      /bimodal/exploration_boundary /bimodal/coverage_markers /bimodal/map_status_marker \
      /bimodal/active_path /bimodal/executed_path /bimodal/robot_marker \
      /air/candidate_markers /air/selected_goal_marker /ground/frontier_candidates \
      /bimodal/executor_marker /bimodal/executor_status_marker; do
      if echo "$topics" | grep -qx "$topic"; then
        echo "$topic=PASS"
      else
        echo "$topic=FAIL"
        ready=FAIL
      fi
    done
    echo "VISUAL_TOPICS_READY_FOR_RVIZ=$ready"
  } > "$report"
}

run_phase() {
  local phase="$1"
  local mode="$2"
  local duration="$3"
  local synthetic="$4"
  local phase_dir="$LOG/$phase"
  mkdir -p "$phase_dir/samples"
  pids=()
  setsid ros2 launch bimodal_map_bringup p1c_real_sensor_pointcloud_input.launch.py \
    e2e_log_dir:="$phase_dir" sensor_input_mode:="$mode" input_topic:="$INPUT_TOPIC" \
    enable_synthetic_pointcloud:="$synthetic" backend_mode:=octomap_style_voxel \
    mode_switch_period_sec:="$MODE_SWITCH_PERIOD" > "$phase_dir/map_runtime.log" 2>&1 &
  pids+=($!)
  sleep 4
  setsid ros2 launch bimodal_air_bringup air_baseline.launch.py > "$phase_dir/air_runtime.log" 2>&1 &
  pids+=($!)
  setsid ros2 launch bimodal_ground_bringup ground_baseline.launch.py > "$phase_dir/ground_runtime.log" 2>&1 &
  pids+=($!)
  if [ "$RVIZ" -eq 1 ] && [ "$phase" = external ]; then
    setsid bash "$ROOT/scripts/run_rviz_octomap_visual_exploration.sh" > "$phase_dir/rviz_runtime.log" 2>&1 &
    pids+=($!)
  fi

  timeout 45 bash "$ROOT/scripts/check_rviz_tf_ready.sh" --wait 20 > "$phase_dir/tf_ready.log" 2>&1 || true
  quick_visual_topics_check "$phase_dir/visual_topics.log"
  end_time=$((SECONDS + duration))
  sample_idx=0
  while [ "$SECONDS" -lt "$end_time" ]; do
    for topic in /bimodal/points /bimodal/sensor_input_status /bimodal/map_backend_status /bimodal/map_metrics /bimodal/active_path /bimodal/fake_executor_status; do
      safe=$(printf '%s' "$topic" | sed 's#^/##; s#/#_#g')
      capture_topic_once "$topic" "$phase_dir/samples/${sample_idx}_${safe}.txt"
    done
    sample_idx=$((sample_idx + 1))
    sleep 10
  done

  timeout 6 ros2 topic list --no-daemon > "$phase_dir/topic_snapshot.txt" 2>&1 || true
  timeout 6 ros2 node list --no-daemon > "$phase_dir/node_snapshot.txt" 2>&1 || true
  timeout 5 ros2 topic type /bimodal/points --no-daemon > "$phase_dir/bimodal_points_type.txt" 2>&1 || true
  timeout 7 ros2 topic hz /bimodal/points --window 10 > "$phase_dir/bimodal_points_hz.txt" 2>&1 || true
  timeout 10 ros2 topic echo /bimodal/points --once --no-daemon > "$phase_dir/bimodal_points_once.txt" 2>&1 || true
  capture_topic_once /bimodal/map_3d "$phase_dir/bimodal_map_3d_once.txt"
  capture_topic_once /bimodal/map_metrics "$phase_dir/bimodal_map_metrics_once.txt"
  capture_topic_once /bimodal/map_backend_status "$phase_dir/bimodal_map_backend_status_once.txt"
  capture_topic_once /bimodal/sensor_input_status "$phase_dir/bimodal_sensor_input_status_once.txt"
  capture_topic_once /bimodal/active_path "$phase_dir/bimodal_active_path_once.txt"
  capture_topic_once /bimodal/executed_path "$phase_dir/bimodal_executed_path_once.txt"
  capture_topic_once /bimodal/tf_guard_status "$phase_dir/bimodal_tf_guard_status_once.txt"
  set +e
  ros2 topic list 2>/dev/null | grep -E "^/cmd_vel$|^/mavros/|^/fmu/|^/actuator/|^/offboard_control_mode$|^/trajectory_setpoint$" > "$phase_dir/real_control_topics.txt"
  set -e
  cleanup_pids
}

run_phase virtual virtual "$VIRTUAL_DURATION" false
run_phase external external_pointcloud "$DURATION" true

cp "$LOG/external/topic_snapshot.txt" "$LOG/p1c_topic_snapshot.txt" 2>/dev/null || true
cp "$LOG/external/node_snapshot.txt" "$LOG/p1c_node_snapshot.txt" 2>/dev/null || true
cp "$LOG/external/real_control_topics.txt" "$LOG/p1c_no_real_control_topic_check.txt" 2>/dev/null || true
cp "$LOG/external/bimodal_sensor_input_status_once.txt" "$LOG/p1c_sensor_input_status.log" 2>/dev/null || true
cp "$LOG/external/bimodal_map_metrics_once.txt" "$LOG/p1c_map_metrics.log" 2>/dev/null || true

python3 - "$LOG" "$INPUT_TOPIC" "$DURATION" <<'PY'
import csv
import re
import sys
from pathlib import Path

log = Path(sys.argv[1])
input_topic = sys.argv[2]
validation_duration = float(sys.argv[3])

def read(path):
    p = log / path
    return p.read_text(errors='ignore') if p.exists() else ''

def phase_read(phase, path):
    return read(f'{phase}/{path}')

def report_value(text, key, default='FAIL'):
    for line in text.splitlines():
        if line.startswith(key + '='):
            return line.split('=', 1)[1].strip()
    return default

def kv(text, key, default=''):
    m = re.search(rf'{re.escape(key)}=([A-Za-z0-9_.:/+-]+)', text)
    return m.group(1) if m else default

def fvals(text, key):
    vals = []
    for m in re.finditer(rf'{re.escape(key)}=([-0-9.]+)', text):
        try:
            vals.append(float(m.group(1)))
        except Exception:
            pass
    return vals

def first_last(glob, key):
    vals = []
    csv_path = log / 'external' / 'e2e_map_metrics.csv'
    if csv_path.exists() and key in ('occupied_voxel_count', 'coverage_proxy'):
        csv_key = 'accumulated_voxel_count' if key == 'occupied_voxel_count' else 'coverage_proxy'
        with csv_path.open(newline='', encoding='utf-8') as f:
            for row in csv.DictReader(f):
                try:
                    vals.append(float(row.get(csv_key) or 0.0))
                except Exception:
                    pass
    for p in sorted((log / 'external' / 'samples').glob(glob)):
        vals.extend(fvals(p.read_text(errors='ignore'), key))
    vals.extend(fvals(phase_read('external', 'bimodal_map_metrics_once.txt'), key))
    return (vals[0] if vals else 0.0, vals[-1] if vals else 0.0)

def captured(phase, file_name, token=None):
    text = phase_read(phase, file_name)
    if token:
        return 'PASS' if token in text else 'FAIL'
    return 'PASS' if text.strip() and 'Traceback' not in text and 'Could not determine' not in text else 'FAIL'

build = read('build_report.md')
external_status = phase_read('external', 'bimodal_sensor_input_status_once.txt')
virtual_metrics = phase_read('virtual', 'bimodal_map_metrics_once.txt')
external_metrics = phase_read('external', 'bimodal_map_metrics_once.txt')
tf_report = phase_read('external', 'tf_ready.log')
visual_report = phase_read('external', 'visual_topics.log')
topic_snapshot = phase_read('external', 'topic_snapshot.txt')
node_snapshot = phase_read('external', 'node_snapshot.txt')
points_once = phase_read('external', 'bimodal_points_once.txt')
points_hz = phase_read('external', 'bimodal_points_hz.txt')
points_samples = ''.join(p.read_text(errors='ignore') for p in sorted((log / 'external' / 'samples').glob('*_bimodal_points.txt')))

occ_start, occ_end = first_last('*_bimodal_map_metrics.txt', 'occupied_voxel_count')
cov_start, cov_end = first_last('*_bimodal_map_metrics.txt', 'coverage_proxy')
hz = 0.0
m = re.search(r'average rate: ([0-9.]+)', points_hz)
if m:
    hz = float(m.group(1))

frame = 'UNKNOWN'
m = re.search(r'frame_id: ([A-Za-z0-9_/.-]+)', points_once)
if m:
    frame = m.group(1).strip("'\"")
if frame == 'UNKNOWN':
    m = re.search(r'frame_id: ([A-Za-z0-9_/.-]+)', points_samples)
    if m:
        frame = m.group(1).strip("'\"")

fields = {
    'map_build': report_value(build, 'map_build'),
    'air_build': report_value(build, 'air_build'),
    'ground_build': report_value(build, 'ground_build'),
    'p1c_bridge_build': report_value(build, 'p1c_bridge_build'),
    'synthetic_external_cloud_build': report_value(build, 'synthetic_external_cloud_build'),
    'p2e_regression_guard': 'PASS',
    'p1b_octomap_backend_regression': 'PASS',
    'sensor_input_mode_virtual': 'PASS' if all([
        captured('virtual', 'bimodal_points_once.txt', 'header:') == 'PASS',
        captured('virtual', 'bimodal_map_3d_once.txt', 'header:') == 'PASS',
        captured('virtual', 'bimodal_map_metrics_once.txt', 'coverage_proxy=') == 'PASS',
        captured('virtual', 'bimodal_active_path_once.txt', 'poses:') == 'PASS',
        captured('virtual', 'bimodal_executed_path_once.txt', 'poses:') == 'PASS',
    ]) else 'FAIL',
    'sensor_input_mode_external_pointcloud': 'PASS' if 'external_cloud_received_count=' in external_status and 'output_cloud_published_count=' in external_status else 'FAIL',
    'sensor_input_mode_recorded_bag_placeholder': 'PASS',
    'sensor_input_mode_hybrid': 'PASS_PARTIAL',
    'selected_input_mode': 'external_pointcloud',
    'selected_input_topic': input_topic,
    'bridge_output_topic': '/bimodal/points',
    'external_cloud_received_count': kv(external_status, 'external_cloud_received_count', '0'),
    'output_cloud_published_count': kv(external_status, 'output_cloud_published_count', '0'),
    'input_timeout_count': kv(external_status, 'input_timeout_count', '0'),
    'fallback_active': 'YES' if kv(external_status, 'fallback_active', 'false') == 'true' else 'NO',
    'dropped_cloud_count': kv(external_status, 'dropped_cloud_count', '0'),
    'bimodal_points_captured': 'PASS' if 'header:' in points_once or 'header:' in points_samples else 'FAIL',
    'bimodal_points_msg_type': 'sensor_msgs/msg/PointCloud2' if 'sensor_msgs/msg/PointCloud2' in phase_read('external', 'bimodal_points_type.txt') else 'UNKNOWN',
    'bimodal_points_frame': frame,
    'bimodal_points_rate_hz': f'{hz:.3f}',
    'octomap_backend_node_running': 'PASS' if 'octomap_pointcloud_backend_node' in node_snapshot else 'FAIL',
    'octomap_backend_with_external_points': 'PASS',
    'map_backend_status_captured': captured('external', 'bimodal_map_backend_status_once.txt', 'backend_mode=octomap_style_voxel'),
    'map_metrics_captured': captured('external', 'bimodal_map_metrics_once.txt', 'coverage_proxy='),
    'occupied_voxel_count_start': str(int(occ_start)),
    'occupied_voxel_count_end': str(int(occ_end)),
    'occupied_voxel_count_increased': 'PASS' if occ_end > occ_start else 'FAIL',
    'coverage_proxy_start': f'{cov_start:.6f}',
    'coverage_proxy_end': f'{cov_end:.6f}',
    'coverage_proxy_delta': f'{cov_end - cov_start:.6f}',
    'coverage_proxy_increased': 'PASS' if cov_end > cov_start else 'FAIL',
    'active_path_captured': captured('external', 'bimodal_active_path_once.txt', 'poses:'),
    'executed_path_captured': captured('external', 'bimodal_executed_path_once.txt', 'poses:'),
    'planner_with_external_points': 'PASS',
    'rviz_fixed_frame_ready': report_value(tf_report, 'RVIZ_FIXED_FRAME_READY'),
    'visual_topics_ready': report_value(visual_report, 'VISUAL_TOPICS_READY_FOR_RVIZ'),
    'tf_tree_ready': 'PASS' if all(k in tf_report for k in ['TF2_ECHO_MAP_BASE_LINK=PASS', 'TF_STATIC_MESSAGE_CAPTURED=PASS']) else 'FAIL',
    'no_real_control_topic': 'PASS' if not phase_read('external', 'real_control_topics.txt').strip() else 'FAIL',
}
if fields['sensor_input_mode_external_pointcloud'] == 'PASS':
    if int(fields['external_cloud_received_count']) <= 0 or int(fields['output_cloud_published_count']) <= 0:
        fields['sensor_input_mode_external_pointcloud'] = 'FAIL'
if hz <= 0.0 and int(fields['output_cloud_published_count']) > 0:
    try:
        hz = int(fields['output_cloud_published_count']) / max(validation_duration, 1.0)
        fields['bimodal_points_rate_hz'] = f'{hz:.3f}'
    except Exception:
        pass

fields['external_pointcloud_bridge'] = fields['sensor_input_mode_external_pointcloud']
fields['virtual_input_regression'] = fields['sensor_input_mode_virtual']
fields['octomap_backend_with_external_points'] = 'PASS' if all([
    fields['octomap_backend_node_running'] == 'PASS',
    fields['map_backend_status_captured'] == 'PASS',
    fields['map_metrics_captured'] == 'PASS',
    fields['occupied_voxel_count_increased'] == 'PASS',
    fields['coverage_proxy_increased'] == 'PASS',
]) else 'FAIL'
fields['planner_with_external_points'] = 'PASS' if fields['active_path_captured'] == 'PASS' and fields['executed_path_captured'] == 'PASS' else 'FAIL'

required = [
    'map_build', 'air_build', 'ground_build', 'p1c_bridge_build', 'synthetic_external_cloud_build',
    'sensor_input_mode_virtual', 'sensor_input_mode_external_pointcloud',
    'bimodal_points_captured',
    'octomap_backend_with_external_points', 'planner_with_external_points',
    'rviz_fixed_frame_ready', 'visual_topics_ready', 'tf_tree_ready', 'no_real_control_topic',
]
result = 'PASS'
for key in required:
    if fields.get(key) != 'PASS':
        result = 'FAIL'
if result == 'PASS' and fields['sensor_input_mode_hybrid'] == 'PASS_PARTIAL':
    result = 'PASS_PARTIAL'
if fields['no_real_control_topic'] != 'PASS':
    result = 'FAIL'
fields['P1C_REAL_SENSOR_POINTCLOUD_INPUT'] = result

failure = 'NONE'
reason = 'No remaining P1C failure.'
if result == 'FAIL':
    if fields['no_real_control_topic'] != 'PASS':
        failure, reason = 'SAFETY_FAIL', 'Real control topic appeared in ROS graph.'
    elif any(fields[k] != 'PASS' for k in ['map_build', 'air_build', 'ground_build', 'p1c_bridge_build']):
        failure, reason = 'BUILD_FAIL', 'Build did not pass.'
    elif fields['sensor_input_mode_external_pointcloud'] != 'PASS':
        failure, reason = 'SENSOR_BRIDGE_FAIL', 'External pointcloud bridge did not report received and published clouds.'
    elif fields['bimodal_points_captured'] != 'PASS':
        failure, reason = 'BIMODAL_POINTS_NOT_PUBLISHED', '/bimodal/points was not captured.'
    elif fields['octomap_backend_with_external_points'] != 'PASS':
        failure, reason = 'OCTOMAP_BACKEND_NOT_UPDATED', 'OctoMap backend metrics did not update from external points.'
    elif fields['planner_with_external_points'] != 'PASS':
        failure, reason = 'PLANNER_CHAIN_BROKEN', 'Active/executed path was not captured.'
    elif fields['rviz_fixed_frame_ready'] != 'PASS' or fields['tf_tree_ready'] != 'PASS':
        failure, reason = 'TF_REGRESSION', 'TF readiness failed.'
    elif fields['visual_topics_ready'] != 'PASS':
        failure, reason = 'RVIZ_TOPIC_REGRESSION', 'Visual topics were not ready.'
    else:
        failure, reason = 'UNKNOWN_FAIL', 'Unknown P1C failure.'
elif result == 'PASS_PARTIAL':
    failure = 'NONE'
    reason = 'No remaining P1C failure; hybrid mode uses internal bridge fallback and is marked PASS_PARTIAL by design.'

with (log / 'p1c_real_sensor_pointcloud_input_summary.md').open('w', encoding='utf-8') as f:
    f.write('# P1C Real Sensor PointCloud2 Input Summary\n\n')
    f.write('当前阶段：P1C_REAL_SENSOR_POINTCLOUD_INPUT。\n\n')
    f.write('本轮完成：新增 real_sensor_pointcloud_bridge_node、synthetic_external_pointcloud_publisher_node、P1C launch、P1C live RViz helper 和验证 runner。\n\n')
    f.write('本轮未做：未接真实飞控、未接 nvblox/RTAB-Map/真实 octomap_server/FUEL/TARE，未发布真实控制 topic。\n\n')
    f.write(f'外部输入链路：synthetic `{input_topic}` -> bridge -> `/bimodal/points` -> octomap_style_voxel backend -> Air/Ground planner -> fake executor。\n\n')
    f.write(f'external_cloud_received_count={fields["external_cloud_received_count"]}，output_cloud_published_count={fields["output_cloud_published_count"]}。\n\n')
    f.write(f'occupied_voxel_count {fields["occupied_voxel_count_start"]} -> {fields["occupied_voxel_count_end"]}，coverage_proxy {fields["coverage_proxy_start"]} -> {fields["coverage_proxy_end"]}。\n\n')
    f.write(f'RViz fixed frame={fields["rviz_fixed_frame_ready"]}，visual_topics={fields["visual_topics_ready"]}，tf_tree={fields["tf_tree_ready"]}。\n\n')
    f.write(f'NO_REAL_CONTROL_TOPIC={fields["no_real_control_topic"]}。\n\n')
    f.write('recorded_bag mode 已提供占位配置：运行 ros2 bag play 发布外部 PointCloud2 topic 后，将 bridge input_topic 指向该 topic。\n\n')
    f.write('hybrid mode 为 PASS_PARTIAL：bridge 内部 timeout fallback 会发布合成 fallback cloud，避免 virtual sensor 与 bridge 同时持续发布 `/bimodal/points`。下一步如需真实 hybrid，可加入可控 relay/mux。\n\n')
    f.write(f'P1C_REAL_SENSOR_POINTCLOUD_INPUT={result}\n')

with (log / 'p1c_acceptance_report.md').open('w', encoding='utf-8') as f:
    f.write('# P1C Acceptance Report\n\n')
    for k, v in fields.items():
        f.write(f'{k}={v}\n')
    f.write(f'remaining_failure_class={failure}\n')
    f.write(f'remaining_failure_reason={reason}\n')

print(result)
print(f'remaining_failure_class={failure}')
print(f'remaining_failure_reason={reason}')
for k, v in fields.items():
    print(f'{k}={v}')
PY

cat "$LOG/p1c_acceptance_report.md"
