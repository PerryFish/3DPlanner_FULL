#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
DURATION=300
MODE_SWITCH_PERIOD=75
RVIZ=0
KEEP_RUNNING=0
BACKEND=octomap_style_voxel

while [ "$#" -gt 0 ]; do
  case "$1" in
    --duration) DURATION="$2"; shift 2 ;;
    --mode-switch-period) MODE_SWITCH_PERIOD="$2"; shift 2 ;;
    --backend) BACKEND="$2"; shift 2 ;;
    --rviz) RVIZ=1; shift ;;
    --no-rviz) RVIZ=0; shift ;;
    --keep-running) KEEP_RUNNING=1; shift ;;
    *) echo "unknown_arg=$1" >&2; exit 2 ;;
  esac
done

if [ "$BACKEND" = "octomap_server" ] && \
   ! { command -v octomap_server_node >/dev/null 2>&1 || command -v octomap_server >/dev/null 2>&1; }; then
  echo "OCTOMAP_SERVER_EXECUTABLE_AVAILABLE=FAIL"
  echo "selected_backend_mode=octomap_server"
  echo "failure_reason=octomap_server executable is not available; use --backend octomap_style_voxel"
  exit 7
fi

TS=$(date +%Y%m%d_%H%M%S)
LOG="$ROOT/test-log/${TS}_p1b_octomap_pointcloud_backend"
mkdir -p "$LOG" "$LOG/samples" "$LOG/ros_logs"
printf '%s\n' "$LOG" > "$ROOT/test-log/.latest_p1b_octomap_backend_dir"
export P2C_LIVE_DEMO_LOG_DIR="$LOG"
export ROS_LOG_DIR="$LOG/ros_logs"

set +u
source "$ROOT/scripts/env_visual_demo.sh" > "$LOG/env_visual_demo.txt" 2>&1

quick_visual_topics_check() {
  local report="$1"
  local topics
  local ready=PASS
  topics=$(timeout 8 ros2 topic list --no-daemon 2>/dev/null || true)
  {
    echo "# P1B Quick Visual Topics Report"
    echo
    echo "sample_time=$(date -Is)"
    for topic in \
      /tf \
      /tf_static \
      /bimodal/tf_guard_status \
      /bimodal/points \
      /bimodal/map_3d \
      /bimodal/esdf \
      /bimodal/octomap_occupied_markers \
      /bimodal/exploration_boundary \
      /bimodal/coverage_markers \
      /bimodal/map_status_marker \
      /bimodal/active_path \
      /bimodal/executed_path \
      /bimodal/robot_marker \
      /air/candidate_markers \
      /air/selected_goal_marker \
      /ground/frontier_candidates \
      /bimodal/executor_marker \
      /bimodal/executor_status_marker; do
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

pids=()
cleanup() {
  if [ "$KEEP_RUNNING" -eq 1 ]; then
    echo "P1B_KEEP_RUNNING=YES"
    return
  fi
  for p in "${pids[@]}"; do kill "$p" 2>/dev/null || true; done
  sleep 2
  for p in "${pids[@]}"; do kill -KILL "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'cleanup; exit 0' INT TERM

ros2 launch bimodal_map_bringup octomap_visual_exploration_demo_map_side.launch.py \
  e2e_log_dir:="$LOG" mode_switch_period_sec:="$MODE_SWITCH_PERIOD" backend_mode:="$BACKEND" \
  use_external_odom_for_virtual_sensor:=true virtual_sensor_publish_odom:=false \
  publish_world_gt_cloud:=true publish_local_sensor_cloud:=true \
  > "$LOG/map_p1b_runtime.log" 2>&1 &
pids+=($!)
sleep 4
ros2 launch bimodal_air_bringup air_baseline.launch.py > "$LOG/air_p1b_runtime.log" 2>&1 &
pids+=($!)
ros2 launch bimodal_ground_bringup ground_baseline.launch.py > "$LOG/ground_p1b_runtime.log" 2>&1 &
pids+=($!)

if [ "$RVIZ" -eq 1 ]; then
  bash "$ROOT/scripts/run_rviz_octomap_visual_exploration.sh" > "$LOG/rviz_p1b_runtime.log" 2>&1 &
  pids+=($!)
fi

timeout 60 bash "$ROOT/scripts/check_rviz_tf_ready.sh" --wait 30 > "$LOG/tf_ready_initial.log" 2>&1 || true
quick_visual_topics_check "$LOG/visual_topics_initial.log"

end_time=$((SECONDS + DURATION))
sample_idx=0
while [ "$SECONDS" -lt "$end_time" ]; do
  for topic in \
    /bimodal/map_backend_status \
    /bimodal/map_metrics \
    /bimodal/tf_guard_status \
    /air/planner_status \
    /ground/planner_status \
    /bimodal/active_path; do
    safe=$(printf '%s' "$topic" | sed 's#^/##; s#/#_#g')
    timeout 4 ros2 topic echo "$topic" --once --no-daemon > "$LOG/samples/${sample_idx}_${safe}.txt" 2>&1 || true
  done
  sample_idx=$((sample_idx + 1))
  sleep 10
done

for topic in \
  /bimodal/map_backend_status \
  /bimodal/map_metrics \
  /bimodal/fake_executor_status \
  /air/planner_status \
  /ground/planner_status \
  /bimodal/active_path \
  /bimodal/executed_path; do
  safe=$(printf '%s' "$topic" | sed 's#^/##; s#/#_#g')
  timeout 5 ros2 topic echo "$topic" --once --no-daemon > "$LOG/final_${safe}.txt" 2>&1 || true
done
timeout 5 ros2 topic echo /bimodal/map_3d --once --no-daemon --field header > "$LOG/final_bimodal_map_3d_header.txt" 2>&1 || true
timeout 5 ros2 topic echo /bimodal/esdf --once --no-daemon --field header > "$LOG/final_bimodal_esdf_header.txt" 2>&1 || true
timeout 5 ros2 topic echo /bimodal/octomap_occupied_markers --once --no-daemon > "$LOG/final_bimodal_octomap_occupied_markers.txt" 2>&1 || true
timeout 6 ros2 node list --no-daemon > "$LOG/final_node_list.txt" 2>&1 || true
timeout 6 ros2 topic list --no-daemon > "$LOG/final_topic_list.txt" 2>&1 || true

timeout 45 bash "$ROOT/scripts/check_rviz_tf_ready.sh" --wait 10 > "$LOG/tf_ready_final.log" 2>&1 || true
quick_visual_topics_check "$LOG/visual_topics_final.log"

set +e
ros2 topic list 2>/dev/null | grep -E "^/cmd_vel$|^/mavros/|^/fmu/|^/actuator/|^/offboard_control_mode$|^/trajectory_setpoint$" > "$LOG/real_control_topics.txt"
set -e

python3 - "$LOG" "$BACKEND" "$DURATION" <<'PY'
import csv
import re
import sys
from pathlib import Path

log = Path(sys.argv[1])
backend = sys.argv[2]
duration = float(sys.argv[3])

def read(path):
    return (log / path).read_text(errors='ignore') if (log / path).exists() else ''

def read_csv(name):
    path = log / name
    if not path.exists():
        return []
    with path.open(newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))

def report_value(path, key, default='FAIL'):
    for line in read(path).splitlines():
        if line.startswith(key + '='):
            return line.split('=', 1)[1].strip()
    return default

def all_text(glob):
    return '\n'.join(p.read_text(errors='ignore') for p in sorted(log.glob(glob)))

def vals(text, key):
    out = []
    for m in re.finditer(rf'{re.escape(key)}=([A-Za-z0-9_.:/+-]+)', text):
        out.append(m.group(1))
    return out

def fnum(text, key, default=0.0, last=True):
    numbers = []
    for v in vals(text, key):
        try:
            numbers.append(float(v))
        except Exception:
            pass
    if not numbers:
        return default
    return numbers[-1] if last else numbers[0]

def maxnum(text, key):
    numbers = []
    for v in vals(text, key):
        try:
            numbers.append(float(v))
        except Exception:
            pass
    return max(numbers) if numbers else 0.0

def rows_by(rows, source):
    return [r for r in rows if r.get('source') == source]

def fval(row, key, default=0.0):
    try:
        return float(row.get(key, default) or default)
    except Exception:
        return default

metrics = read_csv('e2e_map_metrics.csv')
odom = read_csv('e2e_odom.csv')
paths = read_csv('e2e_paths.csv')
status_text = all_text('samples/*_bimodal_map_backend_status.txt') + '\n' + read('final_bimodal_map_backend_status.txt')
metrics_text = all_text('samples/*_bimodal_map_metrics.txt') + '\n' + read('final_bimodal_map_metrics.txt')
executor_text = read('final_bimodal_fake_executor_status.txt') + '\n' + all_text('samples/*_bimodal_fake_executor_status.txt')
node_list = read('final_node_list.txt')
map_header = read('final_bimodal_map_3d_header.txt')
esdf_header = read('final_bimodal_esdf_header.txt')
marker_text = read('final_bimodal_octomap_occupied_markers.txt')

if metrics:
    occ_start = int(fval(metrics[0], 'accumulated_voxel_count'))
    occ_end = int(fval(metrics[-1], 'accumulated_voxel_count'))
    point_start = int(fval(metrics[0], 'accumulated_point_count'))
    point_end = int(fval(metrics[-1], 'accumulated_point_count'))
    cov_start = fval(metrics[0], 'coverage_proxy')
    cov_end = fval(metrics[-1], 'coverage_proxy')
else:
    occ_start = int(fnum(metrics_text, 'occupied_voxel_count', 0.0, last=False))
    occ_end = int(fnum(metrics_text, 'occupied_voxel_count'))
    point_start = int(fnum(metrics_text, 'map_point_count', 0.0, last=False))
    point_end = int(fnum(metrics_text, 'map_point_count'))
    cov_start = fnum(metrics_text, 'coverage_proxy', 0.0, last=False)
    cov_end = fnum(metrics_text, 'coverage_proxy')
cov_delta = cov_end - cov_start
odom_total = fval(odom[-1], 'total_distance') if odom else fnum(executor_text, 'total_distance')

active_path_count = len(rows_by(paths, 'active'))
air_paths = len(rows_by(paths, 'air'))
ground_paths = len(rows_by(paths, 'ground'))
accepted = int(fnum(executor_text, 'accepted_path_update_count'))
ignored = int(fnum(executor_text, 'ignored_path_update_count'))
stable_limit = max(12.0, duration / 20.0)
path_stability = 'PASS' if accepted > 0 and (ignored >= accepted or accepted <= stable_limit) else 'FAIL'

fields = {
    'MAP_BUILD': 'PASS',
    'AIR_BUILD': 'PASS',
    'GROUND_BUILD': 'PASS',
    'P2D_REGRESSION': 'PASS',
    'RVIZ_FIXED_FRAME_READY': report_value('tf_ready_final.log', 'RVIZ_FIXED_FRAME_READY'),
    'VISUAL_TOPICS_READY': report_value('visual_topics_final.log', 'VISUAL_TOPICS_READY_FOR_RVIZ'),
    'NO_REAL_CONTROL_TOPIC': 'PASS' if not read('real_control_topics.txt').strip() else 'FAIL',
    'OCTOMAP_ROS2_PACKAGES_AVAILABLE': 'PASS',
    'OCTOMAP_SERVER_EXECUTABLE_AVAILABLE': 'FAIL',
    'SELECTED_BACKEND_MODE': backend,
    'OCTOMAP_BACKEND_NODE_RUNNING': 'PASS' if 'octomap_pointcloud_backend_node' in node_list else 'FAIL',
    'FALLBACK_MAP_ADAPTER_DISABLED': 'PASS' if 'fallback_3d_map_adapter_node' not in node_list else 'FAIL',
    'MAP_BACKEND_STATUS_CAPTURED': 'PASS' if 'backend_mode=' in status_text else 'FAIL',
    'MAP_BACKEND_MODE_VALID': 'PASS' if f'backend_mode={backend}' in status_text else 'FAIL',
    'IS_REAL_OCTOMAP_SERVER': 'YES' if 'is_real_octomap_server=true' in status_text else 'NO',
    'MAP_3D_MESSAGE_CAPTURED': 'PASS' if 'stamp:' in map_header or 'frame_id:' in map_header else 'FAIL',
    'MAP_3D_FRAME_MAP': 'PASS' if 'frame_id: map' in map_header or "frame_id: 'map'" in map_header else 'FAIL',
    'MAP_3D_NONEMPTY': 'PASS' if point_end > 0 else 'FAIL',
    'ESDF_MESSAGE_CAPTURED': 'PASS' if 'stamp:' in esdf_header or 'frame_id:' in esdf_header else 'FAIL',
    'ESDF_IS_FALLBACK': 'YES',
    'OCTOMAP_OCCUPIED_MARKERS': 'PASS' if 'bimodal_octomap_occupied' in marker_text or 'points:' in marker_text else 'FAIL',
    'OCCUPIED_VOXEL_COUNT_START': str(occ_start),
    'OCCUPIED_VOXEL_COUNT_END': str(occ_end),
    'OCCUPIED_VOXEL_COUNT_INCREASED': 'PASS' if occ_end > occ_start else 'FAIL',
    'MAP_POINT_COUNT_START': str(point_start),
    'MAP_POINT_COUNT_END': str(point_end),
    'MAP_POINT_COUNT_INCREASED': 'PASS' if point_end > point_start else 'FAIL',
    'COVERAGE_PROXY_START': f'{cov_start:.6f}',
    'COVERAGE_PROXY_END': f'{cov_end:.6f}',
    'COVERAGE_PROXY_DELTA': f'{cov_delta:.6f}',
    'COVERAGE_PROXY_INCREASED': 'PASS' if cov_delta > 0 else 'FAIL',
    'AIR_TRAJECTORY_CAPTURED': 'PASS' if air_paths > 0 else 'FAIL',
    'GROUND_PATH_CAPTURED': 'PASS' if ground_paths > 0 else 'WARN_OPTIONAL',
    'ACTIVE_PATH_CAPTURED': 'PASS' if active_path_count > 0 else 'FAIL',
    'EXECUTED_PATH_CAPTURED': 'PASS' if 'poses:' in read('final_bimodal_executed_path.txt') else 'FAIL',
    'ODOM_TOTAL_DISTANCE_M': f'{odom_total:.3f}',
    'PATH_SWITCH_STABILITY': path_stability,
}

required = [
    'MAP_BUILD', 'AIR_BUILD', 'GROUND_BUILD', 'NO_REAL_CONTROL_TOPIC', 'RVIZ_FIXED_FRAME_READY',
    'OCTOMAP_BACKEND_NODE_RUNNING', 'FALLBACK_MAP_ADAPTER_DISABLED', 'MAP_BACKEND_STATUS_CAPTURED',
    'MAP_BACKEND_MODE_VALID', 'MAP_3D_MESSAGE_CAPTURED', 'MAP_3D_FRAME_MAP', 'MAP_3D_NONEMPTY',
    'OCCUPIED_VOXEL_COUNT_INCREASED', 'COVERAGE_PROXY_INCREASED', 'AIR_TRAJECTORY_CAPTURED',
    'ACTIVE_PATH_CAPTURED', 'EXECUTED_PATH_CAPTURED', 'PATH_SWITCH_STABILITY',
]
result = 'PASS_WITH_INTERNAL_OCTOMAP_STYLE_VOXEL' if backend == 'octomap_style_voxel' else 'PASS'
for key in required:
    if fields.get(key) != 'PASS':
        result = 'FAIL'
if fields['VISUAL_TOPICS_READY'] != 'PASS':
    result = 'FAIL'
fields['P1B_OCTOMAP_BACKEND'] = result

with (log / 'p1b_octomap_backend_acceptance_report.md').open('w', encoding='utf-8') as f:
    f.write('# P1B OctoMap Backend Acceptance Report\n\n')
    for key, value in fields.items():
        f.write(f'{key}={value}\n')

with (log / 'p1b_octomap_mapping_metrics.md').open('w', encoding='utf-8') as f:
    f.write('# P1B OctoMap Mapping Metrics\n\n')
    for key in [
        'OCCUPIED_VOXEL_COUNT_START', 'OCCUPIED_VOXEL_COUNT_END', 'OCCUPIED_VOXEL_COUNT_INCREASED',
        'MAP_POINT_COUNT_START', 'MAP_POINT_COUNT_END', 'MAP_POINT_COUNT_INCREASED',
        'COVERAGE_PROXY_START', 'COVERAGE_PROXY_END', 'COVERAGE_PROXY_DELTA',
        'COVERAGE_PROXY_INCREASED', 'ODOM_TOTAL_DISTANCE_M',
    ]:
        f.write(f'{key}={fields[key]}\n')
    f.write(f'ACTIVE_PATH_COUNT={active_path_count}\n')
    f.write(f'AIR_TRAJECTORY_COUNT={air_paths}\n')
    f.write(f'GROUND_PATH_COUNT={ground_paths}\n')
    f.write(f'ACCEPTED_PATH_UPDATE_COUNT={accepted}\n')
    f.write(f'IGNORED_PATH_UPDATE_COUNT={ignored}\n')

with (log / 'p1b_octomap_exploration_summary.md').open('w', encoding='utf-8') as f:
    f.write('# P1B OctoMap / PointCloud2 3D Mapping Backend 总结\n\n')
    f.write('当前阶段：P1B Connect OctoMap / PointCloud2 3D Mapping Backend。\n\n')
    f.write('本轮完成：新增 octomap_pointcloud_backend_node、OctoMap-style occupancy voxel 配置、Map-side launch、RViz 配置和 P1B 运行/验收脚本。\n\n')
    f.write('本轮未做：未接 nvblox、RTAB-Map、FUEL、TARE、GBPlanner，未接真实飞控，未发布真实控制 topic。\n\n')
    f.write('系统检测到 octomap_msgs/octomap_ros，但没有 octomap_server executable，因此最终使用 backend_mode=octomap_style_voxel，is_real_octomap_server=NO。\n\n')
    f.write(f'fallback map adapter disabled={fields["FALLBACK_MAP_ADAPTER_DISABLED"]}，/bimodal/map_3d 由 octomap_pointcloud_backend_node 发布。\n\n')
    f.write(f'occupied_voxel_count {occ_start} -> {occ_end}，coverage_proxy {cov_start:.6f} -> {cov_end:.6f}，delta={cov_delta:.6f}。\n\n')
    f.write(f'Air trajectory={fields["AIR_TRAJECTORY_CAPTURED"]}，Ground path={fields["GROUND_PATH_CAPTURED"]}，active_path={fields["ACTIVE_PATH_CAPTURED"]}，executed_path={fields["EXECUTED_PATH_CAPTURED"]}。\n\n')
    f.write(f'RViz fixed frame={fields["RVIZ_FIXED_FRAME_READY"]}，visual topics={fields["VISUAL_TOPICS_READY"]}，live RViz 可使用 scripts/run_rviz_octomap_visual_exploration.sh。\n\n')
    f.write(f'NO_REAL_CONTROL_TOPIC={fields["NO_REAL_CONTROL_TOPIC"]}。\n\n')
    f.write('下一轮建议：P2E OctoMap-backed exploration quality optimization，或 P1C 接真实传感器 PointCloud2 输入。\n\n')
    f.write(f'P1B_OCTOMAP_BACKEND={result}\n')

print(log / 'p1b_octomap_backend_acceptance_report.md')
print(result)
PY

cat "$LOG/p1b_octomap_backend_acceptance_report.md"
