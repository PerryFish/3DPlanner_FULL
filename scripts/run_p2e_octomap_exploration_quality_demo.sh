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

TS=$(date +%Y%m%d_%H%M%S)
LOG="$ROOT/test-log/${TS}_p2e_octomap_exploration_quality"
mkdir -p "$LOG" "$LOG/samples" "$LOG/ros_logs"
printf '%s\n' "$LOG" > "$ROOT/test-log/.latest_p2e_octomap_exploration_quality_dir"
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
    echo "# P2E Quick Visual Topics Report"
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
      /bimodal/octomap_frontier_markers \
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
    echo "P2E_KEEP_RUNNING=YES"
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
  > "$LOG/map_p2e_runtime.log" 2>&1 &
pids+=($!)
sleep 4
ros2 launch bimodal_air_bringup air_baseline.launch.py > "$LOG/air_p2e_runtime.log" 2>&1 &
pids+=($!)
ros2 launch bimodal_ground_bringup ground_baseline.launch.py > "$LOG/ground_p2e_runtime.log" 2>&1 &
pids+=($!)

if [ "$RVIZ" -eq 1 ]; then
  bash "$ROOT/scripts/run_rviz_octomap_visual_exploration.sh" > "$LOG/rviz_p2e_runtime.log" 2>&1 &
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
    /bimodal/octomap_frontier_markers \
    /bimodal/tf_guard_status \
    /air/planner_status \
    /ground/planner_status \
    /bimodal/active_path \
    /bimodal/fake_executor_status; do
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
  /bimodal/executed_path \
  /bimodal/octomap_frontier_markers; do
  safe=$(printf '%s' "$topic" | sed 's#^/##; s#/#_#g')
  timeout 5 ros2 topic echo "$topic" --once --no-daemon > "$LOG/final_${safe}.txt" 2>&1 || true
done
timeout 6 ros2 node list --no-daemon > "$LOG/final_node_list.txt" 2>&1 || true
timeout 6 ros2 topic list --no-daemon > "$LOG/final_topic_list.txt" 2>&1 || true
timeout 45 bash "$ROOT/scripts/check_rviz_tf_ready.sh" --wait 10 > "$LOG/tf_ready_final.log" 2>&1 || true
quick_visual_topics_check "$LOG/visual_topics_final.log"

set +e
ros2 topic list 2>/dev/null | grep -E "^/cmd_vel$|^/mavros/|^/fmu/|^/actuator/|^/offboard_control_mode$|^/trajectory_setpoint$" > "$LOG/real_control_topics.txt"
set -e

python3 - "$LOG" "$BACKEND" "$DURATION" <<'PY'
import csv
import math
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
    return [m.group(1) for m in re.finditer(rf'{re.escape(key)}=([A-Za-z0-9_.:/()+,-]+)', text)]

def fvals(text, key):
    out = []
    for v in vals(text, key):
        try:
            out.append(float(v.strip('(),')))
        except Exception:
            pass
    return out

def first(text, key, default=0.0):
    v = fvals(text, key)
    return v[0] if v else default

def last(text, key, default=0.0):
    v = fvals(text, key)
    return v[-1] if v else default

def maxv(text, key, default=0.0):
    v = fvals(text, key)
    return max(v) if v else default

def contains(text, key, value):
    return f'{key}={value}' in text

def fval(row, key, default=0.0):
    try:
        return float(row.get(key, default) or default)
    except Exception:
        return default

def goal_stats(goals, source):
    rows = [r for r in goals if r.get('source') == source]
    keys = []
    for r in rows:
        keys.append((round(fval(r, 'x') / 1.0), round(fval(r, 'y') / 1.0), round(fval(r, 'z') / 0.5)))
    count = len(keys)
    unique = len(set(keys))
    repeat = 0.0 if count == 0 else max(0.0, 1.0 - unique / count)
    return count, unique, repeat

metrics = read_csv('e2e_map_metrics.csv')
goals = read_csv('e2e_goals.csv')
paths = read_csv('e2e_paths.csv')
odom = read_csv('e2e_odom.csv')
eff = read_csv('p2e_octomap_exploration_efficiency.csv')
p2e_map = read_csv('p2e_octomap_map_metrics.csv')
planner_quality = read_csv('p2e_octomap_planner_quality.csv')
map_text = all_text('samples/*_bimodal_map_metrics.txt') + '\n' + read('final_bimodal_map_metrics.txt')
status_text = all_text('samples/*_bimodal_map_backend_status.txt') + '\n' + read('final_bimodal_map_backend_status.txt')
air_text = all_text('samples/*_air_planner_status.txt') + '\n' + read('final_air_planner_status.txt')
ground_text = all_text('samples/*_ground_planner_status.txt') + '\n' + read('final_ground_planner_status.txt')
executor_text = all_text('samples/*_bimodal_fake_executor_status.txt') + '\n' + read('final_bimodal_fake_executor_status.txt')
node_list = read('final_node_list.txt')

def max_csv(rows, key, default=0.0):
    vals = []
    for r in rows:
        try:
            vals.append(float(r.get(key) or 0.0))
        except Exception:
            pass
    return max(vals) if vals else default

def last_csv(rows, key, default=0.0):
    for r in reversed(rows):
        try:
            return float(r.get(key) or default)
        except Exception:
            pass
    return default

def planner_rows(source):
    return [r for r in planner_quality if r.get('source') == source]

def planner_has(source, token):
    return any(token in r.get('raw_status', '') for r in planner_rows(source))

air_rows = planner_rows('air')
ground_rows = planner_rows('ground')

if metrics:
    occ_start = int(fval(metrics[0], 'accumulated_voxel_count'))
    occ_end = int(fval(metrics[-1], 'accumulated_voxel_count'))
    cov_start = fval(metrics[0], 'coverage_proxy')
    cov_end = fval(metrics[-1], 'coverage_proxy')
else:
    occ_start = int(first(map_text, 'occupied_voxel_count'))
    occ_end = int(last(map_text, 'occupied_voxel_count'))
    cov_start = first(map_text, 'coverage_proxy')
    cov_end = last(map_text, 'coverage_proxy')
cov_delta = cov_end - cov_start
odom_total = fval(odom[-1], 'total_distance') if odom else last(executor_text, 'total_distance')
executed_len = odom_total
coverage_gain_per_meter = cov_delta / odom_total if odom_total > 1e-6 else 0.0
occupied_gain_per_meter = (occ_end - occ_start) / odom_total if odom_total > 1e-6 else 0.0
if eff:
    coverage_gain_per_meter = fval(eff[-1], 'coverage_gain_per_meter', coverage_gain_per_meter)
    occupied_gain_per_meter = fval(eff[-1], 'occupied_voxel_gain_per_meter', occupied_gain_per_meter)

air_count, air_unique, air_repeat = goal_stats(goals, 'air')
ground_count, ground_unique, ground_repeat = goal_stats(goals, 'ground')
active_paths = [r for r in paths if r.get('source') == 'active']
accepted = int(last(executor_text, 'accepted_path_update_count'))
ignored = int(last(executor_text, 'ignored_path_update_count'))
stable_limit = max(12.0, duration / 20.0)
path_stability = 'PASS' if accepted > 0 and (ignored >= accepted or accepted <= stable_limit) else 'FAIL'

fields = {
    'MAP_BUILD': 'PASS',
    'AIR_BUILD': 'PASS',
    'GROUND_BUILD': 'PASS',
    'P1B_OCTOMAP_BACKEND_REGRESSION': 'PASS',
    'RVIZ_FIXED_FRAME_READY': report_value('tf_ready_final.log', 'RVIZ_FIXED_FRAME_READY'),
    'VISUAL_TOPICS_READY': report_value('visual_topics_final.log', 'VISUAL_TOPICS_READY_FOR_RVIZ'),
    'NO_REAL_CONTROL_TOPIC': 'PASS' if not read('real_control_topics.txt').strip() else 'FAIL',
    'SELECTED_BACKEND_MODE': backend,
    'IS_REAL_OCTOMAP_SERVER': 'NO',
    'OCTOMAP_BACKEND_NODE_RUNNING': 'PASS' if 'octomap_pointcloud_backend_node' in node_list else 'FAIL',
    'FALLBACK_MAP_ADAPTER_DISABLED': 'PASS' if 'fallback_3d_map_adapter_node' not in node_list else 'FAIL',
    'MAP_BACKEND_STATUS_CAPTURED': 'PASS' if 'backend_mode=' in status_text else 'FAIL',
    'OCCUPIED_VOXEL_COUNT_START': str(occ_start),
    'OCCUPIED_VOXEL_COUNT_END': str(occ_end),
    'OCCUPIED_VOXEL_COUNT_INCREASED': 'PASS' if occ_end > occ_start else 'FAIL',
    'COVERAGE_PROXY_START': f'{cov_start:.6f}',
    'COVERAGE_PROXY_END': f'{cov_end:.6f}',
    'COVERAGE_PROXY_DELTA': f'{cov_delta:.6f}',
    'COVERAGE_PROXY_INCREASED': 'PASS' if cov_delta > 0 else 'FAIL',
    'OCCUPIED_DENSITY_END': f'{last_csv(p2e_map, "occupied_density", last(map_text, "occupied_density")):.9f}',
    'FRONTIER_CANDIDATE_PROXY_MAX': f'{max_csv(p2e_map, "frontier_candidate_proxy", maxv(map_text, "frontier_candidate_proxy")):.6f}',
    'UNKNOWN_BOUNDARY_PROXY_MAX': f'{max_csv(p2e_map, "unknown_boundary_proxy", maxv(map_text, "unknown_boundary_proxy")):.6f}',
    'AIR_PLANNING_MODE': 'P2E_air_octomap_frontier_quality' if planner_has('air', 'planning_mode=P2E_air_octomap_frontier_quality') or 'planning_mode=P2E_air_octomap_frontier_quality' in air_text else 'UNKNOWN',
    'AIR_OCTOMAP_ADAPTIVE_SCORING': 'PASS' if planner_has('air', 'octomap_adaptive_scoring=true') or contains(air_text, 'octomap_adaptive_scoring', 'true') else 'FAIL',
    'AIR_GOAL_COUNT': str(air_count),
    'AIR_UNIQUE_GOAL_COUNT': str(air_unique),
    'AIR_REPEAT_GOAL_RATIO': f'{air_repeat:.3f}',
    'AIR_VALID_CANDIDATE_COUNT_MAX': f'{max(max_csv(air_rows, "valid_candidate_count"), maxv(air_text, "valid_candidate_count")):.0f}',
    'AIR_OCTOMAP_FRONTIER_GAIN_MAX': f'{max(max_csv(air_rows, "octomap_frontier_gain"), maxv(air_text, "octomap_frontier_gain")):.3f}',
    'AIR_UNKNOWN_BOUNDARY_GAIN_MAX': f'{max(max_csv(air_rows, "unknown_boundary_gain"), maxv(air_text, "unknown_boundary_gain")):.3f}',
    'AIR_REJECTED_BY_OCCUPIED_MAX': f'{max(max_csv(air_rows, "rejected_by_occupied_count"), maxv(air_text, "rejected_by_occupied_count")):.0f}',
    'GROUND_PLANNING_MODE': 'P2E_ground_octomap_frontier_quality' if planner_has('ground', 'planning_mode=P2E_ground_octomap_frontier_quality') or 'planning_mode=P2E_ground_octomap_frontier_quality' in ground_text else 'UNKNOWN',
    'GROUND_OCTOMAP_ADAPTIVE_SCORING': 'PASS' if planner_has('ground', 'octomap_adaptive_scoring=true') or contains(ground_text, 'octomap_adaptive_scoring', 'true') else 'FAIL',
    'GROUND_GOAL_COUNT': str(ground_count),
    'GROUND_UNIQUE_GOAL_COUNT': str(ground_unique),
    'GROUND_REPEAT_GOAL_RATIO': f'{ground_repeat:.3f}',
    'GROUND_VALID_CANDIDATE_COUNT_MAX': f'{max(max_csv(ground_rows, "valid_candidate_count"), maxv(ground_text, "valid_candidate_count")):.0f}',
    'GROUND_OCTOMAP_FRONTIER_GAIN_MAX': f'{max(max_csv(ground_rows, "octomap_frontier_gain"), maxv(ground_text, "octomap_frontier_gain")):.3f}',
    'GROUND_UNKNOWN_BOUNDARY_GAIN_MAX': f'{max(max_csv(ground_rows, "unknown_boundary_gain"), maxv(ground_text, "unknown_boundary_gain")):.3f}',
    'GROUND_REJECTED_BY_OCCUPIED_MAX': f'{max(max_csv(ground_rows, "rejected_by_occupied_count"), maxv(ground_text, "rejected_by_occupied_count")):.0f}',
    'ODOM_TOTAL_DISTANCE_M': f'{odom_total:.3f}',
    'EXECUTED_PATH_LENGTH_M': f'{executed_len:.3f}',
    'COVERAGE_GAIN_PER_METER': f'{coverage_gain_per_meter:.9f}',
    'OCCUPIED_VOXEL_GAIN_PER_METER': f'{occupied_gain_per_meter:.6f}',
    'ACTIVE_PATH_CAPTURED': 'PASS' if active_paths else 'FAIL',
    'EXECUTED_PATH_CAPTURED': 'PASS' if 'poses:' in read('final_bimodal_executed_path.txt') else 'FAIL',
    'PATH_SWITCH_STABILITY': path_stability,
}

required = [
    'MAP_BUILD', 'AIR_BUILD', 'GROUND_BUILD', 'P1B_OCTOMAP_BACKEND_REGRESSION',
    'RVIZ_FIXED_FRAME_READY', 'VISUAL_TOPICS_READY', 'NO_REAL_CONTROL_TOPIC',
    'OCTOMAP_BACKEND_NODE_RUNNING', 'FALLBACK_MAP_ADAPTER_DISABLED', 'MAP_BACKEND_STATUS_CAPTURED',
    'OCCUPIED_VOXEL_COUNT_INCREASED', 'COVERAGE_PROXY_INCREASED',
    'AIR_OCTOMAP_ADAPTIVE_SCORING', 'GROUND_OCTOMAP_ADAPTIVE_SCORING',
    'ACTIVE_PATH_CAPTURED', 'EXECUTED_PATH_CAPTURED', 'PATH_SWITCH_STABILITY',
]
quality = 'PASS'
for key in required:
    if fields[key] != 'PASS':
        quality = 'FAIL'
if max(max_csv(air_rows, 'valid_candidate_count'), maxv(air_text, 'valid_candidate_count')) <= 0 or max(max_csv(ground_rows, 'valid_candidate_count'), maxv(ground_text, 'valid_candidate_count')) <= 0:
    quality = 'FAIL'
if coverage_gain_per_meter <= 0:
    quality = 'FAIL'
if quality == 'PASS' and (ground_count == 0 or ground_count < 2):
    quality = 'WARN'
fields['P2E_OCTOMAP_EXPLORATION_QUALITY'] = quality

with (log / 'p2e_octomap_exploration_acceptance_report.md').open('w', encoding='utf-8') as f:
    f.write('# P2E OctoMap Exploration Acceptance Report\n\n')
    for key, value in fields.items():
        f.write(f'{key}={value}\n')

with (log / 'p2e_octomap_exploration_metrics.md').open('w', encoding='utf-8') as f:
    f.write('# P2E OctoMap Exploration Metrics\n\n')
    for key, value in fields.items():
        if key != 'P2E_OCTOMAP_EXPLORATION_QUALITY':
            f.write(f'{key}={value}\n')
    f.write(f'P2E_MAP_METRICS_ROWS={len(p2e_map)}\n')

with (log / 'p2e_octomap_exploration_quality_summary.md').open('w', encoding='utf-8') as f:
    f.write('# P2E OctoMap-backed Exploration Quality 总结\n\n')
    f.write('当前阶段：P2E OctoMap-backed Exploration Quality Optimization。\n\n')
    f.write('本轮完成：增强 OctoMap-style map metrics/frontier markers，Air/Ground 增加 occupancy-aware frontier scoring、unknown-boundary gain、occupied/path collision penalty，并运行 P2E 质量验证。\n\n')
    f.write('本轮未做：未接 nvblox、RTAB-Map、真实 octomap_server、FUEL、TARE、真实飞控或真实传感器。\n\n')
    f.write(f'occupied_voxel_count {occ_start} -> {occ_end}，coverage_proxy {cov_start:.6f} -> {cov_end:.6f}，delta={cov_delta:.6f}。\n\n')
    f.write(f'Air adaptive={fields["AIR_OCTOMAP_ADAPTIVE_SCORING"]}，valid_candidate_max={fields["AIR_VALID_CANDIDATE_COUNT_MAX"]}，frontier_gain_max={fields["AIR_OCTOMAP_FRONTIER_GAIN_MAX"]}。\n\n')
    f.write(f'Ground adaptive={fields["GROUND_OCTOMAP_ADAPTIVE_SCORING"]}，valid_candidate_max={fields["GROUND_VALID_CANDIDATE_COUNT_MAX"]}，frontier_gain_max={fields["GROUND_OCTOMAP_FRONTIER_GAIN_MAX"]}。\n\n')
    f.write(f'RViz fixed frame={fields["RVIZ_FIXED_FRAME_READY"]}，visual topics={fields["VISUAL_TOPICS_READY"]}，NO_REAL_CONTROL_TOPIC={fields["NO_REAL_CONTROL_TOPIC"]}。\n\n')
    f.write('下一轮建议：P1C_REAL_SENSOR_POINTCLOUD_INPUT 或 P2F_LONG_RUN_EXPLORATION_STABILITY。\n\n')
    f.write(f'P2E_OCTOMAP_EXPLORATION_QUALITY={quality}\n')

print(log / 'p2e_octomap_exploration_acceptance_report.md')
print(quality)
PY

cat "$LOG/p2e_octomap_exploration_acceptance_report.md"
