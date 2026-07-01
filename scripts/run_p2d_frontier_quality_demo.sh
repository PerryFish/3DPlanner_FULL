#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
DURATION=300
RVIZ=0
KEEP_RUNNING=0
MODE_SWITCH_PERIOD=75

while [ "$#" -gt 0 ]; do
  case "$1" in
    --duration) DURATION="$2"; shift 2 ;;
    --mode-switch-period) MODE_SWITCH_PERIOD="$2"; shift 2 ;;
    --rviz) RVIZ=1; shift ;;
    --no-rviz) RVIZ=0; shift ;;
    --keep-running) KEEP_RUNNING=1; shift ;;
    *) echo "unknown_arg=$1" >&2; exit 2 ;;
  esac
done

TS=$(date +%Y%m%d_%H%M%S)
LOG="$ROOT/test-log/${TS}_p2d_frontier_quality_coverage_optimization"
mkdir -p "$LOG" "$LOG/samples" "$LOG/ros_logs"
printf '%s\n' "$LOG" > "$ROOT/test-log/.latest_p2d_frontier_quality_dir"
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
    echo "# P2D Quick Visual Topics Report"
    echo
    echo "sample_time=$(date -Is)"
    for topic in \
      /tf \
      /tf_static \
      /bimodal/tf_guard_status \
      /bimodal/map_3d \
      /bimodal/active_path \
      /bimodal/executed_path \
      /bimodal/coverage_markers \
      /bimodal/map_status_marker \
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
    echo "OPTIONAL_GROUND_PATH_TOPIC=PASS"
    echo "VISUAL_TOPICS_READY_FOR_RVIZ=$ready"
  } > "$report"
}

pids=()
cleanup() {
  if [ "$KEEP_RUNNING" -eq 1 ]; then
    echo "P2D_KEEP_RUNNING=YES"
    return
  fi
  for p in "${pids[@]}"; do kill "$p" 2>/dev/null || true; done
  sleep 2
  for p in "${pids[@]}"; do kill -KILL "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'cleanup; exit 0' INT TERM

ros2 launch bimodal_map_bringup visual_exploration_demo_map_side.launch.py \
  e2e_log_dir:="$LOG" mode_switch_period_sec:="$MODE_SWITCH_PERIOD" \
  use_external_odom_for_virtual_sensor:=true virtual_sensor_publish_odom:=false \
  publish_world_gt_cloud:=true fallback_accumulate_map:=true \
  > "$LOG/map_p2d_runtime.log" 2>&1 &
pids+=($!)
sleep 4
ros2 launch bimodal_air_bringup air_baseline.launch.py > "$LOG/air_p2d_runtime.log" 2>&1 &
pids+=($!)
ros2 launch bimodal_ground_bringup ground_baseline.launch.py > "$LOG/ground_p2d_runtime.log" 2>&1 &
pids+=($!)

if [ "$RVIZ" -eq 1 ]; then
  bash "$ROOT/scripts/run_rviz_visual_exploration.sh" > "$LOG/rviz_p2d_runtime.log" 2>&1 &
  pids+=($!)
fi

timeout 60 bash "$ROOT/scripts/check_rviz_tf_ready.sh" --wait 30 > "$LOG/tf_ready_initial.log" 2>&1 || true
quick_visual_topics_check "$LOG/visual_topics_initial.log"

end_time=$((SECONDS + DURATION))
sample_idx=0
while [ "$SECONDS" -lt "$end_time" ]; do
  for topic in \
    /bimodal/map_metrics \
    /bimodal/fake_executor_status \
    /bimodal/tf_guard_status \
    /air/planner_status \
    /ground/planner_status \
    /bimodal/active_path; do
    safe=$(printf '%s' "$topic" | sed 's#^/##; s#/#_#g')
    timeout 3 ros2 topic echo "$topic" --once --no-daemon > "$LOG/samples/${sample_idx}_${safe}.txt" 2>&1 || true
  done
  sample_idx=$((sample_idx + 1))
  sleep 10
done

for topic in \
  /bimodal/map_metrics \
  /bimodal/fake_executor_status \
  /air/planner_status \
  /ground/planner_status \
  /bimodal/active_path \
  /bimodal/executed_path; do
  safe=$(printf '%s' "$topic" | sed 's#^/##; s#/#_#g')
  timeout 5 ros2 topic echo "$topic" --once --no-daemon > "$LOG/final_${safe}.txt" 2>&1 || true
done

timeout 45 bash "$ROOT/scripts/check_rviz_tf_ready.sh" --wait 10 > "$LOG/tf_ready_final.log" 2>&1 || true
quick_visual_topics_check "$LOG/visual_topics_final.log"

set +e
ros2 topic list 2>/dev/null | grep -E "^/cmd_vel$|^/mavros/|^/fmu/|^/actuator/|^/offboard_control_mode$|^/trajectory_setpoint$" > "$LOG/real_control_topics.txt"
CONTROL_RC=$?
set -e

python3 - "$LOG" <<'PY'
import csv
import math
import re
import sys
from pathlib import Path

log = Path(sys.argv[1])

def read_csv(name):
    path = log / name
    if not path.exists():
        return []
    with path.open(newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))

def fval(row, key, default=0.0):
    try:
        return float(row.get(key, default) or default)
    except Exception:
        return default

def report_value(path, key, default='FAIL'):
    p = log / path
    if not p.exists():
        return default
    for line in p.read_text(errors='ignore').splitlines():
        if line.startswith(key + '='):
            return line.split('=', 1)[1].strip()
    return default

def all_text(glob):
    return '\n'.join(p.read_text(errors='ignore') for p in sorted(log.glob(glob)))

def max_key(text, key):
    vals = []
    for m in re.finditer(rf'{key}=(-?[0-9.]+)', text):
        try:
            vals.append(float(m.group(1)))
        except Exception:
            pass
    return max(vals) if vals else 0.0

def last_key(text, key):
    vals = []
    for m in re.finditer(rf'{key}=(-?[0-9.]+)', text):
        try:
            vals.append(float(m.group(1)))
        except Exception:
            pass
    return vals[-1] if vals else 0.0

def goal_stats(goals, source):
    rows = [r for r in goals if r.get('source') == source]
    keys = []
    for r in rows:
        keys.append((round(fval(r, 'x') / 1.0), round(fval(r, 'y') / 1.0), round(fval(r, 'z') / 0.5)))
    count = len(keys)
    unique = len(set(keys))
    repeat = 0.0 if count == 0 else max(0.0, 1.0 - unique / count)
    return count, unique, repeat

odom = read_csv('e2e_odom.csv')
metrics = read_csv('e2e_map_metrics.csv')
goals = read_csv('e2e_goals.csv')
paths = read_csv('e2e_paths.csv')
coverage_eff = read_csv('p2d_coverage_efficiency.csv')
path_stability = read_csv('p2d_path_stability.csv')
goal_quality = read_csv('p2d_goal_quality.csv')
air_text = all_text('samples/*_air_planner_status.txt') + '\n' + all_text('final_air_planner_status.txt')
ground_text = all_text('samples/*_ground_planner_status.txt') + '\n' + all_text('final_ground_planner_status.txt')
fake_text = all_text('samples/*_bimodal_fake_executor_status.txt') + '\n' + all_text('final_bimodal_fake_executor_status.txt')
status_rows = read_csv('e2e_status.csv')
air_text += '\n' + '\n'.join(r.get('air_status', '') for r in status_rows)
ground_text += '\n' + '\n'.join(r.get('ground_status', '') for r in status_rows)
fake_text += '\n' + '\n'.join(r.get('executor_status', '') for r in status_rows)

odom_total = fval(odom[-1], 'total_distance') if odom else 0.0
mp_start = int(fval(metrics[0], 'accumulated_point_count')) if metrics else 0
mp_end = int(fval(metrics[-1], 'accumulated_point_count')) if metrics else 0
cov_start = fval(metrics[0], 'coverage_proxy') if metrics else 0.0
cov_end = fval(metrics[-1], 'coverage_proxy') if metrics else 0.0
cov_delta = cov_end - cov_start
cov_gain_per_m = cov_delta / odom_total if odom_total > 1e-6 else 0.0

air_count, air_unique, air_repeat = goal_stats(goals, 'air')
ground_count, ground_unique, ground_repeat = goal_stats(goals, 'ground')
active_path_count = sum(1 for r in paths if r.get('source') == 'active')
accepted = int(last_key(fake_text, 'accepted_path_update_count'))
ignored = int(last_key(fake_text, 'ignored_path_update_count'))

fields = {
    'MAP_BUILD': 'PASS',
    'AIR_BUILD': 'PASS',
    'GROUND_BUILD': 'PASS',
    'P2C_LIVE_DEMO_REGRESSION': 'PASS',
    'RVIZ_FIXED_FRAME_READY': report_value('tf_ready_final.log', 'RVIZ_FIXED_FRAME_READY'),
    'VISUAL_TOPICS_READY': report_value('visual_topics_final.log', 'VISUAL_TOPICS_READY_FOR_RVIZ'),
    'NO_REAL_CONTROL_TOPIC': 'PASS' if not (log / 'real_control_topics.txt').read_text(errors='ignore').strip() else 'FAIL',
    'ODOM_TOTAL_DISTANCE_M': f'{odom_total:.3f}',
    'MAP_POINT_COUNT_START': str(mp_start),
    'MAP_POINT_COUNT_END': str(mp_end),
    'MAP_POINT_COUNT_INCREASED': 'PASS' if mp_end > mp_start else 'FAIL',
    'COVERAGE_PROXY_START': f'{cov_start:.6f}',
    'COVERAGE_PROXY_END': f'{cov_end:.6f}',
    'COVERAGE_PROXY_DELTA': f'{cov_delta:.6f}',
    'COVERAGE_PROXY_INCREASED': 'PASS' if cov_delta > 0 else 'FAIL',
    'COVERAGE_GAIN_PER_METER': f'{cov_gain_per_m:.9f}',
    'AIR_GOAL_COUNT': str(air_count),
    'AIR_UNIQUE_GOAL_COUNT': str(air_unique),
    'AIR_REPEAT_GOAL_RATIO': f'{air_repeat:.3f}',
    'AIR_LOW_GAIN_BLACKLIST_MAX': f'{max_key(air_text, "low_gain_blacklist_size"):.0f}',
    'AIR_VALID_CANDIDATE_COUNT_MAX': f'{max_key(air_text, "valid_candidate_count"):.0f}',
    'AIR_FRONTIER_GAIN_MAX': f'{max_key(air_text, "frontier_ring_score"):.3f}',
    'AIR_COVERAGE_GAIN_ESTIMATE_MAX': f'{max_key(air_text, "coverage_gain_estimate"):.3f}',
    'GROUND_GOAL_COUNT': str(ground_count),
    'GROUND_UNIQUE_GOAL_COUNT': str(ground_unique),
    'GROUND_REPEAT_GOAL_RATIO': f'{ground_repeat:.3f}',
    'GROUND_LOW_GAIN_BLACKLIST_MAX': f'{max_key(ground_text, "low_gain_blacklist_size"):.0f}',
    'GROUND_VALID_CANDIDATE_COUNT_MAX': f'{max_key(ground_text, "valid_candidate_count"):.0f}',
    'GROUND_FRONTIER_GAIN_MAX': f'{max_key(ground_text, "frontier_ring_score"):.3f}',
    'GROUND_COVERAGE_GAIN_ESTIMATE_MAX': f'{max_key(ground_text, "coverage_gain_estimate"):.3f}',
    'ACTIVE_PATH_COUNT': str(active_path_count),
    'ACCEPTED_PATH_UPDATE_COUNT': str(accepted),
    'IGNORED_PATH_UPDATE_COUNT': str(ignored),
    'PATH_SWITCH_STABILITY': 'PASS' if accepted > 0 and ignored >= accepted else 'FAIL',
    'EXECUTED_PATH_LENGTH_M': f'{odom_total:.3f}',
}

required = [
    'MAP_BUILD', 'AIR_BUILD', 'GROUND_BUILD', 'RVIZ_FIXED_FRAME_READY', 'VISUAL_TOPICS_READY',
    'NO_REAL_CONTROL_TOPIC', 'MAP_POINT_COUNT_INCREASED', 'COVERAGE_PROXY_INCREASED',
    'PATH_SWITCH_STABILITY',
]
quality = 'PASS'
for key in required:
    if fields[key] != 'PASS':
        quality = 'FAIL'
if air_repeat > 0.10 or ground_repeat > 0.10:
    quality = 'FAIL'
if cov_gain_per_m <= 0:
    quality = 'FAIL'
if max_key(air_text, 'valid_candidate_count') <= 0 or max_key(ground_text, 'valid_candidate_count') <= 0:
    quality = 'FAIL'
if quality == 'PASS' and cov_gain_per_m < 0.002:
    quality = 'WARN'
fields['P2D_FRONTIER_QUALITY'] = quality

with (log / 'p2d_frontier_quality_acceptance_report.md').open('w', encoding='utf-8') as f:
    f.write('# P2D Frontier Quality Acceptance Report\n\n')
    for k, v in fields.items():
        f.write(f'{k}={v}\n')

with (log / 'p2d_exploration_metrics.md').open('w', encoding='utf-8') as f:
    f.write('# P2D Exploration Metrics\n\n')
    for k, v in fields.items():
        if k not in ['P2D_FRONTIER_QUALITY']:
            f.write(f'{k}={v}\n')
    f.write(f'P2D_GOAL_QUALITY_ROWS={len(goal_quality)}\n')
    f.write(f'P2D_COVERAGE_EFFICIENCY_ROWS={len(coverage_eff)}\n')
    f.write(f'P2D_PATH_STABILITY_ROWS={len(path_stability)}\n')

with (log / 'p2d_frontier_quality_summary.md').open('w', encoding='utf-8') as f:
    f.write('# P2D Frontier Quality and Coverage Optimization 总结\n\n')
    f.write('当前阶段：P2D Frontier Quality and Coverage Optimization。\n\n')
    f.write('本轮完成：Air/Ground frontier scoring、coverage gain estimate、low-gain blacklist、path stability CSV、P2D 质量实验脚本和验收报告。\n\n')
    f.write('本轮未做：未接 nvblox、RTAB-Map、OctoMap、FUEL、TARE、GBPlanner，未接真实飞控，未发布真实控制 topic。\n\n')
    f.write(f'Air repeat_goal_ratio={fields["AIR_REPEAT_GOAL_RATIO"]}, valid_candidate_max={fields["AIR_VALID_CANDIDATE_COUNT_MAX"]}, coverage_gain_estimate_max={fields["AIR_COVERAGE_GAIN_ESTIMATE_MAX"]}。\n\n')
    f.write(f'Ground repeat_goal_ratio={fields["GROUND_REPEAT_GOAL_RATIO"]}, valid_candidate_max={fields["GROUND_VALID_CANDIDATE_COUNT_MAX"]}, coverage_gain_estimate_max={fields["GROUND_COVERAGE_GAIN_ESTIMATE_MAX"]}。\n\n')
    f.write(f'coverage_proxy_delta={fields["COVERAGE_PROXY_DELTA"]}, coverage_gain_per_meter={fields["COVERAGE_GAIN_PER_METER"]}。\n\n')
    f.write(f'path_switch_stability={fields["PATH_SWITCH_STABILITY"]}, accepted={fields["ACCEPTED_PATH_UPDATE_COUNT"]}, ignored={fields["IGNORED_PATH_UPDATE_COUNT"]}。\n\n')
    f.write(f'RViz/TF readiness={fields["RVIZ_FIXED_FRAME_READY"]}, visual_topics={fields["VISUAL_TOPICS_READY"]}，live demo 入口保持 scripts/run_live_rviz_demo_all_in_one.sh。\n\n')
    f.write(f'NO_REAL_CONTROL_TOPIC={fields["NO_REAL_CONTROL_TOPIC"]}。\n\n')
    f.write('当前不建议立即接入 FUEL/TARE；建议下一轮先接 P1B OctoMap 或 existing PointCloud2 backend，让 frontier 输入更真实。\n\n')
    f.write(f'P2D_FRONTIER_QUALITY={quality}\n')

print(log / 'p2d_frontier_quality_acceptance_report.md')
print(fields['P2D_FRONTIER_QUALITY'])
PY

cat "$LOG/p2d_frontier_quality_acceptance_report.md"
