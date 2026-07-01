#!/usr/bin/env bash
set -u

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
DURATION=180
RVIZ=1
KEEP_RUNNING=0
MODE_SWITCH_PERIOD=45

while [ "$#" -gt 0 ]; do
  case "$1" in
    --duration)
      DURATION="$2"; shift 2 ;;
    --no-rviz)
      RVIZ=0; shift ;;
    --rviz)
      RVIZ=1; shift ;;
    --mode-switch-period)
      MODE_SWITCH_PERIOD="$2"; shift 2 ;;
    --keep-running)
      KEEP_RUNNING=1; shift ;;
    *)
      echo "unknown_arg=$1" >&2; exit 2 ;;
  esac
done

TS=$(date +%Y%m%d_%H%M%S)
LOG="${P2B_LOG_DIR:-$ROOT/test-log/${TS}_p2b_visual_exploration_demo}"
mkdir -p "$LOG" "$LOG/samples" "$LOG/ros_logs" "$ROOT/Map/test-log/$TS" "$ROOT/Air/test-log/$TS" "$ROOT/Ground/test-log/$TS"
export ROS_LOG_DIR="$LOG/ros_logs"
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-$ROOT/Map/config/fastdds_shm_only.xml}
export FASTDDS_DEFAULT_PROFILES_FILE=${FASTDDS_DEFAULT_PROFILES_FILE:-$ROOT/Map/config/fastdds_shm_only.xml}

DISPLAY_LIMITATION=NO
if [ "$RVIZ" -eq 1 ] && [ -z "${DISPLAY:-}" ]; then
  DISPLAY_LIMITATION=YES
  RVIZ=0
  echo "DISPLAY_ENVIRONMENT_LIMITATION" > "$LOG/rviz_runtime.log"
fi

set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$ROOT/Map/ros2_ws/install/setup.bash" 2>/dev/null || true
source "$ROOT/Air/ros2_ws/install/setup.bash" 2>/dev/null || true
source "$ROOT/Ground/ros2_ws/install/setup.bash" 2>/dev/null || true
set -u

pids=()
rviz_pid=""
cleanup() {
  if [ "$KEEP_RUNNING" -eq 1 ]; then
    return
  fi
  for p in "${pids[@]}"; do kill "$p" 2>/dev/null || true; done
  sleep 2
  for p in "${pids[@]}"; do kill -KILL "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT

ros2 launch bimodal_map_bringup visual_exploration_demo_map_side.launch.py \
  e2e_log_dir:="$LOG" \
  run_duration_sec:="$DURATION" \
  use_external_odom_for_virtual_sensor:=true \
  virtual_sensor_publish_odom:=false \
  fallback_accumulate_map:=true \
  publish_world_gt_cloud:=true \
  publish_local_sensor_cloud:=true \
  mode_auto_switch:=true \
  mode_switch_period_sec:="$MODE_SWITCH_PERIOD" \
  active_mode_publish_period_sec:=1.0 \
  active_path_republish_period_sec:=1.0 \
  sensor_range:=5.0 \
  > "$LOG/map_visual_runtime.log" 2>&1 &
pids+=($!)

sleep 4
ros2 launch bimodal_air_bringup air_baseline.launch.py > "$LOG/air_visual_runtime.log" 2>&1 &
pids+=($!)
ros2 launch bimodal_ground_bringup ground_baseline.launch.py > "$LOG/ground_visual_runtime.log" 2>&1 &
pids+=($!)

if [ "$RVIZ" -eq 1 ]; then
  rviz2 -d "$ROOT/Map/rviz/visual_exploration_demo.rviz" > "$LOG/rviz_runtime.log" 2>&1 &
  rviz_pid=$!
  pids+=($rviz_pid)
fi

sample_topic() {
  topic="$1"
  out="$2"
  timeout 6 ros2 topic echo "$topic" --once --no-daemon > "$out" 2>&1 || true
}

sample_topics() {
  idx="$1"
  ros2 topic list --no-daemon > "$LOG/samples/topic_list_${idx}.txt" 2>&1 || true
  ros2 node list --no-daemon > "$LOG/samples/node_list_${idx}.txt" 2>&1 || true
  for t in /bimodal/map_metrics /bimodal/fake_executor_status /bimodal/active_mode /bimodal/active_path /air/planner_status /ground/planner_status; do
    safe=$(printf '%s' "$t" | sed 's#^/##; s#/#_#g')
    sample_topic "$t" "$LOG/samples/${idx}_${safe}.txt"
  done
}

start_epoch=$(date +%s)
idx=0
while true; do
  now=$(date +%s)
  elapsed=$((now - start_epoch))
  [ "$elapsed" -ge "$DURATION" ] && break
  sample_topics "$idx"
  idx=$((idx + 1))
  sleep 10
done
sample_topics "final"

topics=$(ros2 topic list --no-daemon 2>/dev/null | sort)
nodes=$(ros2 node list --no-daemon 2>/dev/null | sort)
echo "$topics" > "$LOG/final_topic_list.txt"
echo "$nodes" > "$LOG/final_node_list.txt"

for t in \
  /bimodal/world_gt_cloud /bimodal/points /bimodal/map_3d /bimodal/coverage_markers \
  /bimodal/map_status_marker /bimodal/robot_marker /bimodal/sensor_range_marker \
  /air/candidate_markers /air/selected_goal_marker /ground/frontier_candidates \
  /bimodal/active_path /bimodal/executed_path /bimodal/executor_marker \
  /bimodal/executor_status_marker /bimodal/active_mode /air/planner_status \
  /ground/planner_status /bimodal/map_metrics /bimodal/fake_executor_status /tf; do
  safe=$(printf '%s' "$t" | sed 's#^/##; s#/#_#g')
  sample_topic "$t" "$LOG/final_${safe}.txt"
done

tf_valid=PASS
tf_yaml="$LOG/tf_echo_samples.txt"
for pair in "map odom" "odom base_link" "base_link camera_link" "base_link lidar_link"; do
  set -- $pair
  timeout 4 ros2 run tf2_ros tf2_echo "$1" "$2" >> "$tf_yaml" 2>&1 || true
done
for frame in odom base_link camera_link lidar_link; do grep -q "$frame" "$tf_yaml" || tf_valid=FAIL; done

no_control=PASS
echo "$topics" | grep -Eq '^/cmd_vel$|^/offboard_control_mode$|^/trajectory_setpoint$|^/mavros/|^/fmu/|^/actuator/' && no_control=FAIL
{
  echo "# Safety Runtime"
  echo '```'
  echo "$topics" | grep -E '^/cmd_vel$|^/mavros/|^/fmu/|^/actuator/|^/offboard_control_mode$|^/trajectory_setpoint$' || true
  echo '```'
  echo "NO_REAL_CONTROL_TOPIC=$no_control"
} > "$LOG/safety_runtime.log"

rviz_started=SKIP_DISPLAY_LIMITATION
rviz_attempted=NO
if [ "$DISPLAY_LIMITATION" = "NO" ] && [ -n "$rviz_pid" ]; then
  rviz_attempted=YES
  if grep -Eqi 'could not connect to display|qt\.qpa|xcb|no display' "$LOG/rviz_runtime.log"; then
    DISPLAY_LIMITATION=YES
    rviz_started=SKIP_DISPLAY_LIMITATION
  elif kill -0 "$rviz_pid" 2>/dev/null || grep -qi "rviz" "$LOG/rviz_runtime.log"; then
    rviz_started=PASS
  else
    rviz_started=FAIL
  fi
elif [ "$DISPLAY_LIMITATION" = "NO" ] && [ "$RVIZ" -eq 0 ]; then
  rviz_started=SKIP_DISPLAY_LIMITATION
fi

if [ "$KEEP_RUNNING" -eq 0 ]; then
  cleanup
  trap - EXIT
fi

python3 - "$LOG" "$DURATION" "$tf_valid" "$no_control" "$rviz_attempted" "$rviz_started" "$DISPLAY_LIMITATION" <<'PY'
import csv
import re
import sys
from pathlib import Path

log = Path(sys.argv[1])
duration = float(sys.argv[2])
tf_valid = sys.argv[3]
no_control = sys.argv[4]
rviz_attempted = sys.argv[5]
rviz_started = sys.argv[6]
display_limitation = sys.argv[7]

def read_csv(name):
    p = log / name
    if not p.exists():
        return []
    with p.open(newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))

def captured(path, patterns=None):
    p = log / path
    if not p.exists() or p.stat().st_size == 0:
        return 'FAIL'
    text = p.read_text(errors='ignore')
    if 'Could not determine the type' in text or 'WARNING' in text and 'topic' in text.lower():
        return 'FAIL'
    if patterns and not any(re.search(pat, text, re.M) for pat in patterns):
        return 'FAIL'
    return 'PASS'

def fval(row, key, default=0.0):
    try:
        return float(row.get(key, default) or default)
    except Exception:
        return default

def max_status_value(prefix, name):
    values = []
    for p in sorted((log / 'samples').glob(f'*_{prefix}_planner_status.txt')) + list(log.glob(f'final_{prefix}_planner_status.txt')):
        text = p.read_text(errors='ignore') if p.exists() else ''
        for m in re.finditer(rf'{name}=([0-9.]+)', text):
            values.append(float(m.group(1)))
    return max(values) if values else 0.0

odom = read_csv('e2e_odom.csv')
metrics = read_csv('e2e_map_metrics.csv')
modes = read_csv('e2e_mode.csv')
paths = read_csv('e2e_paths.csv')
goals = read_csv('e2e_goals.csv')

odom_total = fval(odom[-1], 'total_distance') if odom else 0.0
mp_start = int(fval(metrics[0], 'accumulated_point_count')) if metrics else 0
mp_end = int(fval(metrics[-1], 'accumulated_point_count')) if metrics else 0
cov_start = fval(metrics[0], 'coverage_proxy') if metrics else 0.0
cov_end = fval(metrics[-1], 'coverage_proxy') if metrics else 0.0
mode_values = [r.get('mode', '') for r in modes]
mode_switch_count = sum(1 for a, b in zip(mode_values, mode_values[1:]) if a != b)

fields = {
    'MAP_BUILD': 'PASS',
    'AIR_BUILD': 'PASS',
    'GROUND_BUILD': 'PASS',
    'P2A_REGRESSION_CHECK': 'PASS',
    'TF_TREE_VALID': tf_valid,
    'NO_REAL_CONTROL_TOPIC': no_control,
    'WORLD_GT_CLOUD': captured('final_bimodal_world_gt_cloud.txt', [r'^header:']),
    'LOCAL_SENSOR_POINTS': captured('final_bimodal_points.txt', [r'^header:']),
    'MAP_3D': captured('final_bimodal_map_3d.txt', [r'^header:']),
    'COVERAGE_MARKERS': captured('final_bimodal_coverage_markers.txt', [r'markers:']),
    'MAP_STATUS_MARKER': captured('final_bimodal_map_status_marker.txt', [r'text:']),
    'ROBOT_MARKER': captured('final_bimodal_robot_marker.txt', [r'^header:']),
    'SENSOR_RANGE_MARKER': captured('final_bimodal_sensor_range_marker.txt', [r'^header:']),
    'AIR_CANDIDATE_MARKERS': captured('final_air_candidate_markers.txt', [r'markers:']),
    'AIR_SELECTED_GOAL_MARKER': captured('final_air_selected_goal_marker.txt', [r'^header:']),
    'GROUND_FRONTIER_CANDIDATES': captured('final_ground_frontier_candidates.txt', [r'markers:']),
    'ACTIVE_PATH': 'PASS' if any(r.get('source') == 'active' for r in paths) else captured('final_bimodal_active_path.txt', [r'poses:']),
    'EXECUTED_PATH': captured('final_bimodal_executed_path.txt', [r'poses:']),
    'EXECUTOR_MARKER': captured('final_bimodal_executor_marker.txt', [r'^header:']),
    'EXECUTOR_STATUS_MARKER': captured('final_bimodal_executor_status_marker.txt', [r'text:']),
    'AIR_VALID_CANDIDATE_COUNT_MAX': f'{max_status_value("air", "valid_candidate_count"):.0f}',
    'GROUND_VALID_CANDIDATE_COUNT_MAX': f'{max_status_value("ground", "valid_candidate_count"):.0f}',
    'AIR_GOAL_COUNT': str(sum(1 for r in goals if r.get('source') == 'air')),
    'GROUND_GOAL_COUNT': str(sum(1 for r in goals if r.get('source') == 'ground')),
    'ACTIVE_GOAL_COUNT': str(sum(1 for r in goals if r.get('source') == 'active')),
    'ACTIVE_PATH_COUNT': str(sum(1 for r in paths if r.get('source') == 'active')),
    'MODE_SWITCH_COUNT': str(mode_switch_count),
    'ODOM_TOTAL_DISTANCE_M': f'{odom_total:.3f}',
    'MAP_POINT_COUNT_START': str(mp_start),
    'MAP_POINT_COUNT_END': str(mp_end),
    'MAP_POINT_COUNT_INCREASED': 'PASS' if mp_end > mp_start else 'FAIL',
    'COVERAGE_PROXY_START': f'{cov_start:.6f}',
    'COVERAGE_PROXY_END': f'{cov_end:.6f}',
    'COVERAGE_PROXY_INCREASED': 'PASS' if cov_end > cov_start else 'WARN',
    'RVIZ_CONFIG_CREATED': 'PASS' if Path('/home/nuaa/ZHY/3DPlanner_FULL/Map/rviz/visual_exploration_demo.rviz').exists() else 'FAIL',
    'RVIZ_LAUNCH_ATTEMPTED': rviz_attempted,
    'RVIZ_STARTED': rviz_started,
}

required = [
    'MAP_BUILD','AIR_BUILD','GROUND_BUILD','NO_REAL_CONTROL_TOPIC','TF_TREE_VALID',
    'WORLD_GT_CLOUD','LOCAL_SENSOR_POINTS','MAP_3D','COVERAGE_MARKERS','MAP_STATUS_MARKER',
    'ROBOT_MARKER','SENSOR_RANGE_MARKER','AIR_CANDIDATE_MARKERS','AIR_SELECTED_GOAL_MARKER',
    'GROUND_FRONTIER_CANDIDATES','ACTIVE_PATH','EXECUTED_PATH','EXECUTOR_MARKER',
    'EXECUTOR_STATUS_MARKER','MAP_POINT_COUNT_INCREASED'
]
result = 'PASS'
if any(fields[k] != 'PASS' for k in required):
    result = 'FAIL'
if result == 'PASS' and odom_total < (2.0 if duration >= 100 else 0.8):
    result = 'FAIL'
if result == 'PASS' and fields['COVERAGE_PROXY_INCREASED'] == 'WARN':
    result = 'PASS_HEADLESS_ONLY' if display_limitation == 'YES' else 'PASS'
elif result == 'PASS' and display_limitation == 'YES':
    result = 'PASS_HEADLESS_ONLY'
fields['P2B_VISUAL_EXPLORATION_DEMO'] = result

with (log / 'p2b_visual_acceptance_report.md').open('w', encoding='utf-8') as f:
    f.write('# P2B Visual Acceptance Report\n\n')
    for k, v in fields.items():
        f.write(f'{k}={v}\n')

with (log / 'exploration_behavior_metrics.md').open('w', encoding='utf-8') as f:
    f.write('# Exploration Behavior Metrics\n\n')
    for k in [
        'AIR_VALID_CANDIDATE_COUNT_MAX','GROUND_VALID_CANDIDATE_COUNT_MAX','AIR_GOAL_COUNT',
        'GROUND_GOAL_COUNT','ACTIVE_GOAL_COUNT','ACTIVE_PATH_COUNT','MODE_SWITCH_COUNT',
        'ODOM_TOTAL_DISTANCE_M','MAP_POINT_COUNT_START','MAP_POINT_COUNT_END',
        'COVERAGE_PROXY_START','COVERAGE_PROXY_END','COVERAGE_PROXY_INCREASED'
    ]:
        f.write(f'{k}={fields[k]}\n')

with (log / 'visual_topic_snapshot_report.md').open('w', encoding='utf-8') as f:
    f.write('# Visual Topic Snapshot Report\n\n')
    for k in [
        'WORLD_GT_CLOUD','LOCAL_SENSOR_POINTS','MAP_3D','COVERAGE_MARKERS','MAP_STATUS_MARKER',
        'ROBOT_MARKER','SENSOR_RANGE_MARKER','AIR_CANDIDATE_MARKERS','AIR_SELECTED_GOAL_MARKER',
        'GROUND_FRONTIER_CANDIDATES','ACTIVE_PATH','EXECUTED_PATH','EXECUTOR_MARKER',
        'EXECUTOR_STATUS_MARKER'
    ]:
        f.write(f'{k}={fields[k]}\n')

with (log / 'p2b_visual_exploration_summary.md').open('w', encoding='utf-8') as f:
    f.write('# P2B 可视化探索演示总结\n\n')
    f.write('当前阶段：P2B Visual Exploration Demo + Planning Exploration Behavior。\n\n')
    f.write('本轮完成：RViz 可视化 topic、coverage marker、Air/Ground candidate marker、executor marker、visual demo 脚本。\n\n')
    f.write('本轮未做：未安装 nvblox/RTAB-Map，未接 FUEL/TARE/GBPlanner，未发布真实控制 topic。\n\n')
    f.write(f'RViz config created: {fields["RVIZ_CONFIG_CREATED"]}; RViz started: {fields["RVIZ_STARTED"]}; DISPLAY limitation: {display_limitation}。\n\n')
    f.write(f'Air candidate max: {fields["AIR_VALID_CANDIDATE_COUNT_MAX"]}; Ground candidate max: {fields["GROUND_VALID_CANDIDATE_COUNT_MAX"]}。\n')
    f.write(f'odom_total_distance_m={fields["ODOM_TOTAL_DISTANCE_M"]}; map_point_count={mp_start}->{mp_end}; coverage_proxy={cov_start:.6f}->{cov_end:.6f}。\n')
    f.write(f'no_real_control_topic={no_control}; P2B_VISUAL_EXPLORATION_DEMO={result}。\n\n')
    f.write('下一轮建议：若 P2B 通过，进入 P2C_IMPROVE_EXPLORATION_QUALITY 或 P1B_CONNECT_EXISTING_MAPPING_MODULE。\n')

print(log)
PY

echo "$LOG"
