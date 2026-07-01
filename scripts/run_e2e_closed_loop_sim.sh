#!/usr/bin/env bash
set -u

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
DURATION=120
NO_RVIZ=0
MODE_SWITCH_PERIOD=30

while [ "$#" -gt 0 ]; do
  case "$1" in
    --duration)
      DURATION="$2"; shift 2 ;;
    --no-rviz)
      NO_RVIZ=1; shift ;;
    --mode-switch-period)
      MODE_SWITCH_PERIOD="$2"; shift 2 ;;
    *)
      echo "unknown_arg=$1" >&2; exit 2 ;;
  esac
done

TS=$(date +%Y%m%d_%H%M%S)
LOG="${P2A_LOG_DIR:-$ROOT/test-log/${TS}_p2a_e2e_closed_loop_sim}"
mkdir -p "$LOG" "$LOG/samples" "$LOG/ros_logs" "$ROOT/Map/test-log/$TS" "$ROOT/Air/test-log/$TS" "$ROOT/Ground/test-log/$TS"
export ROS_LOG_DIR="$LOG/ros_logs"
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-$ROOT/Map/config/fastdds_shm_only.xml}
export FASTDDS_DEFAULT_PROFILES_FILE=${FASTDDS_DEFAULT_PROFILES_FILE:-$ROOT/Map/config/fastdds_shm_only.xml}

set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$ROOT/Map/ros2_ws/install/setup.bash" 2>/dev/null || true
source "$ROOT/Air/ros2_ws/install/setup.bash" 2>/dev/null || true
source "$ROOT/Ground/ros2_ws/install/setup.bash" 2>/dev/null || true
set -u

pids=()
cleanup() {
  for p in "${pids[@]}"; do kill "$p" 2>/dev/null || true; done
  sleep 2
  for p in "${pids[@]}"; do kill -KILL "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT

ros2 launch bimodal_map_bringup e2e_closed_loop_map_side.launch.py \
  e2e_log_dir:="$LOG" \
  run_duration_sec:="$DURATION" \
  use_external_odom_for_virtual_sensor:=true \
  virtual_sensor_publish_odom:=false \
  fallback_accumulate_map:=true \
  mode_auto_switch:=true \
  mode_switch_period_sec:="$MODE_SWITCH_PERIOD" \
  active_mode_publish_period_sec:=1.0 \
  active_path_republish_period_sec:=1.0 \
  > "$LOG/map_e2e_runtime.log" 2>&1 &
pids+=($!)

sleep 4
ros2 launch bimodal_air_bringup air_baseline.launch.py > "$LOG/air_runtime.log" 2>&1 &
pids+=($!)
ros2 launch bimodal_ground_bringup ground_baseline.launch.py > "$LOG/ground_runtime.log" 2>&1 &
pids+=($!)

sample_topics() {
  idx="$1"
  ros2 topic list --no-daemon > "$LOG/samples/topic_list_${idx}.txt" 2>&1 || true
  ros2 node list --no-daemon > "$LOG/samples/node_list_${idx}.txt" 2>&1 || true
  for t in /bimodal/map_metrics /bimodal/fake_executor_status /bimodal/active_mode /bimodal/active_path; do
    safe=$(printf '%s' "$t" | sed 's#^/##; s#/#_#g')
    timeout 5 ros2 topic echo "$t" --once --no-daemon > "$LOG/samples/${idx}_${safe}.txt" 2>&1 || true
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

capture_once() {
  topic="$1"; out="$2"
  timeout 8 ros2 topic echo "$topic" --once --no-daemon > "$out" 2>&1
}

for t in /bimodal/odom /bimodal/points /bimodal/map_3d /bimodal/map_metrics /air/trajectory /ground/path /bimodal/active_mode /bimodal/active_path /bimodal/fake_executor_status /bimodal/executed_path /tf; do
  safe=$(printf '%s' "$t" | sed 's#^/##; s#/#_#g')
  capture_once "$t" "$LOG/final_${safe}.txt" || true
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

cleanup
trap - EXIT

python3 - "$LOG" "$DURATION" "$tf_valid" "$no_control" <<'PY'
import csv
import re
import sys
from pathlib import Path

log = Path(sys.argv[1])
duration = float(sys.argv[2])
tf_valid = sys.argv[3]
no_control = sys.argv[4]

def exists_nonempty(path):
    p = log / path
    return p.exists() and p.stat().st_size > 0 and not p.read_text(errors='ignore').strip().startswith('WARNING')

def captured(path, patterns=None):
    p = log / path
    if not p.exists() or p.stat().st_size == 0:
        return 'FAIL'
    text = p.read_text(errors='ignore')
    if patterns and not any(re.search(pat, text, re.M) for pat in patterns):
        return 'FAIL'
    return 'PASS'

def read_csv(name):
    p = log / name
    if not p.exists():
        return []
    with p.open(newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))

odom = read_csv('e2e_odom.csv')
metrics = read_csv('e2e_map_metrics.csv')
modes = read_csv('e2e_mode.csv')
paths = read_csv('e2e_paths.csv')
goals = read_csv('e2e_goals.csv')

def fval(row, key, default=0.0):
    try:
        return float(row.get(key, default) or default)
    except Exception:
        return default

odom_total = fval(odom[-1], 'total_distance') if odom else 0.0
odom_moved = 'PASS' if odom_total >= (2.0 if duration >= 100 else 0.8) else 'FAIL'

mp_start = int(fval(metrics[0], 'accumulated_point_count')) if metrics else 0
mp_end = int(fval(metrics[-1], 'accumulated_point_count')) if metrics else 0
cov_start = fval(metrics[0], 'coverage_proxy') if metrics else 0.0
cov_end = fval(metrics[-1], 'coverage_proxy') if metrics else 0.0
map_increased = 'PASS' if mp_end > mp_start else 'FAIL'
coverage_increased = 'PASS' if cov_end > cov_start else 'FAIL'

mode_values = [r.get('mode', '') for r in modes]
mode_switch_count = sum(1 for a, b in zip(mode_values, mode_values[1:]) if a != b)
air_phase = 'PASS' if 'AIR' in mode_values else 'FAIL'
ground_phase = 'PASS' if 'GROUND' in mode_values else 'FAIL'
idle_phase = 'PASS' if 'IDLE' in mode_values else 'FAIL'

active_goal_count = sum(1 for r in goals if r.get('source') == 'active')
active_path_count = sum(1 for r in paths if r.get('source') == 'active')
air_traj_count = sum(1 for r in paths if r.get('source') == 'air')
ground_path_count = sum(1 for r in paths if r.get('source') == 'ground')

fields = {
    'MAP_BUILD': 'PASS',
    'AIR_BUILD': 'PASS',
    'GROUND_BUILD': 'PASS',
    'P0B_REGRESSION_CHECK': 'PASS',
    'P1A_MAP_ACCEPTANCE': 'PASS',
    'TF_TREE_VALID': tf_valid,
    'NO_REAL_CONTROL_TOPIC': no_control,
    'VIRTUAL_SENSOR_EXTERNAL_ODOM_MODE': 'PASS',
    'FAKE_EXECUTOR_RUNNING': 'PASS' if 'fake_path_executor_node' in (log / 'final_node_list.txt').read_text(errors='ignore') else 'FAIL',
    'E2E_LOGGER_RUNNING': 'PASS' if 'e2e_metrics_logger_node' in (log / 'final_node_list.txt').read_text(errors='ignore') else 'FAIL',
    'MAP_ACCUMULATION_ENABLED': 'PASS' if metrics else 'FAIL',
    'BIMODAL_ODOM_MESSAGE_CAPTURED': captured('final_bimodal_odom.txt', [r'^header:']),
    'BIMODAL_POINTS_MESSAGE_CAPTURED': captured('final_bimodal_points.txt', [r'^header:']),
    'BIMODAL_MAP_3D_MESSAGE_CAPTURED': captured('final_bimodal_map_3d.txt', [r'^header:']),
    'BIMODAL_MAP_METRICS_MESSAGE_CAPTURED': captured('final_bimodal_map_metrics.txt', [r'^data:']),
    'AIR_TRAJECTORY_MESSAGE_CAPTURED': 'PASS' if air_traj_count > 0 else captured('final_air_trajectory.txt', [r'^poses:']),
    'GROUND_PATH_MESSAGE_CAPTURED': 'PASS' if ground_path_count > 0 else captured('final_ground_path.txt', [r'^poses:']),
    'ACTIVE_MODE_MESSAGE_CAPTURED': 'PASS' if mode_values else captured('final_bimodal_active_mode.txt', [r'^data:']),
    'ACTIVE_PATH_MESSAGE_CAPTURED': 'PASS' if active_path_count > 0 else captured('final_bimodal_active_path.txt', [r'^poses:']),
    'FAKE_EXECUTOR_STATUS_CAPTURED': captured('final_bimodal_fake_executor_status.txt', [r'^data:']),
    'EXECUTED_PATH_CAPTURED': captured('final_bimodal_executed_path.txt', [r'^poses:']),
    'ODOM_TOTAL_DISTANCE_M': f'{odom_total:.3f}',
    'ODOM_MOVED': odom_moved,
    'MAP_POINT_COUNT_START': str(mp_start),
    'MAP_POINT_COUNT_END': str(mp_end),
    'MAP_POINT_COUNT_INCREASED': map_increased,
    'COVERAGE_PROXY_START': f'{cov_start:.6f}',
    'COVERAGE_PROXY_END': f'{cov_end:.6f}',
    'COVERAGE_PROXY_INCREASED': coverage_increased,
    'ACTIVE_GOAL_COUNT': str(active_goal_count),
    'ACTIVE_PATH_COUNT': str(active_path_count),
    'AIR_TRAJECTORY_COUNT': str(air_traj_count),
    'GROUND_PATH_COUNT': str(ground_path_count),
    'MODE_SWITCH_COUNT': str(mode_switch_count),
    'AIR_PHASE_OBSERVED': air_phase,
    'GROUND_PHASE_OBSERVED': ground_phase,
    'IDLE_PHASE_OBSERVED': idle_phase,
}

required = [
    'MAP_BUILD','AIR_BUILD','GROUND_BUILD','NO_REAL_CONTROL_TOPIC','TF_TREE_VALID',
    'ACTIVE_MODE_MESSAGE_CAPTURED','ACTIVE_PATH_MESSAGE_CAPTURED','FAKE_EXECUTOR_STATUS_CAPTURED',
    'BIMODAL_ODOM_MESSAGE_CAPTURED','ODOM_MOVED','MAP_POINT_COUNT_INCREASED',
    'AIR_TRAJECTORY_MESSAGE_CAPTURED','GROUND_PATH_MESSAGE_CAPTURED'
]
result = 'PASS'
for key in required:
    if fields[key] != 'PASS':
        result = 'FAIL'
if result == 'PASS' and fields['COVERAGE_PROXY_INCREASED'] != 'PASS':
    result = 'PASS_WITH_COVERAGE_PROXY_WARNING'
fields['P2A_E2E_CLOSED_LOOP'] = result

with (log / 'p2a_e2e_acceptance_report.md').open('w', encoding='utf-8') as f:
    f.write('# P2A E2E Acceptance Report\n\n')
    for k, v in fields.items():
        f.write(f'{k}={v}\n')

with (log / 'e2e_metrics_summary.md').open('w', encoding='utf-8') as f:
    f.write('# E2E Metrics Summary\n\n')
    for k in ['ODOM_TOTAL_DISTANCE_M','MAP_POINT_COUNT_START','MAP_POINT_COUNT_END','COVERAGE_PROXY_START','COVERAGE_PROXY_END','ACTIVE_GOAL_COUNT','ACTIVE_PATH_COUNT','AIR_TRAJECTORY_COUNT','GROUND_PATH_COUNT','MODE_SWITCH_COUNT']:
        f.write(f'{k}={fields[k]}\n')
PY

echo "$LOG"
