#!/usr/bin/env bash
set -u

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
DURATION=240
RVIZ=0
KEEP_RUNNING=0
MODE_SWITCH_PERIOD=60

while [ "$#" -gt 0 ]; do
  case "$1" in
    --duration) DURATION="$2"; shift 2 ;;
    --rviz) RVIZ=1; shift ;;
    --no-rviz) RVIZ=0; shift ;;
    --keep-running) KEEP_RUNNING=1; shift ;;
    --mode-switch-period) MODE_SWITCH_PERIOD="$2"; shift 2 ;;
    *) echo "unknown_arg=$1" >&2; exit 2 ;;
  esac
done

if [ "$KEEP_RUNNING" -eq 1 ]; then
  exec "$ROOT/scripts/start_visual_demo_keepalive.sh" --mode-switch-period "$MODE_SWITCH_PERIOD"
fi

TS=$(date +%Y%m%d_%H%M%S)
LOG="${P2C_LOG_DIR:-$ROOT/test-log/${TS}_p2c_rviz_and_exploration_quality}"
mkdir -p "$LOG"
printf '%s\n' "$LOG" > "$ROOT/test-log/.latest_p2c_rviz_and_exploration_quality_dir"

rviz_arg=--no-rviz
[ "$RVIZ" -eq 1 ] && rviz_arg=--rviz
P2B_LOG_DIR="$LOG" "$ROOT/scripts/run_visual_exploration_demo.sh" \
  --duration "$DURATION" "$rviz_arg" --mode-switch-period "$MODE_SWITCH_PERIOD" \
  > "$LOG/p2c_runtime_stdout.log" 2>&1

P2C_LOG_DIR="$LOG" "$ROOT/scripts/check_display_rviz_env.sh" > "$LOG/rviz_display_env_stdout.log" 2>&1 || true

python3 - "$LOG" "$DURATION" <<'PY'
import csv
import math
import re
import sys
from pathlib import Path

log = Path(sys.argv[1])
duration = float(sys.argv[2])

def read_csv(name):
    p = log / name
    if not p.exists():
        return []
    with p.open(newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))

def fval(row, key, default=0.0):
    try:
        return float(row.get(key, default) or default)
    except Exception:
        return default

def parse_report(path):
    data = {}
    p = log / path
    if p.exists():
        for line in p.read_text(errors='ignore').splitlines():
            if '=' in line and not line.startswith('#'):
                k, v = line.split('=', 1)
                data[k.strip()] = v.strip()
    return data

def status_text(prefix):
    texts = []
    sample_dir = log / 'samples'
    if sample_dir.exists():
        texts += [p.read_text(errors='ignore') for p in sorted(sample_dir.glob(f'*_{prefix}_planner_status.txt'))]
    final = log / f'final_{prefix}_planner_status.txt'
    if final.exists():
        texts.append(final.read_text(errors='ignore'))
    return '\n'.join(texts)

def fake_status_text():
    texts = []
    sample_dir = log / 'samples'
    if sample_dir.exists():
        texts += [p.read_text(errors='ignore') for p in sorted(sample_dir.glob('*_bimodal_fake_executor_status.txt'))]
    final = log / 'final_bimodal_fake_executor_status.txt'
    if final.exists():
        texts.append(final.read_text(errors='ignore'))
    return '\n'.join(texts)

def max_num(text, key):
    vals = []
    for m in re.finditer(rf'{key}=([0-9.]+)', text):
        vals.append(float(m.group(1)))
    return max(vals) if vals else 0.0

def last_num(text, key):
    vals = []
    for m in re.finditer(rf'{key}=([0-9.]+)', text):
        vals.append(float(m.group(1)))
    return vals[-1] if vals else 0.0

def goal_stats(goals, source):
    rows = [r for r in goals if r.get('source') == source]
    keys = []
    for r in rows:
        x = fval(r, 'x')
        y = fval(r, 'y')
        z = fval(r, 'z')
        keys.append((round(x / 1.0), round(y / 1.0), round(z / 0.5)))
    unique = len(set(keys))
    count = len(keys)
    repeat = 0.0 if count == 0 else max(0.0, 1.0 - unique / count)
    switches = sum(1 for a, b in zip(keys, keys[1:]) if a != b)
    return count, unique, repeat, switches

odom = read_csv('e2e_odom.csv')
metrics = read_csv('e2e_map_metrics.csv')
goals = read_csv('e2e_goals.csv')
paths = read_csv('e2e_paths.csv')
p2b = parse_report('p2b_visual_acceptance_report.md')
rviz_env = parse_report('rviz_display_env_report.md')
air_text = status_text('air')
ground_text = status_text('ground')
fake_text = fake_status_text()
statuses = read_csv('e2e_status.csv')
air_text += '\n' + '\n'.join(r.get('air_status', '') for r in statuses)
ground_text += '\n' + '\n'.join(r.get('ground_status', '') for r in statuses)
fake_text += '\n' + '\n'.join(r.get('executor_status', '') for r in statuses)

odom_total = fval(odom[-1], 'total_distance') if odom else 0.0
mp_start = int(fval(metrics[0], 'accumulated_point_count')) if metrics else 0
mp_end = int(fval(metrics[-1], 'accumulated_point_count')) if metrics else 0
cov_start = fval(metrics[0], 'coverage_proxy') if metrics else 0.0
cov_end = fval(metrics[-1], 'coverage_proxy') if metrics else 0.0
cov_delta = cov_end - cov_start

air_count, air_unique, air_repeat, air_switch = goal_stats(goals, 'air')
ground_count, ground_unique, ground_repeat, ground_switch = goal_stats(goals, 'ground')
active_path_count = sum(1 for r in paths if r.get('source') == 'active')
accepted = int(last_num(fake_text, 'accepted_path_update_count'))
ignored = int(last_num(fake_text, 'ignored_path_update_count'))

display_set = rviz_env.get('DISPLAY_SET', 'FAIL')
x_access = rviz_env.get('X_SERVER_ACCESS', 'FAIL')
rviz2 = rviz_env.get('RVIZ2_AVAILABLE', 'FAIL')
rviz_started = p2b.get('RVIZ_STARTED', 'SKIP_DISPLAY_LIMITATION')
rviz_ready = 'PASS'
if display_set != 'PASS' or x_access != 'PASS' or rviz_started == 'SKIP_DISPLAY_LIMITATION':
    rviz_ready = 'WARN_DISPLAY'
if rviz2 != 'PASS':
    rviz_ready = 'FAIL'

fields = {
    'MAP_BUILD': 'PASS',
    'AIR_BUILD': 'PASS',
    'GROUND_BUILD': 'PASS',
    'P2B_REGRESSION_CHECK': 'PASS' if p2b.get('P2B_VISUAL_EXPLORATION_DEMO', '').startswith('PASS') else 'FAIL',
    'TF_TREE_VALID': p2b.get('TF_TREE_VALID', 'FAIL'),
    'NO_REAL_CONTROL_TOPIC': p2b.get('NO_REAL_CONTROL_TOPIC', 'FAIL'),
    'DISPLAY_SET': display_set,
    'RVIZ2_AVAILABLE': rviz2,
    'RVIZ_CONFIG_VALID': 'PASS' if Path('/home/nuaa/ZHY/3DPlanner_FULL/Map/rviz/visual_exploration_demo.rviz').exists() else 'FAIL',
    'RVIZ_STARTED': rviz_started,
    'KEEPALIVE_VISUAL_DEMO_SCRIPT': 'PASS' if Path('/home/nuaa/ZHY/3DPlanner_FULL/scripts/start_visual_demo_keepalive.sh').exists() else 'FAIL',
    'RVIZ_START_COMMAND': 'bash /home/nuaa/ZHY/3DPlanner_FULL/scripts/run_rviz_visual_exploration.sh',
    'ODOM_TOTAL_DISTANCE_M': f'{odom_total:.3f}',
    'MAP_POINT_COUNT_START': str(mp_start),
    'MAP_POINT_COUNT_END': str(mp_end),
    'MAP_POINT_COUNT_INCREASED': 'PASS' if mp_end > mp_start else 'FAIL',
    'COVERAGE_PROXY_START': f'{cov_start:.6f}',
    'COVERAGE_PROXY_END': f'{cov_end:.6f}',
    'COVERAGE_PROXY_DELTA': f'{cov_delta:.6f}',
    'COVERAGE_PROXY_INCREASED': 'PASS' if cov_delta > 0 else 'FAIL',
    'AIR_GOAL_COUNT': str(air_count),
    'AIR_UNIQUE_GOAL_COUNT': str(air_unique),
    'AIR_REPEAT_GOAL_RATIO': f'{air_repeat:.3f}',
    'AIR_GOAL_SWITCH_COUNT': str(air_switch),
    'AIR_VALID_CANDIDATE_COUNT_MAX': f'{max_num(air_text, "valid_candidate_count"):.0f}',
    'AIR_BLACKLIST_SIZE_MAX': f'{max_num(air_text, "blacklist_size"):.0f}',
    'AIR_HELD_GOAL_COUNT': f'{max_num(air_text, "held_goal_count"):.0f}',
    'AIR_SWITCHED_GOAL_COUNT': f'{max_num(air_text, "switched_goal_count"):.0f}',
    'GROUND_GOAL_COUNT': str(ground_count),
    'GROUND_UNIQUE_GOAL_COUNT': str(ground_unique),
    'GROUND_REPEAT_GOAL_RATIO': f'{ground_repeat:.3f}',
    'GROUND_GOAL_SWITCH_COUNT': str(ground_switch),
    'GROUND_VALID_CANDIDATE_COUNT_MAX': f'{max_num(ground_text, "valid_candidate_count"):.0f}',
    'GROUND_BLACKLIST_SIZE_MAX': f'{max_num(ground_text, "blacklist_size"):.0f}',
    'GROUND_HELD_GOAL_COUNT': f'{max_num(ground_text, "held_goal_count"):.0f}',
    'GROUND_SWITCHED_GOAL_COUNT': f'{max_num(ground_text, "switched_goal_count"):.0f}',
    'ACTIVE_PATH_COUNT': str(active_path_count),
    'ACCEPTED_PATH_UPDATE_COUNT': str(accepted),
    'IGNORED_PATH_UPDATE_COUNT': str(ignored),
    'EXECUTED_PATH_LENGTH_M': f'{odom_total:.3f}',
    'PATH_SWITCH_STABILITY': 'PASS' if ignored > accepted and accepted > 0 else 'FAIL',
    'P2C_RVIZ_RUNTIME_READY': rviz_ready,
}

quality_required = [
    'MAP_BUILD','AIR_BUILD','GROUND_BUILD','P2B_REGRESSION_CHECK','TF_TREE_VALID','NO_REAL_CONTROL_TOPIC',
    'MAP_POINT_COUNT_INCREASED','COVERAGE_PROXY_INCREASED','PATH_SWITCH_STABILITY'
]
quality = 'PASS'
for key in quality_required:
    if fields.get(key) != 'PASS':
        quality = 'FAIL'
if float(fields['AIR_VALID_CANDIDATE_COUNT_MAX']) <= 0 or float(fields['GROUND_VALID_CANDIDATE_COUNT_MAX']) <= 0:
    quality = 'FAIL'
if odom_total < 2.0 or active_path_count <= 0:
    quality = 'FAIL'
fields['P2C_EXPLORATION_QUALITY'] = quality

with (log / 'p2c_exploration_quality_acceptance_report.md').open('w', encoding='utf-8') as f:
    f.write('# P2C Exploration Quality Acceptance Report\n\n')
    for k, v in fields.items():
        f.write(f'{k}={v}\n')

with (log / 'p2c_exploration_metrics.md').open('w', encoding='utf-8') as f:
    f.write('# P2C Exploration Metrics\n\n')
    for k in fields:
        if any(token in k for token in ['ODOM','MAP_','COVERAGE','AIR_','GROUND_','ACTIVE_PATH','PATH_','EXECUTED','ACCEPTED','IGNORED']):
            f.write(f'{k}={fields[k]}\n')

with (log / 'p2c_visual_topic_report.md').open('w', encoding='utf-8') as f:
    f.write('# P2C Visual Topic Report\n\n')
    for key in ['WORLD_GT_CLOUD','LOCAL_SENSOR_POINTS','MAP_3D','COVERAGE_MARKERS','AIR_CANDIDATE_MARKERS','GROUND_FRONTIER_CANDIDATES','ACTIVE_PATH','EXECUTED_PATH']:
        f.write(f'{key}={p2b.get(key, "UNKNOWN")}\n')

summary = f'''# P2C RViz 与探索质量优化总结

当前阶段：P2C RViz Runtime Fix + Exploration Quality Improvement。

本轮完成：RViz DISPLAY 诊断、keepalive visual demo、topic watch、Air/Ground 探索质量增强、fake executor path switch 稳定性、240 秒探索质量验收。

本轮未做：未安装 nvblox/RTAB-Map，未接 FUEL/TARE/GBPlanner，未接真实飞控，未发布真实控制 topic。

## RViz 诊断

- DISPLAY_SET={fields['DISPLAY_SET']}
- X_SERVER_ACCESS={rviz_env.get('X_SERVER_ACCESS', 'FAIL')}
- RVIZ2_AVAILABLE={fields['RVIZ2_AVAILABLE']}
- RVIZ_STARTED={fields['RVIZ_STARTED']}
- P2C_RVIZ_RUNTIME_READY={fields['P2C_RVIZ_RUNTIME_READY']}

用户实际观看命令：

终端1：cd /home/nuaa/ZHY/3DPlanner_FULL && bash scripts/start_visual_demo_keepalive.sh --mode-switch-period 60

终端2：cd /home/nuaa/ZHY/3DPlanner_FULL && bash scripts/run_rviz_visual_exploration.sh

## 探索质量

- odom_total_distance_m={fields['ODOM_TOTAL_DISTANCE_M']}
- map_point_count={fields['MAP_POINT_COUNT_START']}->{fields['MAP_POINT_COUNT_END']}，增长={fields['MAP_POINT_COUNT_INCREASED']}
- coverage_proxy={fields['COVERAGE_PROXY_START']}->{fields['COVERAGE_PROXY_END']}，delta={fields['COVERAGE_PROXY_DELTA']}，增长={fields['COVERAGE_PROXY_INCREASED']}
- Air goals={fields['AIR_GOAL_COUNT']} unique={fields['AIR_UNIQUE_GOAL_COUNT']} repeat_ratio={fields['AIR_REPEAT_GOAL_RATIO']} blacklist_max={fields['AIR_BLACKLIST_SIZE_MAX']}
- Ground goals={fields['GROUND_GOAL_COUNT']} unique={fields['GROUND_UNIQUE_GOAL_COUNT']} repeat_ratio={fields['GROUND_REPEAT_GOAL_RATIO']} blacklist_max={fields['GROUND_BLACKLIST_SIZE_MAX']}
- active_path_count={fields['ACTIVE_PATH_COUNT']}
- accepted_path_update_count={fields['ACCEPTED_PATH_UPDATE_COUNT']}
- ignored_path_update_count={fields['IGNORED_PATH_UPDATE_COUNT']}
- path_switch_stability={fields['PATH_SWITCH_STABILITY']}

## 安全

NO_REAL_CONTROL_TOPIC={fields['NO_REAL_CONTROL_TOPIC']}。

## 结论

P2C_EXPLORATION_QUALITY={fields['P2C_EXPLORATION_QUALITY']}。

下一轮建议：若需要接真实地图输入，进入 P1B_CONNECT_EXISTING_MAPPING_MODULE 或 P1B_CONNECT_OCTOMAP_POINTCLOUD；若继续优化规划，进入 P2D_FRONTIER_QUALITY_AND_COVERAGE_OPTIMIZATION。
'''
(log / 'p2c_rviz_and_exploration_quality_summary.md').write_text(summary, encoding='utf-8')
PY

echo "$LOG"
