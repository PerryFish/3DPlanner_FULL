#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
DURATION=300
MODE_SWITCH_PERIOD=75
INPUT_TOPIC=/points_raw

while [ "$#" -gt 0 ]; do
  case "$1" in
    --duration) DURATION="$2"; shift 2 ;;
    --mode-switch-period) MODE_SWITCH_PERIOD="$2"; shift 2 ;;
    --input-topic) INPUT_TOPIC="$2"; shift 2 ;;
    *) echo "unknown_arg=$1" >&2; exit 2 ;;
  esac
done

exec "$ROOT/scripts/run_p1c_real_sensor_pointcloud_input_validation.sh" \
  --duration "$DURATION" --mode-switch-period "$MODE_SWITCH_PERIOD" --input-topic "$INPUT_TOPIC" --rviz --skip-build
