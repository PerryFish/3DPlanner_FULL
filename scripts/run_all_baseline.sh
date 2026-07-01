#!/usr/bin/env bash
set -u
ROOT=/home/nuaa/ZHY/3DPlanner_FULL
TS=$(date +%Y%m%d_%H%M%S)
mkdir -p "$ROOT/Map/test-log/$TS" "$ROOT/Air/test-log/$TS" "$ROOT/Ground/test-log/$TS" "$ROOT/test-log/$TS"
if command -v tmux >/dev/null 2>&1; then
  SESSION=bimodal_baseline_$TS
  tmux new-session -d -s "$SESSION" "$ROOT/scripts/run_map.sh > '$ROOT/Map/test-log/$TS/map_runtime.log' 2>&1"
  tmux new-window -t "$SESSION" "$ROOT/scripts/run_air.sh > '$ROOT/Air/test-log/$TS/air_runtime.log' 2>&1"
  tmux new-window -t "$SESSION" "$ROOT/scripts/run_ground.sh > '$ROOT/Ground/test-log/$TS/ground_runtime.log' 2>&1"
  tmux new-window -t "$SESSION" "$ROOT/scripts/run_mux.sh > '$ROOT/test-log/$TS/integration_runtime.log' 2>&1"
  echo "tmux_session=$SESSION"
else
  "$ROOT/scripts/run_map.sh" > "$ROOT/Map/test-log/$TS/map_runtime.log" 2>&1 &
  "$ROOT/scripts/run_air.sh" > "$ROOT/Air/test-log/$TS/air_runtime.log" 2>&1 &
  "$ROOT/scripts/run_ground.sh" > "$ROOT/Ground/test-log/$TS/ground_runtime.log" 2>&1 &
  "$ROOT/scripts/run_mux.sh" > "$ROOT/test-log/$TS/integration_runtime.log" 2>&1 &
  echo "background_pids=$!"
fi
echo "log_dir=$ROOT/test-log/$TS"
