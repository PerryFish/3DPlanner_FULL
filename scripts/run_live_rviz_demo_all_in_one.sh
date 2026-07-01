#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
MODE_SWITCH_PERIOD=60
KEEP_AFTER_RVIZ=NO
NO_CLEANUP=NO

while [ "$#" -gt 0 ]; do
  case "$1" in
    --mode-switch-period) MODE_SWITCH_PERIOD="$2"; shift 2 ;;
    --keep-after-rviz) KEEP_AFTER_RVIZ=YES; shift ;;
    --no-cleanup) NO_CLEANUP=YES; KEEP_AFTER_RVIZ=YES; shift ;;
    *) echo "unknown_arg=$1" >&2; exit 2 ;;
  esac
done

set +u
source "$ROOT/scripts/env_visual_demo.sh"

LOG="$ROOT/test-log/$(date +%Y%m%d_%H%M%S)_p2c_live_all_in_one"
mkdir -p "$LOG"
export P2C_LIVE_DEMO_LOG_DIR="$LOG"

pids=()
cleanup() {
  if [ "$NO_CLEANUP" = YES ] || [ "$KEEP_AFTER_RVIZ" = YES ]; then
    echo "BACKGROUND_VISUAL_DEMO_LEFT_RUNNING=YES"
    return
  fi
  for p in "${pids[@]}"; do kill "$p" 2>/dev/null || true; done
  sleep 2
  for p in "${pids[@]}"; do kill -KILL "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'cleanup; exit 0' INT TERM

bash "$ROOT/scripts/start_visual_demo_keepalive.sh" --mode-switch-period "$MODE_SWITCH_PERIOD" --log-dir "$LOG/demo" > "$LOG/keepalive_stdout.log" 2>&1 &
pids+=($!)
echo "LIVE_VISUAL_DEMO_STARTED=YES"
sleep 2

if bash "$ROOT/scripts/check_rviz_tf_ready.sh" --wait 30 > "$LOG/tf_ready_before_rviz.log" 2>&1; then
  echo "TF_READY_BEFORE_RVIZ=PASS"
else
  echo "TF_READY_BEFORE_RVIZ=FAIL"
  cat "$LOG/tf_ready_before_rviz.log"
  exit 5
fi

if bash "$ROOT/scripts/check_visual_topics_ready.sh" > "$LOG/visual_topics_before_rviz.log" 2>&1; then
  echo "VISUAL_TOPICS_READY_BEFORE_RVIZ=PASS"
else
  echo "VISUAL_TOPICS_READY_BEFORE_RVIZ=FAIL"
  cat "$LOG/visual_topics_before_rviz.log"
  exit 6
fi

echo "RVIZ_STARTED=YES"
bash "$ROOT/scripts/run_rviz_visual_exploration.sh"

if [ "$KEEP_AFTER_RVIZ" = NO ] && [ "$NO_CLEANUP" = NO ]; then
  answer=""
  read -r -t 20 -p "Clean up background demo processes? [Y/n] " answer || answer=""
  case "$answer" in
    n|N|no|NO) KEEP_AFTER_RVIZ=YES ;;
    *) KEEP_AFTER_RVIZ=NO ;;
  esac
fi
