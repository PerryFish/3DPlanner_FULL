#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL

set +u
source "$ROOT/scripts/env_visual_demo.sh" >/tmp/visual_topic_watch_env.log 2>&1

topics=(
  /tf
  /tf_static
  /bimodal/tf_guard_status
  /bimodal/world_gt_cloud
  /bimodal/points
  /bimodal/map_3d
  /bimodal/coverage_markers
  /air/candidate_markers
  /ground/frontier_candidates
  /bimodal/active_path
  /bimodal/executed_path
  /bimodal/robot_marker
  /bimodal/executor_status_marker
)

while true; do
  clear 2>/dev/null || true
  date
  for t in "${topics[@]}"; do
    extra_args=()
    if [ "$t" = "/tf_static" ]; then
      extra_args=(--qos-durability transient_local)
    fi
    if timeout 1.8 ros2 topic echo "$t" --once --no-daemon "${extra_args[@]}" >/tmp/visual_topic_watch.out 2>&1; then
      echo "PASS $t"
    else
      echo "WAIT $t"
    fi
  done
  sleep 2
done
