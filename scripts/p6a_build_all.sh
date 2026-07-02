#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
TS=$(date +%Y%m%d_%H%M%S)
LOG="${P6A_LOG_DIR:-$ROOT/test-log/${TS}_p6a_jetson_deployment_prep}"
mkdir -p "$LOG"
SUMMARY="$LOG/p6a_build_summary.md"

map_build=FAIL
air_build=FAIL
ground_build=FAIL

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/humble/setup.bash
  set -u
else
  {
    echo "# P6A Build Summary"
    echo "P6A_BUILD_ALL=FAIL"
    echo "failure_reason=ROS2 Humble setup missing"
  } > "$SUMMARY"
  cat "$SUMMARY"
  exit 2
fi

build_ws() {
  local name="$1"
  local ws="$ROOT/$name/ros2_ws"
  local log_file="$LOG/p6a_build_${name,,}.log"
  if [ ! -d "$ws" ]; then
    echo "$name workspace missing: $ws" > "$log_file"
    return 1
  fi
  (
    cd "$ws"
    colcon build --symlink-install
  ) > "$log_file" 2>&1
}

if build_ws Map; then map_build=PASS; fi
if build_ws Air; then air_build=PASS; fi
if build_ws Ground; then ground_build=PASS; fi

overall=PASS
if [ "$map_build" != PASS ] || [ "$air_build" != PASS ] || [ "$ground_build" != PASS ]; then
  overall=FAIL
fi

cat > "$SUMMARY" <<EOF
# P6A Build Summary

sample_time=$(date -Is)
root=$ROOT
log_dir=$LOG
map_build=$map_build
air_build=$air_build
ground_build=$ground_build
P6A_BUILD_ALL=$overall

After successful build, source:

\`\`\`bash
source /opt/ros/humble/setup.bash
source $ROOT/Map/ros2_ws/install/setup.bash
source $ROOT/Air/ros2_ws/install/setup.bash
source $ROOT/Ground/ros2_ws/install/setup.bash
\`\`\`
EOF

cat "$SUMMARY"
[ "$overall" = PASS ]
