#!/usr/bin/env bash
set -u
ROOT=/home/nuaa/ZHY/3DPlanner_FULL
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-$ROOT/Map/config/fastdds_shm_only.xml}
export FASTDDS_DEFAULT_PROFILES_FILE=${FASTDDS_DEFAULT_PROFILES_FILE:-$ROOT/Map/config/fastdds_shm_only.xml}
LOG_DIR="${1:-$(find "$ROOT/test-log" -mindepth 1 -maxdepth 1 -type d | sort | tail -1)}"
TMP="$ROOT/test-log/debug_package_tmp"
rm -rf "$TMP"
mkdir -p "$TMP"
cp -r "$ROOT/Map/docs" "$TMP/Map_docs" 2>/dev/null || true
cp -r "$ROOT/Air/docs" "$TMP/Air_docs" 2>/dev/null || true
cp -r "$ROOT/Ground/docs" "$TMP/Ground_docs" 2>/dev/null || true
cp -r "$ROOT/Map/config" "$TMP/Map_config" 2>/dev/null || true
cp -r "$ROOT/Air/config" "$TMP/Air_config" 2>/dev/null || true
cp -r "$ROOT/Ground/config" "$TMP/Ground_config" 2>/dev/null || true
find "$ROOT/Map/ros2_ws/src" "$ROOT/Air/ros2_ws/src" "$ROOT/Ground/ros2_ws/src" \( -name package.xml -o -name setup.py -o -path '*/launch/*.launch.py' \) -print0 | xargs -0 -I{} cp --parents {} "$TMP" 2>/dev/null || true
cp -r "$ROOT/scripts" "$TMP/scripts" 2>/dev/null || true
cp -r "$LOG_DIR" "$TMP/latest_test_log" 2>/dev/null || true
set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$ROOT/Map/ros2_ws/install/setup.bash" 2>/dev/null || true
source "$ROOT/Air/ros2_ws/install/setup.bash" 2>/dev/null || true
source "$ROOT/Ground/ros2_ws/install/setup.bash" 2>/dev/null || true
set -u
ros2 topic list --no-daemon > "$TMP/ros2_topic_list.txt" 2>&1 || true
ros2 node list --no-daemon > "$TMP/ros2_node_list.txt" 2>&1 || true
timeout 5 ros2 run tf2_tools view_frames > "$TMP/tf_report.txt" 2>&1 || true
ros2 doctor --report > "$TMP/ros2_doctor_report.txt" 2>&1 || true
tar -czf "$ROOT/latest_debug_package.tar.gz" -C "$TMP" .
rm -rf "$TMP"
echo "$ROOT/latest_debug_package.tar.gz"
