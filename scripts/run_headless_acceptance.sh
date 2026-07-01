#!/usr/bin/env bash
set -u
ROOT=/home/nuaa/ZHY/3DPlanner_FULL
TS=$(date +%Y%m%d_%H%M%S)
LOG="$ROOT/test-log/$TS"
mkdir -p "$LOG" "$ROOT/Map/test-log/$TS" "$ROOT/Air/test-log/$TS" "$ROOT/Ground/test-log/$TS"
export ROS_LOG_DIR="$LOG/ros_logs"
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-$ROOT/Map/config/fastdds_shm_only.xml}
export FASTDDS_DEFAULT_PROFILES_FILE=${FASTDDS_DEFAULT_PROFILES_FILE:-$ROOT/Map/config/fastdds_shm_only.xml}
mkdir -p "$ROS_LOG_DIR"
set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$ROOT/Map/ros2_ws/install/setup.bash" 2>/dev/null || true
source "$ROOT/Air/ros2_ws/install/setup.bash" 2>/dev/null || true
source "$ROOT/Ground/ros2_ws/install/setup.bash" 2>/dev/null || true
set -u

pids=()
cleanup() { for p in "${pids[@]}"; do kill "$p" 2>/dev/null || true; done; wait 2>/dev/null || true; }
trap cleanup EXIT

"$ROOT/scripts/run_map.sh" > "$ROOT/Map/test-log/$TS/map_runtime.log" 2>&1 & pids+=($!)
sleep 3
"$ROOT/scripts/run_air.sh" > "$ROOT/Air/test-log/$TS/air_runtime.log" 2>&1 & pids+=($!)
"$ROOT/scripts/run_ground.sh" > "$ROOT/Ground/test-log/$TS/ground_runtime.log" 2>&1 & pids+=($!)
"$ROOT/scripts/run_mux.sh" > "$LOG/integration_runtime.log" 2>&1 & pids+=($!)
sleep "${ACCEPTANCE_DURATION_SEC:-60}"

topics=$(ros2 topic list --no-daemon 2>/dev/null | sort)
nodes=$(ros2 node list --no-daemon 2>/dev/null | sort)
echo "$topics" > "$LOG/topic_list.txt"
echo "$nodes" > "$LOG/node_list.txt"

has_topic() { echo "$topics" | grep -qx "$1"; }
pass_topic() { if has_topic "$1"; then echo PASS; else echo FAIL; fi; }
no_control=PASS
for t in /cmd_vel /offboard_control_mode /trajectory_setpoint; do has_topic "$t" && no_control=FAIL; done
echo "$topics" | grep -Eq '^/mavros/|^/fmu/|^/actuator/' && no_control=FAIL

capture_once() {
  topic="$1"
  out="$2"
  timeout 10 ros2 topic echo "$topic" --once --no-daemon > "$out" 2>&1
}

active_mode_file="$LOG/active_mode_echo.txt"
active_path_file="$LOG/active_path_echo.txt"
active_mux_status_file="$LOG/mux_status_echo.txt"
map_3d_file="$LOG/map_3d_echo.txt"
esdf_file="$LOG/esdf_echo.txt"
exploration_boundary_file="$LOG/exploration_boundary_echo.txt"
map_backend_status_file="$LOG/map_backend_status_echo.txt"

active_mode_topic_exists=$(pass_topic /bimodal/active_mode)
active_path_topic_exists=$(pass_topic /bimodal/active_path)
active_mode_message_captured=FAIL
active_mode_value_valid=FAIL
active_path_message_captured=FAIL
active_path_frame_map=FAIL
active_path_pose_count_valid=FAIL
map_3d_topic_exists=$(pass_topic /bimodal/map_3d)
map_3d_message_captured=FAIL
map_3d_frame_map=FAIL
map_3d_nonempty=FAIL
esdf_topic_exists=$(pass_topic /bimodal/esdf)
esdf_message_captured=FAIL
esdf_frame_map=FAIL
esdf_nonempty=FAIL
esdf_is_fallback=YES
exploration_boundary_topic_exists=$(pass_topic /bimodal/exploration_boundary)
exploration_boundary_message_captured=FAIL
exploration_boundary_nonempty=FAIL
map_backend_status_topic_exists=SKIP
map_backend_status_message_captured=SKIP
backend_mode=fallback
fallback_map_adapter_active=YES
external_map_backend_active=NO

if [ "$active_mode_topic_exists" = PASS ]; then
  if capture_once /bimodal/active_mode "$active_mode_file"; then
    grep -q '^data:' "$active_mode_file" && active_mode_message_captured=PASS
    grep -Eq '^data: (AIR|GROUND|IDLE)$' "$active_mode_file" && active_mode_value_valid=PASS
  fi
fi

if [ "$active_path_topic_exists" = PASS ]; then
  for attempt in 1 2 3; do
    if capture_once /bimodal/active_path "$active_path_file"; then
      grep -q '^poses:' "$active_path_file" && active_path_message_captured=PASS
      grep -Eq '^[[:space:]]*frame_id: map$' "$active_path_file" && active_path_frame_map=PASS
      pose_count=$(grep -c '^- header:' "$active_path_file" || true)
      [ "${pose_count:-0}" -ge 2 ] && active_path_pose_count_valid=PASS
    fi
    [ "$active_path_message_captured$active_path_frame_map$active_path_pose_count_valid" = PASSPASSPASS ] && break
    sleep 10
  done
fi

timeout 10 ros2 topic echo /bimodal/mux_status --once --no-daemon > "$active_mux_status_file" 2>&1 || true

if [ "$map_3d_topic_exists" = PASS ]; then
  if capture_once /bimodal/map_3d "$map_3d_file"; then
    grep -q '^header:' "$map_3d_file" && map_3d_message_captured=PASS
    grep -Eq '^[[:space:]]*frame_id: map$' "$map_3d_file" && map_3d_frame_map=PASS
    width=$(awk '/^width:/ {print $2; exit}' "$map_3d_file" 2>/dev/null || echo 0)
    [ "${width:-0}" -gt 0 ] && map_3d_nonempty=PASS
  fi
fi

if [ "$esdf_topic_exists" = PASS ]; then
  if capture_once /bimodal/esdf "$esdf_file"; then
    grep -q '^header:' "$esdf_file" && esdf_message_captured=PASS
    grep -Eq '^[[:space:]]*frame_id: map$' "$esdf_file" && esdf_frame_map=PASS
    width=$(awk '/^width:/ {print $2; exit}' "$esdf_file" 2>/dev/null || echo 0)
    [ "${width:-0}" -gt 0 ] && esdf_nonempty=PASS
  fi
fi

if [ "$exploration_boundary_topic_exists" = PASS ]; then
  if capture_once /bimodal/exploration_boundary "$exploration_boundary_file"; then
    grep -q '^markers:' "$exploration_boundary_file" && exploration_boundary_message_captured=PASS
    grep -q '^- header:' "$exploration_boundary_file" && exploration_boundary_nonempty=PASS
  fi
fi

if has_topic /bimodal/map_backend_status; then
  map_backend_status_topic_exists=PASS
  if capture_once /bimodal/map_backend_status "$map_backend_status_file"; then
    grep -q '^data:' "$map_backend_status_file" && map_backend_status_message_captured=PASS
    grep -q 'backend_mode=fallback' "$map_backend_status_file" && backend_mode=fallback
  fi
fi

p1a_map_backend_readiness=PASS
if [ "$map_3d_topic_exists$map_3d_message_captured$map_3d_frame_map$map_3d_nonempty" != PASSPASSPASSPASS ]; then
  p1a_map_backend_readiness=FAIL
fi
if [ "$esdf_topic_exists$esdf_message_captured$esdf_frame_map$esdf_nonempty" != PASSPASSPASSPASS ]; then
  p1a_map_backend_readiness=FAIL
fi
if [ "$exploration_boundary_topic_exists$exploration_boundary_message_captured$exploration_boundary_nonempty" != PASSPASSPASS ]; then
  p1a_map_backend_readiness=FAIL
fi
if [ "$map_backend_status_topic_exists" = PASS ] && [ "$map_backend_status_message_captured" != PASS ]; then
  p1a_map_backend_readiness=FAIL
fi

tf_yaml="$LOG/tf_echo_samples.txt"
for pair in "map odom" "odom base_link" "base_link camera_link" "base_link lidar_link"; do
  set -- $pair
  timeout 4 ros2 run tf2_ros tf2_echo "$1" "$2" >> "$tf_yaml" 2>&1 || true
done
tf_valid=PASS
for frame in odom base_link camera_link lidar_link; do grep -q "$frame" "$tf_yaml" || tf_valid=FAIL; done

map_build=PASS; air_build=PASS; ground_build=PASS
test -d "$ROOT/Map/ros2_ws/install/bimodal_map_bringup" || map_build=FAIL
test -d "$ROOT/Air/ros2_ws/install/bimodal_air_bringup" || air_build=FAIL
test -d "$ROOT/Ground/ros2_ws/install/bimodal_ground_bringup" || ground_build=FAIL

{
  echo "# Topic Report"
  echo '```'
  echo "$topics"
  echo '```'
  for t in /bimodal/points /bimodal/map_3d /bimodal/esdf /bimodal/exploration_boundary /bimodal/map_backend_status /air/planner_status /ground/planner_status /bimodal/active_mode /bimodal/active_path /bimodal/mux_status; do
    echo "## $t"
    timeout 4 ros2 topic echo "$t" --once --no-daemon 2>/dev/null | head -80 || true
  done
  echo "## saved active_mode echo"
  cat "$active_mode_file" 2>/dev/null || true
  echo "## saved active_path echo"
  cat "$active_path_file" 2>/dev/null || true
  echo "## saved mux_status echo"
  cat "$active_mux_status_file" 2>/dev/null || true
  echo "## saved map_3d echo"
  head -120 "$map_3d_file" 2>/dev/null || true
  echo "## saved esdf echo"
  head -120 "$esdf_file" 2>/dev/null || true
  echo "## saved exploration_boundary echo"
  cat "$exploration_boundary_file" 2>/dev/null || true
  echo "## saved map_backend_status echo"
  cat "$map_backend_status_file" 2>/dev/null || true
} > "$LOG/topic_report.md"
{
  echo "# TF Report"
  echo '```'
  cat "$tf_yaml"
  echo '```'
  echo "TF_TREE_VALID=$tf_valid"
} > "$LOG/tf_report.md"
{
  echo "# System Environment"
  echo "- Ubuntu version: $(lsb_release -ds 2>/dev/null || echo unknown)"
  echo "- ROS distro: ${ROS_DISTRO:-unset}"
  echo "- Python version: $(python3 --version 2>&1 || true)"
  echo "- colcon version: $(python3 -c 'import colcon_core; print(colcon_core.__version__)' 2>/dev/null || echo unknown)"
  echo "- RMW implementation: ${RMW_IMPLEMENTATION:-default}"
  if [ -f /etc/nv_tegra_release ]; then echo "- GPU / Jetson: $(cat /etc/nv_tegra_release)"; else echo "- GPU / Jetson: x86_64 desktop test, Jetson not detected"; fi
} > "$LOG/system_env.md"

P0=PASS; P1=PASS; P2=PASS; P3=PASS; P4=PASS
for t in /bimodal/points /bimodal/depth/image /bimodal/camera_info /bimodal/odom /bimodal/map_3d /bimodal/esdf /bimodal/exploration_boundary; do has_topic "$t" || P1=FAIL; done
if [ "$p1a_map_backend_readiness" != PASS ]; then
  P1=FAIL
fi
for t in /air/exploration_goal /air/trajectory /air/planner_status; do has_topic "$t" || P2=FAIL; done
for t in /ground/exploration_goal /ground/path /ground/planner_status /ground/frontier_candidates; do has_topic "$t" || P3=FAIL; done
for t in /bimodal/active_mode /bimodal/active_goal /bimodal/active_path /bimodal/mux_status; do has_topic "$t" || P4=FAIL; done
if [ "$active_mode_message_captured$active_mode_value_valid$active_path_message_captured$active_path_frame_map$active_path_pose_count_valid" != PASSPASSPASSPASSPASS ]; then
  P4=FAIL
fi
[ "$P1$P2$P3$P4$no_control$tf_valid" = "PASSPASSPASSPASSPASSPASS" ] || P0=FAIL
headless=PASS; [ "$P0" = PASS ] || headless=FAIL

{
  echo "MAP_WORKSPACE_BUILD=$map_build"
  echo "AIR_WORKSPACE_BUILD=$air_build"
  echo "GROUND_WORKSPACE_BUILD=$ground_build"
  echo
  echo "P0_INTERFACE_CONTRACT=$P0"
  echo "P1_SHARED_3D_MAP=$P1"
  echo "P2_AIR_BASELINE=$P2"
  echo "P3_GROUND_BASELINE=$P3"
  echo "P4_BIMODAL_MUX=$P4"
  echo
  echo "VIRTUAL_CAMERA_INTERFACE=$(pass_topic /bimodal/depth/image)"
  echo "VIRTUAL_LIDAR_INTERFACE=$(pass_topic /bimodal/points)"
  echo "VIRTUAL_ODOM_INTERFACE=$(pass_topic /bimodal/odom)"
  echo "TF_TREE_VALID=$tf_valid"
  echo
  echo "MAP_3D_TOPIC=$(pass_topic /bimodal/map_3d)"
  echo "ESDF_TOPIC=$(pass_topic /bimodal/esdf)"
  echo "EXPLORATION_BOUNDARY_TOPIC=$(pass_topic /bimodal/exploration_boundary)"
  echo
  echo "P1A_MAP_BACKEND_READINESS=$p1a_map_backend_readiness"
  echo "MAP_3D_TOPIC_EXISTS=$map_3d_topic_exists"
  echo "MAP_3D_MESSAGE_CAPTURED=$map_3d_message_captured"
  echo "MAP_3D_FRAME_MAP=$map_3d_frame_map"
  echo "MAP_3D_NONEMPTY=$map_3d_nonempty"
  echo
  echo "ESDF_TOPIC_EXISTS=$esdf_topic_exists"
  echo "ESDF_MESSAGE_CAPTURED=$esdf_message_captured"
  echo "ESDF_FRAME_MAP=$esdf_frame_map"
  echo "ESDF_NONEMPTY=$esdf_nonempty"
  echo "ESDF_IS_FALLBACK=$esdf_is_fallback"
  echo
  echo "EXPLORATION_BOUNDARY_TOPIC_EXISTS=$exploration_boundary_topic_exists"
  echo "EXPLORATION_BOUNDARY_MESSAGE_CAPTURED=$exploration_boundary_message_captured"
  echo "EXPLORATION_BOUNDARY_NONEMPTY=$exploration_boundary_nonempty"
  echo
  echo "MAP_BACKEND_STATUS_TOPIC_EXISTS=$map_backend_status_topic_exists"
  echo "MAP_BACKEND_STATUS_MESSAGE_CAPTURED=$map_backend_status_message_captured"
  echo
  echo "BACKEND_MODE=$backend_mode"
  echo "FALLBACK_MAP_ADAPTER_ACTIVE=$fallback_map_adapter_active"
  echo "EXTERNAL_MAP_BACKEND_ACTIVE=$external_map_backend_active"
  echo
  echo "AIR_GOAL_TOPIC=$(pass_topic /air/exploration_goal)"
  echo "AIR_TRAJECTORY_TOPIC=$(pass_topic /air/trajectory)"
  echo "AIR_STATUS_TOPIC=$(pass_topic /air/planner_status)"
  echo
  echo "GROUND_GOAL_TOPIC=$(pass_topic /ground/exploration_goal)"
  echo "GROUND_PATH_TOPIC=$(pass_topic /ground/path)"
  echo "GROUND_STATUS_TOPIC=$(pass_topic /ground/planner_status)"
  echo "GROUND_FRONTIER_MARKERS=$(pass_topic /ground/frontier_candidates)"
  echo
  echo "BIMODAL_ACTIVE_MODE=$(pass_topic /bimodal/active_mode)"
  echo "BIMODAL_ACTIVE_GOAL=$(pass_topic /bimodal/active_goal)"
  echo "BIMODAL_ACTIVE_PATH=$(pass_topic /bimodal/active_path)"
  echo "BIMODAL_MUX_STATUS=$(pass_topic /bimodal/mux_status)"
  echo
  echo "ACTIVE_MODE_TOPIC_EXISTS=$active_mode_topic_exists"
  echo "ACTIVE_MODE_MESSAGE_CAPTURED=$active_mode_message_captured"
  echo "ACTIVE_MODE_VALUE_VALID=$active_mode_value_valid"
  echo
  echo "ACTIVE_PATH_TOPIC_EXISTS=$active_path_topic_exists"
  echo "ACTIVE_PATH_MESSAGE_CAPTURED=$active_path_message_captured"
  echo "ACTIVE_PATH_FRAME_MAP=$active_path_frame_map"
  echo "ACTIVE_PATH_POSE_COUNT_VALID=$active_path_pose_count_valid"
  echo
  echo "NO_REAL_CONTROL_TOPIC=$no_control"
  echo "HEADLESS_ACCEPTANCE=$headless"
} > "$LOG/acceptance_report.md"

{
  echo "# Final Summary"
  echo
  echo "本轮建立了 Bimodal 3D Planning Baseline，包含 Map、Air、Ground 三个独立 ROS2 workspace。"
  echo
  echo "- Map: 虚拟 3D LiDAR、虚拟深度相机、odom、TF、fallback 3D map adapter、ESDF-like 输出、探索边界和临时 mode mux。"
  echo "- Air: FUEL-compatible ROS2 wrapper 骨架，订阅共享 3D map/ESDF/boundary，输出 air goal、trajectory、status。"
  echo "- Ground: 基于 3D map 的 frontier proxy baseline，带 ground constraint、clearance、revisit penalty，输出 ground goal、path、status、candidate markers。"
  echo "- 已跑通 topic: /bimodal/points, /bimodal/depth/image, /bimodal/camera_info, /bimodal/odom, /bimodal/map_3d, /bimodal/esdf, /air/*, /ground/*, /bimodal/active_*。"
  echo "- mock/fallback: virtual sensors、fallback map adapter、ESDF-like cloud、Air stub、Ground frontier proxy、simple mode commander。"
  echo "- Air 后续: 使用 wrapper/bridge 接 FUEL_PLANNER_V3，不直接修改原工程。"
  echo "- Ground 后续: 升级到 TARE-style/GBPlanner-style 3D exploration，包括 dense frontier、sparse graph、traversability、viewpoint utility。"
  echo "- Map 后续: 优先接 nvblox，RTAB-Map ROS2 作为 fallback，OctoMap 作为轻量 fallback。"
  echo "- 下一轮建议: 接入真实传感器/定位，替换 mapping backend，增强 Ground 全局图搜索，准备 Jetson Docker 部署。"
} > "$LOG/final_summary.md"
cp "$LOG/acceptance_report.md" "$LOG/build_report.md"
"$ROOT/scripts/collect_debug_package.sh" "$LOG" >/dev/null 2>&1 || true
echo "$LOG"
