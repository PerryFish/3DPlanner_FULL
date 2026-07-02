#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
TS=$(date +%Y%m%d_%H%M%S)
LOG="${P5C_LOG_DIR:-$ROOT/test-log/${TS}_p5c_sensor_interface_freeze}"
BAG_PATH=${P5C_BAG_PATH:-}
INPUT_TOPIC=${P5C_INPUT_TOPIC:-}
mkdir -p "$LOG" "$LOG/ros_cli_logs"
printf '%s\n' "$LOG" > "$ROOT/test-log/.latest_p5c_sensor_interface_freeze_dir"
export ROS_LOG_DIR="$LOG/ros_cli_logs"
REPORT="$LOG/p5c_pointcloud_input_preflight_report.md"

set +u
source "$ROOT/scripts/env_visual_demo.sh" > "$LOG/env_visual_demo_preflight.txt" 2>&1 || true
set -u

forbidden_re='^/cmd_vel$|^/mavros/|^/fmu/|^/actuator/|^/offboard_control_mode$|^/trajectory_setpoint$'

write_no_input_report() {
  cat > "$REPORT" <<EOF
# P5C PointCloud2 Input Preflight

mode=no_input
real_bag_found=NO
selected_bag_path=NONE
selected_bag_pointcloud_topic=NONE
live_input_topic_found=NO
selected_live_input_topic=NONE
synthetic_fallback_available=YES
preflight_result=PASS
preflight_failure_reason=No real bag or live topic was provided; P5C will use synthetic fallback when allowed.
EOF
  cat "$REPORT"
}

if [ -z "$BAG_PATH" ] && [ -z "$INPUT_TOPIC" ]; then
  write_no_input_report
  exit 0
fi

if [ -n "$BAG_PATH" ]; then
  bag_exists=NO
  [ -d "$BAG_PATH" ] && bag_exists=YES
  {
    echo "# P5C PointCloud2 Input Preflight"
    echo
    echo "mode=bag"
    echo "selected_bag_path=$BAG_PATH"
    echo "real_bag_found=$bag_exists"
  } > "$REPORT"
  if [ "$bag_exists" != YES ]; then
    {
      echo "selected_bag_pointcloud_topic=NONE"
      echo "synthetic_fallback_available=YES"
      echo "preflight_result=PASS_PARTIAL"
      echo "preflight_failure_reason=Bag path does not exist; no real replay was started."
    } >> "$REPORT"
    cat "$REPORT"
    exit 0
  fi
  timeout 20 ros2 bag info "$BAG_PATH" > "$LOG/p5c_ros2_bag_info.txt" 2>&1 || true
  python3 - "$LOG/p5c_ros2_bag_info.txt" "$REPORT" <<'PY'
import re
import sys
from pathlib import Path

info = Path(sys.argv[1]).read_text(errors='ignore')
report = Path(sys.argv[2])
priority = [
    '/points_raw',
    '/lidar/points',
    '/velodyne_points',
    '/livox/lidar',
    '/ouster/points',
    '/camera/depth/points',
    '/realsense/depth/color/points',
]
topics = []
for line in info.splitlines():
    m = re.search(r'Topic:\s*([^| ]+).*Type:\s*([^| ]+)', line)
    if m:
        topics.append((m.group(1), m.group(2)))
pc2 = [t for t, typ in topics if typ == 'sensor_msgs/msg/PointCloud2']
selected = 'NONE'
for candidate in priority:
    if candidate in pc2:
        selected = candidate
        break
if selected == 'NONE' and pc2:
    selected = pc2[0]
tf_present = any(t == '/tf' for t, _ in topics)
tf_static_present = any(t == '/tf_static' for t, _ in topics)
odom_present = any(t in ('/odom', '/bimodal/odom') for t, _ in topics)
forbidden = [t for t, _ in topics if re.match(r'^/cmd_vel$|^/mavros/|^/fmu/|^/actuator/|^/offboard_control_mode$|^/trajectory_setpoint$', t)]
with report.open('a', encoding='utf-8') as f:
    f.write(f'pointcloud_topic_count={len(pc2)}\n')
    f.write(f'pointcloud_topic_list={",".join(pc2) if pc2 else "NONE"}\n')
    f.write(f'selected_bag_pointcloud_topic={selected}\n')
    f.write(f'bag_tf_topic_present={"YES" if tf_present else "NO"}\n')
    f.write(f'bag_tf_static_topic_present={"YES" if tf_static_present else "NO"}\n')
    f.write(f'bag_odom_topic_present={"YES" if odom_present else "NO"}\n')
    f.write(f'bag_forbidden_topic_detected_count={len(forbidden)}\n')
    f.write(f'bag_forbidden_topic_list={",".join(forbidden) if forbidden else "NONE"}\n')
    f.write('synthetic_fallback_available=YES\n')
    f.write(f'preflight_result={"PASS" if selected != "NONE" else "PASS_PARTIAL"}\n')
    f.write(f'preflight_failure_reason={"NONE" if selected != "NONE" else "No sensor_msgs/msg/PointCloud2 topic found in bag."}\n')
PY
  cat "$REPORT"
  exit 0
fi

if [ -n "$INPUT_TOPIC" ]; then
  TOPICS=$(timeout 6 ros2 topic list --no-daemon 2>/dev/null || true)
  topic_found=NO
  echo "$TOPICS" | grep -qx "$INPUT_TOPIC" && topic_found=YES
  topic_type=$(timeout 4 ros2 topic type "$INPUT_TOPIC" --no-daemon 2>/dev/null | head -1 || true)
  type_ok=NO
  [ "$topic_type" = "sensor_msgs/msg/PointCloud2" ] && type_ok=YES
  frame_id=UNKNOWN
  if [ "$type_ok" = YES ]; then
    timeout 8 ros2 topic echo "$INPUT_TOPIC" --once --no-daemon --field header > "$LOG/p5c_live_input_header.txt" 2>&1 || true
    frame_id=$(awk '/frame_id:/ {print $2; exit}' "$LOG/p5c_live_input_header.txt" 2>/dev/null || echo UNKNOWN)
    timeout 10 ros2 topic hz "$INPUT_TOPIC" --window 8 > "$LOG/p5c_live_input_rate.txt" 2>&1 || true
  fi
  rate=$(awk '/average rate:/ {print $3; exit}' "$LOG/p5c_live_input_rate.txt" 2>/dev/null || echo 0.0)
  tf_present=NO
  tf_static_present=NO
  echo "$TOPICS" | grep -qx /tf && tf_present=YES
  echo "$TOPICS" | grep -qx /tf_static && tf_static_present=YES
  forbidden=$(echo "$TOPICS" | grep -E "$forbidden_re" || true)
  forbidden_count=0
  [ -n "$forbidden" ] && forbidden_count=$(printf '%s\n' "$forbidden" | sed '/^$/d' | wc -l)
  {
    echo "# P5C PointCloud2 Input Preflight"
    echo
    echo "mode=live_topic"
    echo "real_bag_found=NO"
    echo "selected_bag_path=NONE"
    echo "selected_bag_pointcloud_topic=NONE"
    echo "selected_live_input_topic=$INPUT_TOPIC"
    echo "live_input_topic_found=$topic_found"
    echo "live_input_topic_type=${topic_type:-UNKNOWN}"
    echo "live_input_type_is_pointcloud2=$type_ok"
    echo "live_input_frame_id=$frame_id"
    echo "live_input_rate_avg_hz=${rate:-0.0}"
    echo "live_tf_topic_present=$tf_present"
    echo "live_tf_static_topic_present=$tf_static_present"
    echo "live_forbidden_topic_detected_count=$forbidden_count"
    echo "live_forbidden_topic_list=${forbidden:-NONE}"
    echo "synthetic_fallback_available=YES"
    if [ "$topic_found" = YES ] && [ "$type_ok" = YES ]; then
      echo "preflight_result=PASS"
      echo "preflight_failure_reason=NONE"
    else
      echo "preflight_result=PASS_PARTIAL"
      echo "preflight_failure_reason=Live topic missing or not sensor_msgs/msg/PointCloud2."
    fi
  } > "$REPORT"
  cat "$REPORT"
fi
