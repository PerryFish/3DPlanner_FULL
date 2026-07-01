#!/usr/bin/env bash
set -u

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
TS=$(date +%Y%m%d_%H%M%S)
LOG="${P2C_LOG_DIR:-$ROOT/test-log/${TS}_p2c_rviz_and_exploration_quality}"
mkdir -p "$LOG"

display_set=FAIL
[ -n "${DISPLAY:-}" ] && display_set=PASS
xauth_set=WARN
[ -n "${XAUTHORITY:-}" ] && xauth_set=PASS
rviz2_available=FAIL
command -v rviz2 >/dev/null 2>&1 && rviz2_available=PASS
x_server_access=FAIL
timeout 5 xdpyinfo >/dev/null 2>&1 && x_server_access=PASS
qt_xcb=WARN
if [ "$display_set" = PASS ] && [ "$x_server_access" = PASS ]; then
  qt_xcb=PASS
elif [ "$display_set" = FAIL ] || [ "$x_server_access" = FAIL ]; then
  qt_xcb=FAIL
fi
limitation=NO
if [ "$display_set" = FAIL ] || [ "$x_server_access" = FAIL ]; then
  limitation=YES
fi

cmd="cd $ROOT && bash scripts/run_rviz_visual_exploration.sh"
{
  echo "# RViz Display Environment Report"
  echo
  echo "DISPLAY_SET=$display_set"
  echo "XAUTHORITY_SET=$xauth_set"
  echo "X_SERVER_ACCESS=$x_server_access"
  echo "RVIZ2_AVAILABLE=$rviz2_available"
  echo "QT_XCB_LIKELY_OK=$qt_xcb"
  echo "DISPLAY_ENVIRONMENT_LIMITATION=$limitation"
  echo "RECOMMENDED_RVIZ_COMMAND=$cmd"
  echo
  echo "## Environment"
  echo '```'
  echo "DISPLAY=${DISPLAY:-}"
  echo "WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-}"
  echo "XAUTHORITY=${XAUTHORITY:-}"
  printenv | grep -E 'DISPLAY|QT|XDG|WAYLAND|XAUTH' || true
  echo '```'
  echo
  echo "## xdpyinfo"
  echo '```'
  timeout 5 xdpyinfo 2>&1 || true
  echo '```'
  echo
  echo "## xhost"
  echo '```'
  timeout 5 xhost 2>&1 || true
  echo '```'
  echo
  echo "## rviz2"
  echo '```'
  which rviz2 || true
  timeout 5 rviz2 --help 2>&1 | head -80 || true
  echo '```'
} > "$LOG/rviz_display_env_report.md"

cat "$LOG/rviz_display_env_report.md"
