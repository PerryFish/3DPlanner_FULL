#!/usr/bin/env bash
set -u

ROOT=/home/nuaa/ZHY/3DPlanner_FULL

echo "# P2C-RVIZ-DISPLAY-FIX"
echo "# This script prints safe RViz display access diagnostics and suggestions."
echo
echo "USER=$(whoami)"
echo "HOME=$HOME"
echo "SHELL=$SHELL"
echo "DISPLAY=${DISPLAY:-}"
echo "WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-}"
echo "XAUTHORITY=${XAUTHORITY:-}"
echo "XDG_SESSION_TYPE=${XDG_SESSION_TYPE:-}"
echo

export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1

if [ -z "${DISPLAY:-}" ]; then
  echo "DISPLAY is empty. RViz must be started from an Ubuntu graphical desktop terminal or a shell with X11 forwarding."
else
  echo "DISPLAY is set to ${DISPLAY}."
fi

if command -v rviz2 >/dev/null 2>&1; then
  echo "RVIZ2_AVAILABLE=PASS ($(which rviz2))"
else
  echo "RVIZ2_AVAILABLE=FAIL"
fi

if timeout 5 xdpyinfo >/dev/null 2>&1; then
  echo "X_SERVER_ACCESS=PASS"
else
  echo "X_SERVER_ACCESS=FAIL"
  echo
  echo "Likely causes:"
  echo "- This shell is headless, sandboxed, SSH without X11 forwarding, or inside tmux with stale DISPLAY/XAUTHORITY."
  echo "- DISPLAY=${DISPLAY:-<empty>} points to a desktop session this process cannot access."
  echo "- XAUTHORITY is missing, stale, or not valid for this shell."
fi

cat <<EOF

方案 A：在 Ubuntu 图形桌面终端运行

终端1：
cd $ROOT
bash scripts/start_visual_demo_keepalive.sh --mode-switch-period 60

终端2：
cd $ROOT
bash scripts/run_rviz_visual_exploration.sh

方案 B：如果当前 shell 应该能访问本机 X server，可在图形桌面终端授权当前用户

xhost +SI:localuser:$(whoami)
cd $ROOT
bash scripts/run_rviz_visual_exploration.sh

不要运行全局开放的 xhost +。

方案 C：如果是 SSH

ssh -X user@host
# 或 ssh -Y user@host
cd $ROOT
bash scripts/run_rviz_visual_exploration.sh

方案 D：如果是 tmux

在图形桌面终端中执行：
echo \$DISPLAY
echo \$XAUTHORITY

然后在 tmux 会话中设置相同值，例如：
export DISPLAY=:0
export XAUTHORITY=\$HOME/.Xauthority
cd $ROOT
bash scripts/run_rviz_visual_exploration.sh

方案 E：如果是 Wayland/Xwayland 兼容问题

export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1
unset WAYLAND_DISPLAY
cd $ROOT
bash scripts/run_rviz_visual_exploration.sh

RViz 问题不是当前算法失败。P2C exploration quality 和 visual topics 已经通过。
EOF
