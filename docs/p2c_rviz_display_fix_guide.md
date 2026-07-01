# P2C-RVIZ-DISPLAY-FIX 使用说明

本阶段只处理 RViz GUI 显示权限和启动流程，不修改 Map/Air/Ground 算法，不接真实建图后端，也不发布真实控制 topic。

## 当前结论

P2C 算法链路已经通过，visual topics 已经 ready。当前 RViz 无法在 Codex/headless shell 中直接显示的主要原因是：环境中虽然存在 `DISPLAY=:0`，但当前 shell 无法访问对应 X server，`xdpyinfo` 和 `xhost` 均失败。这不是 ROS topic、RViz 配置或探索算法失败。

## 推荐运行方式

终端 1：在 Ubuntu 图形桌面终端中启动持续运行的探索 demo：

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
bash scripts/start_visual_demo_keepalive.sh --mode-switch-period 60
```

终端 2：可选，检查 RViz 所需 topic 是否持续发布：

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
bash scripts/visual_topic_watch.sh
```

终端 3：在同一个图形桌面会话中启动 RViz：

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
bash scripts/run_rviz_visual_exploration.sh
```

## 如果 RViz 仍然打不开

先运行诊断建议脚本：

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
bash scripts/fix_rviz_display_env_suggestions.sh
```

常见处理方式：

```bash
xhost +SI:localuser:$(whoami)
```

如果在 SSH 中运行，需要使用：

```bash
ssh -X user@host
# 或
ssh -Y user@host
```

如果在 tmux 中运行，需要把图形桌面终端里的 `DISPLAY` 和 `XAUTHORITY` 导入 tmux 会话。

如果是 Wayland 环境，可以尝试：

```bash
export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1
unset WAYLAND_DISPLAY
```

不要使用 `xhost +` 全局开放 X server。
