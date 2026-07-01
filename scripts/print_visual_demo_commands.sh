#!/usr/bin/env bash
cat <<'EOF'
# 当前阶段：P2C-RVIZ-DISPLAY-FIX
# 目标：打开 RViz 观察双模态探索可视化

# 终端1：启动持续运行的探索 demo
cd /home/nuaa/ZHY/3DPlanner_FULL
bash scripts/start_visual_demo_keepalive.sh --mode-switch-period 60

# 终端2：检查可视化 topic
cd /home/nuaa/ZHY/3DPlanner_FULL
bash scripts/visual_topic_watch.sh

# 终端3：启动 RViz
cd /home/nuaa/ZHY/3DPlanner_FULL
bash scripts/run_rviz_visual_exploration.sh

# 如果 RViz 仍提示 X server 无权限：
cd /home/nuaa/ZHY/3DPlanner_FULL
bash scripts/fix_rviz_display_env_suggestions.sh

说明：
- 终端3 必须是 Ubuntu 图形桌面终端，或者已经正确配置 X11 forwarding 的终端。
- Codex/headless/tmux 中 DISPLAY=:0 也可能无法访问 X server。
- RViz 问题不是算法失败；P2C visual topics 和 exploration quality 已通过。
EOF
