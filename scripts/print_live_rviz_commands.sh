#!/usr/bin/env bash
set -eo pipefail

cat <<'EOF'
当前阶段：P2C-LIVE-DEMO-ORCHESTRATION-FIX

推荐方式 A：一键图形终端运行
cd /home/nuaa/ZHY/3DPlanner_FULL
bash scripts/run_live_rviz_demo_all_in_one.sh --mode-switch-period 60

推荐方式 B：三终端运行

终端1：
cd /home/nuaa/ZHY/3DPlanner_FULL
bash scripts/start_visual_demo_keepalive.sh --mode-switch-period 60

终端2：
cd /home/nuaa/ZHY/3DPlanner_FULL
bash scripts/check_rviz_tf_ready.sh
bash scripts/check_visual_topics_ready.sh
bash scripts/visual_topic_watch.sh

终端3：
cd /home/nuaa/ZHY/3DPlanner_FULL
bash scripts/run_rviz_visual_exploration.sh

说明：
- 只运行 check_rviz_tf_ready.sh 不会启动 demo。
- 如果没有先运行 start_visual_demo_keepalive.sh，/tf 不存在是正常现象。
- RViz 必须在 TF_READY=PASS 后启动。
EOF
