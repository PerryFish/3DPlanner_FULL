# P2C RViz 与探索质量优化计划

## 当前阶段

P2C：RViz Runtime Fix + Exploration Quality Improvement。

本阶段在 P2B 可视化 topic 全部通过之后，解决两个问题：第一是把 RViz 观看流程做成可诊断、可复现；第二是让 Air/Ground baseline 探索目标更稳定、更偏向覆盖增长。

## 本轮做什么

- 增加 RViz DISPLAY / Qt xcb 环境诊断。
- 增加 keepalive demo，方便用户先启动 ROS graph，再另开图形终端启动 RViz。
- 增加 visual topic watch，确认 RViz 所需 topic 是否持续发布。
- 提升 Air/Ground 探索目标稳定性和覆盖性，引入 goal hold、blacklist、sector balance、frontier/unknown gain。
- 提升 fake executor 对 active_path 频繁更新的稳定性，过滤微小终点变化和过快切换。

## 本轮不代表什么

- 不代表真实 nvblox 建图。
- 不代表真实 FUEL 探索。
- 不代表真实 TARE/GBPlanner 地面探索。
- 不代表真机控制能力。

## 用户如何运行可视化

终端 1：

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
bash scripts/start_visual_demo_keepalive.sh --mode-switch-period 60
```

终端 2：

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
bash scripts/run_rviz_visual_exploration.sh
```

或直接运行质量实验：

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
bash scripts/run_p2c_exploration_quality_demo.sh --duration 240 --rviz --mode-switch-period 60
```

DISPLAY 不可用时：

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
bash scripts/check_display_rviz_env.sh
```

根据报告处理远程桌面、X11 forwarding 或本地图形终端问题。

## RViz 中应该观察

- `world_gt_cloud`：完整虚拟世界。
- `points`：当前局部感知。
- `map_3d`：累计建图。
- `coverage_markers`：覆盖区域。
- `air/ground candidate markers`：候选探索点。
- `active_path`：当前被 mux 选择的路径。
- `executed_path`：模拟执行轨迹。
- `robot_marker`：当前位置。
- `status markers`：模式、执行状态和覆盖率。

## 下一轮建议

- 如果 P2C PASS：进入 P1B_CONNECT_EXISTING_MAPPING_MODULE 或 P1B_CONNECT_OCTOMAP_POINTCLOUD。
- 如果 RViz DISPLAY 仍失败：单独做 P2C-RVIZ-DISPLAY-FIX。
- 如果探索重复仍明显：进入 P2D_FRONTIER_QUALITY_AND_COVERAGE_OPTIMIZATION。
