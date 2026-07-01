# P2B 可视化探索演示计划

## 当前阶段

P2B：Visual Exploration Demo + Planning Exploration Behavior。

本阶段位于 P2A 闭环仿真通过之后，目标是把已经跑通的数据闭环升级为 RViz 中可观察、可解释的探索演示。

## 本轮做什么

- 将 P2A 数据闭环升级为 RViz 可视化探索演示。
- 增强虚拟传感器的 world ground truth、机器人位姿、传感器范围和当前姿态 marker。
- 增强 fallback accumulated map 的 coverage marker、map status marker 和 map metrics。
- 增强 Air/Ground 规划候选点、选中目标和 planner status。
- 增强 fake executor 的执行位置、执行状态和轨迹可视化。

## 本轮不代表什么

- 不代表真实 nvblox 建图性能。
- 不代表真实 FUEL 探索性能。
- 不代表真实 TARE/GBPlanner 地面探索性能。
- 不代表真机控制能力。

## 如何运行

```bash
bash scripts/run_visual_exploration_demo.sh --duration 180 --rviz
bash scripts/run_visual_exploration_demo.sh --duration 180 --no-rviz
bash scripts/run_rviz_visual_exploration.sh
```

无 DISPLAY 环境下，`run_visual_exploration_demo.sh` 会自动降级为 headless 运行，并在报告中记录 DISPLAY 限制。

## RViz 中应该看什么

- `/bimodal/world_gt_cloud`：完整虚拟环境，仅用于对比显示。
- `/bimodal/points`：当前局部感知点云。
- `/bimodal/map_3d`：累计 3D 地图。
- `/bimodal/coverage_markers`：已观测区域近似显示。
- `/bimodal/active_path`：mode mux 当前选中的路径。
- `/bimodal/executed_path`：fake executor 实际执行轨迹。
- `/air/candidate_markers`、`/ground/frontier_candidates`：规划候选点。
- `/bimodal/map_status_marker`、`/bimodal/executor_status_marker`：文本状态。

## 下一轮建议

- P2C：提升探索规划质量，减少重复目标和无效路径。
- P1B：接 existing PointCloud2 / OctoMap 真实建图输入。
- P3A：接 Air FUEL wrapper。
- P4A：接 Ground TARE-style hierarchical exploration。
