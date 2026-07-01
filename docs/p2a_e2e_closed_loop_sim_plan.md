# P2A E2E Closed-loop Simulation Plan

## 当前阶段

当前阶段是 P2A：End-to-End Bimodal Closed-loop Simulation。它用于验证“建图、规划、模态切换、模拟执行、odom 回流、再建图”的闭环数据流。

## 本轮做什么

本轮使用 fallback accumulated 3D map 验证闭环。`virtual_sensor_node` 根据当前 odom 发布局部虚拟雷达/相机数据；`fallback_3d_map_adapter_node` 将局部点云累积成 `/bimodal/map_3d` 并发布 `/bimodal/map_metrics`；`fake_path_executor_node` 订阅 `/bimodal/active_path` 并生成模拟 `/bimodal/odom`；Air/Ground baseline 输出路径；mode mux 负责双模态切换和 active path 选择。

## 本轮不代表什么

P2A 不代表真实 nvblox 建图性能，不代表真实 FUEL 探索性能，不代表真实 TARE/GBPlanner 地面探索性能，也不代表真机控制效果。fake executor 只是仿真 odom generator，不发布真实控制 topic。

## 为什么必要

在真实后端接入前先验证 topic contract 和闭环逻辑，可以提前发现 odom、TF、path、map、mode mux 的数据流问题，并为 Jetson / 真机部署前建立自动化 acceptance。

## 下一轮建议

如果 P2A PASS，可以进入 `P1B_CONNECT_EXISTING_MAPPING_MODULE` 或 `P1B_CONNECT_OCTOMAP_POINTCLOUD`。如果 P2A FAIL，应根据 failure_class 修复闭环。当前环境不建议强行安装 nvblox；如需真实建图，优先接已有 PointCloud2 建图模块。
