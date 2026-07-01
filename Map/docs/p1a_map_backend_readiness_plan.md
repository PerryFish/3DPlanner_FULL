# P1A Map Backend Readiness Plan

## 阶段位置

P1A 属于 Shared 3D Map Integration 的准备阶段。它不是最终建图算法接入，也不是 Air 或 Ground 规划算法增强。本阶段目标是冻结 Map 层统一接口，并为后续 nvblox、RTAB-Map、OctoMap 或已有建图模块接入预留清晰的 bridge/selector 结构。

## 已冻结接口

Map 层对 Air、Ground 和 mux 暴露以下稳定接口：

- `/bimodal/map_3d`：`sensor_msgs/msg/PointCloud2`，统一 3D map cloud，默认 `frame_id=map`。
- `/bimodal/esdf`：`sensor_msgs/msg/PointCloud2`，ESDF 或 ESDF-like cloud，默认 `frame_id=map`。
- `/bimodal/exploration_boundary`：`visualization_msgs/msg/MarkerArray`，探索边界。
- `/bimodal/map_backend_status`：`std_msgs/msg/String`，描述 backend_mode、map/esdf/boundary 接收状态和消息年龄。

当前 `/bimodal/esdf` 在 fallback 模式下不是真实 ESDF，只是 ESDF-like 点云占位输出。后续真实后端必须保持 topic contract 不变。

## 后续真实后端接入方式

nvblox：优先将 nvblox 的 map slice / ESDF pointcloud 映射到 `/bimodal/map_3d` 和 `/bimodal/esdf`。`map_backend_bridge_node` 已预留 `nvblox_map_cloud_topic`、`nvblox_esdf_topic`、`nvblox_mesh_topic` 参数；`backend_mode=nvblox_ready` 时如果未收到 nvblox 输出，会通过 `/bimodal/map_backend_status` 报告 `WAITING_FOR_NVBOX`。

RTAB-Map ROS2：将 `/rtabmap/cloud_map` 映射到 `/bimodal/map_3d`。RTAB-Map 默认不是 ESDF 后端，因此 `/bimodal/esdf` 需要后续额外转换或降级策略。`backend_mode=rtabmap_ready` 时未收到点云会报告 `WAITING_FOR_RTABMAP`。

OctoMap：系统如果只有 OctoMap binary map，本轮不做复杂格式转换。可先使用 `/octomap_point_cloud_centers` 作为 `/bimodal/map_3d` 输入；ESDF 仍需后续实现。`backend_mode=octomap_ready` 会报告 `OCTOMAP_POINTCLOUD_BRIDGE_PENDING`。

已有建图模块：只要能发布 `PointCloud2` 类型的 map cloud，以及可选 ESDF-like cloud 和 boundary，就可以用 `backend_mode=external_pointcloud` 接入。默认输入是 `/external/map_3d`、`/external/esdf`、`/external/exploration_boundary`。

## 为什么本轮不直接安装 nvblox

Isaac ROS / nvblox 依赖较重，Jetson Docker 环境、NVIDIA container runtime、CUDA/JetPack 版本都需要单独准备。当前先冻结 topic contract 和 adapter 结构，可以降低后续真实后端集成时对 Air/Ground/Mux 的影响。

## 配置说明

- `Map/config/map_adapter.yaml`：仅用于 fallback adapter。
- `Map/config/map_backend.yaml`：用于真实后端 bridge / selector 预配置。
- `map_fallback_baseline.launch.py`：当前稳定 fallback baseline。
- `map_backend_bridge.launch.py`：外部真实建图模块已经运行时使用，不启动 virtual sensor 或 fallback adapter。
- `map_p1a_readiness.launch.py`：P1A readiness 验证，启动 virtual sensor 和 fallback adapter。

## 下一轮 P1B 建议

如果在 Jetson 上部署，优先准备 Isaac ROS nvblox Docker，并用 `backend_mode=nvblox_ready` 做桥接验证。如果在当前 Ubuntu x86_64 上，先确认 NVIDIA GPU 驱动、Docker 和 nvidia-container-runtime，再决定是否本机运行 nvblox。如果 nvblox 暂时不可用，则优先接 RTAB-Map ROS2 或项目已有 3D 建图模块。
