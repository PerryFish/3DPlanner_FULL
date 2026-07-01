# Shared 3D Mapping Plan

The current `fallback_3d_map_adapter_node` is only a baseline adapter. It filters NaN points, optionally downsamples by voxel key, republishes `/bimodal/map_3d`, creates an ESDF-like point cloud, and publishes a 3D exploration boundary.

Next mapping upgrades:

- First choice: nvblox for GPU accelerated reconstruction and ESDF, especially for Jetson deployment.
- Fallback: RTAB-Map ROS2 for RGB-D/LiDAR mapping and loop closure where GPU ESDF is not available.
- Lightweight fallback: OctoMap for occupancy-oriented 3D exploration tests.

The Air and Ground modules must not depend on a specific mapper implementation. Any real mapper can replace the baseline when it publishes `/bimodal/map_3d`, `/bimodal/esdf`, `/bimodal/exploration_boundary`, and valid TF frames.

## P1A Backend Readiness Result

P1A freezes the shared Map contract and introduces a backend bridge path without replacing the working fallback baseline.

Frozen output topics:

- `/bimodal/map_3d`
- `/bimodal/esdf`
- `/bimodal/exploration_boundary`
- `/bimodal/map_backend_status`

Backend modes prepared in `map_backend_bridge_node`:

- `fallback`: bridge stays idle and reports that `fallback_3d_map_adapter_node` should be used.
- `external_pointcloud`: bridges externally published `PointCloud2` map and optional ESDF cloud into `/bimodal/*`.
- `nvblox_ready`: waits for configured nvblox map/ESDF topics and reports `WAITING_FOR_NVBOX` until data arrives.
- `rtabmap_ready`: waits for configured RTAB-Map cloud output and reports `WAITING_FOR_RTABMAP` until data arrives.
- `octomap_ready`: accepts a point-cloud representation such as `/octomap_point_cloud_centers`; binary OctoMap conversion is still pending.

`Map/config/map_adapter.yaml` remains the fallback adapter configuration. `Map/config/map_backend.yaml` is the bridge/selector configuration for real backends.
