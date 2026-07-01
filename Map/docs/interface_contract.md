# Bimodal 3D Planning Interface Contract

| Topic | Type | Publisher | Subscriber | frame_id | Required | Current |
|---|---|---|---|---|---|---|
| /bimodal/points | sensor_msgs/msg/PointCloud2 | virtual_sensor_node | fallback_3d_map_adapter_node | map | Yes | mock virtual 3D lidar |
| /bimodal/depth/image | sensor_msgs/msg/Image | virtual_sensor_node | future mapping | camera_link | Yes | mock 32FC1 depth |
| /bimodal/camera_info | sensor_msgs/msg/CameraInfo | virtual_sensor_node | future mapping | camera_link | Yes | mock intrinsics |
| /bimodal/odom | nav_msgs/msg/Odometry | virtual_sensor_node | Air/Ground/Map | odom, child base_link | Yes | mock odom |
| /tf, /tf_static | tf2_msgs/msg/TFMessage | virtual_sensor_node | all modules/RViz | map, odom, base_link, camera_link, lidar_link | Yes | baseline TF |
| /bimodal/map_3d | sensor_msgs/msg/PointCloud2 | fallback_3d_map_adapter_node | Air/Ground | map | Yes | fallback occupancy cloud |
| /bimodal/esdf | sensor_msgs/msg/PointCloud2 | fallback_3d_map_adapter_node | Air/Ground | map | Yes | ESDF-like fallback |
| /bimodal/exploration_boundary | visualization_msgs/msg/MarkerArray | fallback_3d_map_adapter_node | Air/Ground/RViz | map | Yes | boundary cube |
| /air/exploration_goal | geometry_msgs/msg/PoseStamped | air_exploration_stub_node | bimodal_mode_mux_node | map | Yes | FUEL wrapper stub |
| /air/trajectory | nav_msgs/msg/Path | air_exploration_stub_node | bimodal_mode_mux_node | map | Yes | straight segment |
| /air/planner_status | std_msgs/msg/String | air_exploration_stub_node | bimodal_mode_mux_node | n/a | Yes | status string |
| /ground/exploration_goal | geometry_msgs/msg/PoseStamped | ground_3d_frontier_node | bimodal_mode_mux_node | map | Yes | 3D frontier proxy |
| /ground/path | nav_msgs/msg/Path | ground_3d_frontier_node | bimodal_mode_mux_node | map | Yes | straight segment |
| /ground/planner_status | std_msgs/msg/String | ground_3d_frontier_node | bimodal_mode_mux_node | n/a | Yes | status string |
| /ground/frontier_candidates | visualization_msgs/msg/MarkerArray | ground_3d_frontier_node | RViz | map | Yes | candidate markers |
| /bimodal/active_mode | std_msgs/msg/String | simple_mode_commander_node | bimodal_mode_mux_node | n/a | Test only | AIR/GROUND/IDLE |
| /bimodal/active_goal | geometry_msgs/msg/PoseStamped | bimodal_mode_mux_node | future controller/state machine | map | Test only | mux output |
| /bimodal/active_path | nav_msgs/msg/Path | bimodal_mode_mux_node | future controller/state machine | map | Test only | mux output |
| /bimodal/mux_status | std_msgs/msg/String | bimodal_mode_mux_node | diagnostics | n/a | Test only | mux status |

Replacement path: virtual camera, lidar, and odom publishers can be replaced by real sensor drivers and localization as long as they publish the same topics and frames. The fallback map adapter can be replaced by nvblox, RTAB-Map, OctoMap, or another 3D mapper if it publishes `/bimodal/map_3d` and `/bimodal/esdf`.
