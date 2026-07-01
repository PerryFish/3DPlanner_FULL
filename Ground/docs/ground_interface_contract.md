# Ground Interface Contract

Inputs:

| Topic | Type | frame_id | Required |
|---|---|---|---|
| /bimodal/odom | nav_msgs/msg/Odometry | odom/base_link | Yes |
| /bimodal/map_3d | sensor_msgs/msg/PointCloud2 | map | Yes |
| /bimodal/esdf | sensor_msgs/msg/PointCloud2 | map | Yes |
| /bimodal/exploration_boundary | visualization_msgs/msg/MarkerArray | map | Yes |

Outputs:

| Topic | Type | frame_id | Current implementation |
|---|---|---|---|
| /ground/exploration_goal | geometry_msgs/msg/PoseStamped | map | best 3D-map candidate with ground constraint |
| /ground/path | nav_msgs/msg/Path | map | straight-line baseline path |
| /ground/planner_status | std_msgs/msg/String | n/a | diagnostic contract |
| /ground/frontier_candidates | visualization_msgs/msg/MarkerArray | map | selected/rejected/risk candidate markers |

This module does not publish `/cmd_vel` or any real control topic.
