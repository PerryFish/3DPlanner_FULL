# Air Interface Contract

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
| /air/exploration_goal | geometry_msgs/msg/PoseStamped | map | collision-checked sampled goal |
| /air/trajectory | nav_msgs/msg/Path | map | current pose to goal |
| /air/planner_status | std_msgs/msg/String | n/a | diagnostic contract |

This module does not publish real control topics.
