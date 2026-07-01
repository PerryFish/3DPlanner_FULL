# Air FUEL Wrapper Plan

`air_exploration_stub_node` is a FUEL-compatible ROS2 wrapper skeleton. It validates the ROS2 topic contract and the shared 3D map data loop without modifying `/home/nuaa/ZHY/FUEL_PLANNER_V3`.

Future FUEL integration should use an isolated wrapper, bridge, or adapter around FUEL_PLANNER_V3. Expected FUEL inputs are odom, 3D map or occupancy/ESDF data, and exploration boundary. Expected FUEL outputs are best viewpoint, path, trajectory, and planner status.

The current stub only samples collision-checked 3D air goals and publishes `/air/exploration_goal`, `/air/trajectory`, and `/air/planner_status`.
