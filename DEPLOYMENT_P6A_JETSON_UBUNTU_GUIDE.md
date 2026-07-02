# P6A Jetson / Ubuntu Deployment Guide

## Project

`3DPlanner_FULL` is a ROS2 Humble dual-modal air/ground 3D mapping and exploration planning demo. The current deployment baseline is P5C, built on the P5B explainable realistic visual demo.

Current baseline:

- P5B explainable synthetic demo with `realistic_room_corridor_v1`
- P5C frozen PointCloud2 input interface for live camera/lidar topics and rosbag2 replay
- Shared 3D map backend: `octomap_style_voxel`
- Air planner mode: `fuel_style_v0`
- Ground planner mode: `ground_3d_frontier_v0`
- No real vehicle control output

## Clone And Checkout

```bash
git clone https://github.com/PerryFish/3DPlanner_FULL.git
cd 3DPlanner_FULL
git checkout p5b-p5c-final-baseline
```

If the stable branch is not available, use:

```bash
git checkout p5b-p5c-final-baseline-20260702
```

Or use the P5C tag:

```bash
git checkout p5c-sensor-interface-freeze-20260702
```

## Environment

Required baseline:

- Ubuntu 22.04
- ROS2 Humble
- Python 3
- `colcon`
- `rosdep`
- RViz2 for visual demo runs

Run the environment check first:

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
./scripts/p6a_check_environment.sh
```

## Jetson Notes

Jetson deployment should start with the same ROS2 Humble setup and the same scripts. Expected differences:

- `nvidia-smi` is usually unavailable on Jetson.
- `tegrastats` may be available and is preferred for live resource monitoring.
- RViz may need a valid X11 display, SSH X forwarding, or a remote desktop session.
- If CPU or memory is tight, reduce validation duration before field testing.
- Do not connect real flight-control outputs during P6A/P5C dry runs.

## Safety Limits

This baseline is for mapping, planning, visualization, and sensor-input integration only. It must not publish real control topics.

Forbidden topics:

- `/cmd_vel`
- `/mavros/*`
- `/fmu/*`
- `/actuator/*`
- `/offboard_control_mode`
- `/trajectory_setpoint`

The P6A preflight and validation scripts check these topics before declaring PASS.

## Directory Layout

- `Map`: shared map backend, PointCloud2 bridge, TF guard, demo overlays, RViz configs
- `Air`: Air P3B planner workspace
- `Ground`: Ground P4B planner workspace
- `scripts`: build, demo, validation, diagnostics, export helpers

ROS2 workspaces:

- `Map/ros2_ws`
- `Air/ros2_ws`
- `Ground/ros2_ws`

## Build

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
./scripts/p6a_build_all.sh
```

After a successful build, source in new terminals:

```bash
source /opt/ros/humble/setup.bash
source /home/nuaa/ZHY/3DPlanner_FULL/Map/ros2_ws/install/setup.bash
source /home/nuaa/ZHY/3DPlanner_FULL/Air/ros2_ws/install/setup.bash
source /home/nuaa/ZHY/3DPlanner_FULL/Ground/ros2_ws/install/setup.bash
```

## Runtime Preflight

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
./scripts/p6a_runtime_preflight.sh
```

This checks scripts, configs, RViz availability, existing ROS nodes, forbidden topics, and display limitations.

## Synthetic Fallback Demo

P5B explainable live demo:

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
./scripts/run_p5b_explainable_bimodal_live_demo.sh
```

Second terminal status check:

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
./scripts/check_p5b_live_demo_status.sh
```

P5C synthetic fallback validation:

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
P5C_ALLOW_SYNTHETIC_FALLBACK=1 P5C_DURATION_SEC=180 ./scripts/run_p5c_sensor_interface_validation.sh
```

## Unified Launcher

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
P6A_MODE=p5b_synthetic ./scripts/run_p6a_unified_demo_launcher.sh
P6A_MODE=p5c_synthetic_validation ./scripts/run_p6a_unified_demo_launcher.sh
P6A_MODE=status ./scripts/run_p6a_unified_demo_launcher.sh
P6A_MODE=rviz_only ./scripts/run_p6a_unified_demo_launcher.sh
```

## Live External PointCloud2

Examples:

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
P6A_MODE=p5c_live_topic P5C_INPUT_TOPIC=/camera/depth/points ./scripts/run_p6a_unified_demo_launcher.sh
P6A_MODE=p5c_live_topic P5C_INPUT_TOPIC=/lidar/points ./scripts/run_p6a_unified_demo_launcher.sh
```

Supported PointCloud2 topics include:

- `/camera/depth/points`
- `/realsense/depth/color/points`
- `/lidar/points`
- `/points_raw`
- `/livox/lidar`
- `/velodyne_points`
- `/ouster/points`

The live external topic demo does not start the synthetic publisher unless explicitly allowed by `P5C_ALLOW_SYNTHETIC_FALLBACK=1`.

## Rosbag2 Replay

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
P6A_MODE=p5c_bag P5C_BAG_PATH=/path/to/rosbag2_dir ./scripts/run_p6a_unified_demo_launcher.sh
```

Optional:

```bash
P5C_INPUT_TOPIC=/velodyne_points
P5C_USE_SIM_TIME=1
P5C_BAG_PLAY_RATE=1.0
P5C_LOOP_BAG=0
```

Current repository releases do not include real bag files.

## RViz Only

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
rviz2 -d /home/nuaa/ZHY/3DPlanner_FULL/Map/rviz/p5b_explainable_bimodal_demo.rviz
```

Or:

```bash
P6A_MODE=rviz_only ./scripts/run_p6a_unified_demo_launcher.sh
```

## Runtime Monitor

Run while a demo is active:

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
./scripts/p6a_runtime_monitor.sh
```

The monitor reports CPU, memory, disk, ROS node/topic counts, key topic rates, map/path availability, and forbidden control topics.

## Debug Package Export

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
./scripts/p6a_export_debug_package.sh
```

Output:

```text
/home/nuaa/ZHY/3DPlanner_FULL/latest_p6a_jetson_deployment_prep_package.tar.gz
```

The package excludes `build/`, `install/`, `log/`, rosbag/db3 files, nested tarballs, and the custom git directory.

## Full P6A Validation

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
P6A_DURATION_SEC=120 ./scripts/run_p6a_jetson_deployment_prep_validation.sh
```

## Common Issues

TF does not exist:
Run the live demo first. Validation scripts clean up their ROS nodes after completion, so `tf2_echo map base_link` will fail after cleanup.

RViz opens but data is stale:
Start the live demo and run `./scripts/check_p5b_live_demo_status.sh` in a second terminal. RViz can retain old marker visuals after nodes are stopped.

`/bimodal/points` has no data:
Check that the bridge input topic exists. For synthetic fallback, `/points_raw` should be published. For real topics, run `./scripts/preflight_p5c_pointcloud_input.sh`.

No real bag:
This is allowed for P5C/P6A. Use synthetic fallback validation until a real camera/lidar bag is available.

GitHub network failure:
Keep local commits and tags. Use `MANUAL_GITHUB_UPLOAD_P5B_P5C.md` when network access is available.

ROS_DOMAIN_ID mismatch:
Use the same `ROS_DOMAIN_ID` in all terminals. The default scripts set `ROS_DOMAIN_ID=0` unless overridden.

DISPLAY/X11 limitation:
Headless validation can still pass. RViz requires a valid display server.

Jetson performance is insufficient:
Use shorter validations first, watch `tegrastats`, and avoid running RViz locally until the headless chain is stable.
