# 3DPlanner_FULL: Bimodal Land-Air 3D Exploration Planning Baseline

This repository contains a ROS 2 Humble baseline for bimodal land-air 3D exploration planning on Ubuntu 22.04.

Current stable point: **P2C live RViz visual exploration baseline**.

The current version runs a fallback accumulated 3D map, Air/Ground baseline exploration planners, a mode mux, a fake executor, a visual TF guard, and RViz visualization. It is a simulation and visualization baseline only; it does not publish real robot control topics.

## Directory Structure

```text
3DPlanner_FULL/
  Map/
  Air/
  Ground/
  scripts/
  docs/
  test-log/
```

## Environment

- Ubuntu 22.04
- ROS 2 Humble
- Python ROS 2 packages built with `colcon`
- Three workspaces:
  - `Map/ros2_ws`
  - `Air/ros2_ws`
  - `Ground/ros2_ws`

## Build

```bash
source /opt/ros/humble/setup.bash

cd /home/nuaa/ZHY/3DPlanner_FULL
bash scripts/setup_all_workspaces.sh
```

## Quick Start: One-Shot Live RViz Demo

Run this from an Ubuntu graphical desktop terminal:

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
bash scripts/run_live_rviz_demo_all_in_one.sh --mode-switch-period 60
```

The script starts the Map/Air/Ground visual demo, waits for TF readiness, checks visual topics, and then launches RViz.

## Three-Terminal Live RViz Demo

Terminal 1:

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
bash scripts/start_visual_demo_keepalive.sh --mode-switch-period 60
```

Terminal 2:

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
bash scripts/check_rviz_tf_ready.sh
bash scripts/check_visual_topics_ready.sh
bash scripts/visual_topic_watch.sh
```

Terminal 3:

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
bash scripts/run_rviz_visual_exploration.sh
```

## Expected RViz Topics

RViz should show:

- `/bimodal/map_3d`
- `/bimodal/points`
- `/bimodal/coverage_markers`
- `/air/candidate_markers`
- `/ground/frontier_candidates`
- `/bimodal/active_path`
- `/bimodal/executed_path`
- `/bimodal/robot_marker`

The RViz fixed frame is `map`. The visual TF guard publishes the stable `map -> odom -> base_link -> camera_link/lidar_link` tree for the visual demo.

## Current Capabilities

- Map workspace with fallback accumulated 3D map
- Air workspace baseline exploration visualization
- Ground workspace baseline frontier visualization
- Virtual sensor point cloud and robot markers
- `visual_tf_guard_node` for RViz TF stability
- Air/Ground mode mux
- Fake executor publishing `/bimodal/odom`, active/executed path visualization, and status markers
- Live RViz orchestration scripts

## Safety Scope

This baseline does not publish real control topics:

- no `/cmd_vel`
- no `/mavros/*`
- no `/fmu/*`
- no `/actuator/*`
- no `/offboard_control_mode`
- no `/trajectory_setpoint`

The fake executor is simulation-only and publishes `/bimodal/odom` plus visualization/status topics.

## Current Stage and Next Steps

Current stable point:

- `P2C live RViz visual exploration baseline`

Recommended next stage:

- `P2D frontier quality optimization`

Later integration stages:

- `P1B` OctoMap or existing `PointCloud2` map backend
- `P3A` Air FUEL wrapper
- `P4A` Ground TARE-style planner
