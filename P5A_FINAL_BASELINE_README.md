# 3DPlanner_FULL P5A Final Baseline

## Project

`3DPlanner_FULL` is a ROS2 Humble bimodal air-ground 3D mapping and exploration planning baseline.

## Baseline

- Baseline stage: `P5A_FULL_BIMODAL_EXPLORATION_DEMO_AND_ACCEPTANCE_GATE`
- Baseline commit before this documentation commit: `1e13b38cebc905b218af47487310e8bfc569c767`
- Default chain: external PointCloud2 input, shared 3D voxel map, Air P3B, Ground P4B, mode mux, fake executor, and RViz visualization.

## Capability Boundary

This baseline is for simulation, replay, visualization, and integration validation. It does not connect to real flight controllers or real robot motor controllers.

The system is configured to avoid real control topics. The accepted P5A validation reported:

- `no_real_control_topic=PASS`
- `final_acceptance_gate=PASS`

Forbidden real-control topics:

- `/cmd_vel`
- `/mavros/*`
- `/fmu/*`
- `/actuator/*`
- `/offboard_control_mode`
- `/trajectory_setpoint`

## Environment

- Ubuntu 22.04
- ROS2 Humble

Source ROS and the module workspaces before manual runs:

```bash
source /opt/ros/humble/setup.bash
source /home/nuaa/ZHY/3DPlanner_FULL/Map/ros2_ws/install/setup.bash
source /home/nuaa/ZHY/3DPlanner_FULL/Air/ros2_ws/install/setup.bash
source /home/nuaa/ZHY/3DPlanner_FULL/Ground/ros2_ws/install/setup.bash
```

## Final Demo Commands

Live demo:

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
./scripts/run_p5a_full_bimodal_live_demo.sh
```

Headless acceptance validation:

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
P5A_DURATION_SEC=300 ./scripts/run_p5a_full_bimodal_acceptance_validation.sh
```

RViz only:

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
rviz2 -d /home/nuaa/ZHY/3DPlanner_FULL/Map/rviz/p5a_full_bimodal_demo.rviz
```

## Key Topics

- `/bimodal/points`
- `/bimodal/map_3d`
- `/bimodal/map_metrics`
- `/air/candidate_markers`
- `/ground/frontier_candidates`
- `/bimodal/active_mode`
- `/bimodal/active_path`
- `/bimodal/executed_path`
- `/bimodal/robot_marker`

## P5A Acceptance Summary

- `duration_sec=300`
- `external_pointcloud_chain=PASS`
- `shared_3d_map_chain=PASS`
- `air_p3b_final_demo=PASS`
- `ground_p4b_final_demo=PASS`
- `full_bimodal_planner_chain=PASS`
- `final_rviz_demo_ready=PASS`
- `no_real_control_topic=PASS`
- `final_acceptance_gate=PASS`

P5A debug package from the accepted local run:

```text
/home/nuaa/ZHY/3DPlanner_FULL/latest_p5a_full_bimodal_demo_package.tar.gz
```

The package is intentionally ignored by Git because it is a generated artifact.

## Clone Reproduction

```bash
git clone https://github.com/PerryFish/3DPlanner_FULL.git
cd 3DPlanner_FULL
git checkout p5a-final-baseline-20260702
```

Build the existing module workspaces using the repository scripts or the same ROS2 Humble setup used for P5A. Then run either the live demo or the headless validation command above.

If the stable branch is preferred:

```bash
git checkout p5a-final-baseline
```
