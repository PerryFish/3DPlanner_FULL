# Manual GitHub Upload Guide For P5B/P5C Baseline

## Current Local Baseline

- Project: `3DPlanner_FULL`
- Repository: `https://github.com/PerryFish/3DPlanner_FULL`
- P5B accepted commit: `4882bd16fb6d74ecb7624349a003168d7ac60289`
- P5C branch pattern: `dev/p5c-sensor-interface-freeze-<timestamp>`
- P5C tag: `p5c-sensor-interface-freeze-20260702`
- Stable upload branch recommendation: `p5b-p5c-final-baseline`

After the P5C local commit is created, use:

```bash
git --git-dir=/home/nuaa/ZHY/3DPlanner_FULL/.git_3dplanner_full --work-tree=/home/nuaa/ZHY/3DPlanner_FULL rev-parse HEAD
```

to confirm the exact final P5C commit.

## Check Local Status

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
git --git-dir="$PWD/.git_3dplanner_full" --work-tree="$PWD" status --short
git --git-dir="$PWD/.git_3dplanner_full" --work-tree="$PWD" branch --show-current
git --git-dir="$PWD/.git_3dplanner_full" --work-tree="$PWD" log --oneline -5
```

The worktree should be clean before pushing.

## Large File Guard

Do not commit generated workspaces, logs, bags, database files, or debug archives.

Check tracked files:

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
git --git-dir="$PWD/.git_3dplanner_full" --work-tree="$PWD" ls-files \
  | rg '(^|/)(build|install|log|test-log)(/|$)|\\.bag$|\\.db3$|\\.tar\\.gz$|__pycache__/|\\.pytest_cache/'
```

Expected result: no output.

Ignored generated artifacts may exist locally, including:

- `Map/ros2_ws/build/`, `Map/ros2_ws/install/`, `Map/ros2_ws/log/`
- `Air/ros2_ws/build/`, `Air/ros2_ws/install/`, `Air/ros2_ws/log/`
- `Ground/ros2_ws/build/`, `Ground/ros2_ws/install/`, `Ground/ros2_ws/log/`
- `test-log/`
- `*.bag`, `*.db3`, `*.tar.gz`

## Push Current Branch

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
BRANCH=$(git --git-dir="$PWD/.git_3dplanner_full" --work-tree="$PWD" branch --show-current)
git --git-dir="$PWD/.git_3dplanner_full" --work-tree="$PWD" push -u origin "$BRANCH"
```

## Push Stable Baseline Branch

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
git --git-dir="$PWD/.git_3dplanner_full" --work-tree="$PWD" push origin HEAD:p5b-p5c-final-baseline
```

If GitHub rejects because the remote branch already exists and is not a fast-forward, do not force push. Use a dated branch instead:

```bash
git --git-dir="$PWD/.git_3dplanner_full" --work-tree="$PWD" push origin HEAD:p5b-p5c-final-baseline-20260702
```

## Push Tag

If the P5C tag already exists locally and points to the final commit:

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
git --git-dir="$PWD/.git_3dplanner_full" --work-tree="$PWD" push origin p5c-sensor-interface-freeze-20260702
```

If no tag exists yet:

```bash
git --git-dir="$PWD/.git_3dplanner_full" --work-tree="$PWD" tag -a p5c-sensor-interface-freeze-20260702 -m "P5C sensor input interface freeze baseline"
git --git-dir="$PWD/.git_3dplanner_full" --work-tree="$PWD" push origin p5c-sensor-interface-freeze-20260702
```

If the tag name conflicts remotely, create a new local tag such as `p5c-sensor-interface-freeze-20260702-v2` and push that. Do not force push tags.

## Verify Remote

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
git --git-dir="$PWD/.git_3dplanner_full" --work-tree="$PWD" ls-remote --heads origin
git --git-dir="$PWD/.git_3dplanner_full" --work-tree="$PWD" ls-remote --tags origin
```

Verify the remote has the current branch, the stable baseline branch, and the P5C tag.

## Clone And Checkout

```bash
git clone https://github.com/PerryFish/3DPlanner_FULL.git
cd 3DPlanner_FULL
git checkout p5c-sensor-interface-freeze-20260702
```

Or use the stable branch:

```bash
git checkout p5b-p5c-final-baseline
```

## Build After Clone

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
source /opt/ros/humble/setup.bash
./scripts/setup_all_workspaces.sh
```

## Run P5B Explainable Live Demo

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
./scripts/run_p5b_explainable_bimodal_live_demo.sh
```

## Run P5C Sensor Interface Validation

No real bag is required:

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
P5C_ALLOW_SYNTHETIC_FALLBACK=1 P5C_DURATION_SEC=180 ./scripts/run_p5c_sensor_interface_validation.sh
```

## Run Real Bag Replay Live Demo

This repository does not include real bag files. Keep bags outside Git.

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
P5C_BAG_PATH=/path/to/rosbag2_dir ./scripts/run_p5c_real_bag_replay_live_demo.sh
```

Optional:

```bash
P5C_INPUT_TOPIC=/velodyne_points P5C_BAG_PLAY_RATE=1.0 P5C_LOOP_BAG=0 ./scripts/run_p5c_real_bag_replay_live_demo.sh
```

## Run Live External PointCloud2 Demo

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
P5C_INPUT_TOPIC=/camera/depth/points ./scripts/run_p5c_live_external_pointcloud_demo.sh
```

Other supported topics include:

- `/realsense/depth/color/points`
- `/lidar/points`
- `/points_raw`
- `/livox/lidar`
- `/velodyne_points`
- `/ouster/points`

## Safety Notes

The P5B/P5C baseline is a mapping, planning, visualization, and fake-execution demo. It does not connect to real flight controllers or robot motor controllers.

Forbidden topics must not be published by this system:

- `/cmd_vel`
- `/mavros/*`
- `/fmu/*`
- `/actuator/*`
- `/offboard_control_mode`
- `/trajectory_setpoint`

If a future bag contains these topics, preflight may report them, but the P5C bridge must only use the selected PointCloud2 topic.
