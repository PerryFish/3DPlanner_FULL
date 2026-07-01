# GitHub Backup Report

## Project Stage

Current stage: P2C-STABLE-GITHUB-BACKUP.

The P2C live RViz visual demo has been run successfully. The current stable version is intended as a fallback 3D map + Air/Ground baseline planner + fake executor + RViz visualization baseline.

## Core Run Commands

One-shot live RViz run:

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
bash scripts/run_live_rviz_demo_all_in_one.sh --mode-switch-period 60
```

Three-terminal run:

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

## Main Capabilities

- Map workspace
- Air workspace
- Ground workspace
- Virtual sensor
- Accumulated fallback 3D map
- `visual_tf_guard_node`
- Air/Ground baseline exploration
- Mode mux
- Fake executor
- RViz visual demo

## Not Completed Yet

- nvblox integration
- RTAB-Map integration
- OctoMap real input integration
- FUEL integration
- TARE / GBPlanner integration
- Real flight controller integration

## Safety Notes

This version does not publish:

- `/cmd_vel`
- `/mavros/*`
- `/fmu/*`
- `/actuator/*`
- `/offboard_control_mode`
- `/trajectory_setpoint`

The fake executor only publishes `/bimodal/odom` and visualization trajectory/status topics.

## Backup Target

GitHub repository:

```text
https://github.com/PerryFish/3DPlanner_FULL
```

## Backup Execution Result

Generated on: 2026-07-01

### Git Repository Note

The default `git` discovery from this path resolves to `/home/nuaa`, which is outside the project and includes unrelated home-directory files. To avoid committing unrelated files, this backup used an isolated project git metadata directory:

```text
/home/nuaa/ZHY/3DPlanner_FULL/.git_3dplanner_full
```

Use the following form for local operations if needed:

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
git --git-dir=.git_3dplanner_full --work-tree=. status
git --git-dir=.git_3dplanner_full --work-tree=. log --oneline --decorate -1
```

### Commit and Tag

Local commit and tag were created. Use this command to inspect the exact local commit hash:

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
git --git-dir=.git_3dplanner_full --work-tree=. log --oneline --decorate -1
```

Tag:

```text
p2c-live-rviz-baseline-20260701
```

### Build Regression

```text
Map build PASS
Air build PASS
Ground build PASS
```

### Pre-Commit Safety Checks

```text
large_file_check=PASS_WITH_IGNORED_TEST_LOG_ONLY
secret_check=PASS
real_control_publisher_check=PASS
```

Large files were found only under `test-log/`, which is ignored and was not staged.

Credential keyword scan found no secrets.

Real-control keyword hits were limited to documentation and existing safety-check scripts; no publisher to real control topics was found.

### GitHub Push

Push was attempted once and failed because the execution environment could not connect to GitHub:

```text
fatal: unable to access 'https://github.com/PerryFish/3DPlanner_FULL.git/': Couldn't connect to server
```

`gh auth status` also reported that the local GitHub token is no longer valid. Re-authenticate from a normal terminal before pushing:

```bash
gh auth login -h github.com
cd /home/nuaa/ZHY/3DPlanner_FULL
git --git-dir=.git_3dplanner_full --work-tree=. push -u origin main
git --git-dir=.git_3dplanner_full --work-tree=. push origin --tags
```
