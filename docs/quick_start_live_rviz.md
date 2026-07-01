# Quick Start: Live RViz Visual Exploration Demo

Current stage: P2C live RViz visual exploration baseline.

## One-Shot Run

Run from an Ubuntu graphical desktop terminal:

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
bash scripts/run_live_rviz_demo_all_in_one.sh --mode-switch-period 60
```

This starts the visual demo, waits for TF readiness, checks visual topics, and launches RViz.

## Three-Terminal Run

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

## Notes

- Running `check_rviz_tf_ready.sh` alone does not start the demo.
- If the demo is not running, missing `/tf` and `/tf_static` is expected.
- RViz should be started after `RVIZ_FIXED_FRAME_READY=PASS`.
