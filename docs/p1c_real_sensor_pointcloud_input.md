# P1C Real Sensor PointCloud2 Input

Current supported input modes:

- `virtual`: keeps the existing virtual sensor as the only `/bimodal/points` publisher.
- `external_pointcloud`: bridges an external `sensor_msgs/msg/PointCloud2` topic into `/bimodal/points`.
- `recorded_bag`: same bridge path as external input; use `ros2 bag play` to provide the source topic.
- `hybrid`: bridge-owned synthetic fallback after input timeout. This avoids two continuous `/bimodal/points` publishers and is marked `PASS_PARTIAL`.

Synthetic validation:

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
bash scripts/run_p1c_real_sensor_pointcloud_input_validation.sh --duration 90 --no-rviz --input-topic /points_raw --mode-switch-period 75
```

Live RViz with synthetic external cloud:

```bash
cd /home/nuaa/ZHY/3DPlanner_FULL
bash scripts/run_p1c_live_rviz_external_pointcloud.sh --duration 300 --input-topic /points_raw --mode-switch-period 75
```

Recorded bag placeholder:

```bash
# terminal 1
cd /home/nuaa/ZHY/3DPlanner_FULL
source scripts/env_visual_demo.sh
ros2 bag play /path/to/bag --clock

# terminal 2
cd /home/nuaa/ZHY/3DPlanner_FULL
bash scripts/run_p1c_real_sensor_pointcloud_input_validation.sh \
  --duration 180 \
  --no-rviz \
  --input-topic /your/bag/pointcloud_topic \
  --mode-switch-period 75
```

The bridge diagnostics topic is `/bimodal/sensor_input_status`.
