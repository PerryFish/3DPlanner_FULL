#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
MODE=${P6A_MODE:-}

usage() {
  cat <<EOF
P6A unified launcher

Usage:
  P6A_MODE=p5b_synthetic ./scripts/run_p6a_unified_demo_launcher.sh
  P6A_MODE=p5c_synthetic_validation ./scripts/run_p6a_unified_demo_launcher.sh
  P6A_MODE=p5c_bag P5C_BAG_PATH=/path/to/bag ./scripts/run_p6a_unified_demo_launcher.sh
  P6A_MODE=p5c_live_topic P5C_INPUT_TOPIC=/camera/depth/points ./scripts/run_p6a_unified_demo_launcher.sh
  P6A_MODE=rviz_only ./scripts/run_p6a_unified_demo_launcher.sh
  P6A_MODE=status ./scripts/run_p6a_unified_demo_launcher.sh
EOF
}

case "$MODE" in
  p5b_synthetic)
    exec "$ROOT/scripts/run_p5b_explainable_bimodal_live_demo.sh"
    ;;
  p5c_synthetic_validation)
    export P5C_ALLOW_SYNTHETIC_FALLBACK=${P5C_ALLOW_SYNTHETIC_FALLBACK:-1}
    export P5C_DURATION_SEC=${P5C_DURATION_SEC:-180}
    exec "$ROOT/scripts/run_p5c_sensor_interface_validation.sh"
    ;;
  p5c_bag)
    if [ -z "${P5C_BAG_PATH:-}" ]; then
      echo "P5C_BAG_PATH is required for P6A_MODE=p5c_bag" >&2
      usage
      exit 2
    fi
    exec "$ROOT/scripts/run_p5c_real_bag_replay_live_demo.sh"
    ;;
  p5c_live_topic)
    if [ -z "${P5C_INPUT_TOPIC:-}" ]; then
      echo "P5C_INPUT_TOPIC is required for P6A_MODE=p5c_live_topic" >&2
      usage
      exit 2
    fi
    exec "$ROOT/scripts/run_p5c_live_external_pointcloud_demo.sh"
    ;;
  rviz_only)
    set +u
    source "$ROOT/scripts/env_visual_demo.sh"
    set -u
    exec rviz2 -d "$ROOT/Map/rviz/p5b_explainable_bimodal_demo.rviz"
    ;;
  status)
    exec "$ROOT/scripts/check_p5b_live_demo_status.sh"
    ;;
  *)
    usage
    exit 2
    ;;
esac
