import csv
import math
import os
import re

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


class E2EMetricsLogger(Node):
    def __init__(self):
        super().__init__('e2e_metrics_logger_node')
        self.declare_parameter('log_dir', '/tmp/bimodal_e2e')
        self.log_dir = str(self.get_parameter('log_dir').value)
        os.makedirs(self.log_dir, exist_ok=True)

        self.last_odom = None
        self.total_distance = 0.0
        self.air_status = ''
        self.ground_status = ''
        self.executor_status = ''
        self.counts = {
            'active_goal': 0,
            'active_path': 0,
            'air_trajectory': 0,
            'ground_path': 0,
            'mode': 0,
        }

        self.files = {}
        self.writers = {}
        self._open_csv('odom', 'e2e_odom.csv', ['timestamp', 'x', 'y', 'z', 'total_distance'])
        self._open_csv('mode', 'e2e_mode.csv', ['timestamp', 'mode'])
        self._open_csv('goals', 'e2e_goals.csv', ['timestamp', 'source', 'x', 'y', 'z'])
        self._open_csv('paths', 'e2e_paths.csv', ['timestamp', 'source', 'pose_count', 'path_length'])
        self._open_csv('map_metrics', 'e2e_map_metrics.csv', ['timestamp', 'accumulated_point_count', 'accumulated_voxel_count', 'coverage_proxy'])
        self._open_csv('status', 'e2e_status.csv', ['timestamp', 'air_status', 'ground_status', 'executor_status'])

        self.create_subscription(String, '/bimodal/active_mode', self._mode_cb, 10)
        self.create_subscription(PoseStamped, '/bimodal/active_goal', lambda m: self._goal_cb('active', m), 10)
        self.create_subscription(Path, '/bimodal/active_path', lambda m: self._path_cb('active', m), 10)
        self.create_subscription(Odometry, '/bimodal/odom', self._odom_cb, 10)
        self.create_subscription(PointCloud2, '/bimodal/map_3d', lambda m: None, 10)
        self.create_subscription(String, '/bimodal/map_metrics', self._map_metrics_cb, 10)
        self.create_subscription(PoseStamped, '/air/exploration_goal', lambda m: self._goal_cb('air', m), 10)
        self.create_subscription(Path, '/air/trajectory', lambda m: self._path_cb('air', m), 10)
        self.create_subscription(String, '/air/planner_status', self._air_status_cb, 10)
        self.create_subscription(PoseStamped, '/ground/exploration_goal', lambda m: self._goal_cb('ground', m), 10)
        self.create_subscription(Path, '/ground/path', lambda m: self._path_cb('ground', m), 10)
        self.create_subscription(String, '/ground/planner_status', self._ground_status_cb, 10)
        self.create_subscription(String, '/bimodal/fake_executor_status', self._executor_status_cb, 10)
        self.status_timer = self.create_timer(5.0, self._log_status)
        self.get_logger().info(f'e2e metrics logger writing CSV files under {self.log_dir}')

    def _open_csv(self, key, filename, header):
        path = os.path.join(self.log_dir, filename)
        handle = open(path, 'w', newline='', encoding='utf-8')
        writer = csv.writer(handle)
        writer.writerow(header)
        handle.flush()
        self.files[key] = handle
        self.writers[key] = writer

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _mode_cb(self, msg):
        self.counts['mode'] += 1
        self.writers['mode'].writerow([self._now(), msg.data.strip()])
        self.files['mode'].flush()

    def _goal_cb(self, source, msg):
        if source == 'active':
            self.counts['active_goal'] += 1
        p = msg.pose.position
        self.writers['goals'].writerow([self._now(), source, p.x, p.y, p.z])
        self.files['goals'].flush()

    def _path_cb(self, source, msg):
        if source == 'active':
            self.counts['active_path'] += 1
        elif source == 'air':
            self.counts['air_trajectory'] += 1
        elif source == 'ground':
            self.counts['ground_path'] += 1
        self.writers['paths'].writerow([self._now(), source, len(msg.poses), self._path_length(msg)])
        self.files['paths'].flush()

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        if self.last_odom is not None:
            dx = p.x - self.last_odom[0]
            dy = p.y - self.last_odom[1]
            dz = p.z - self.last_odom[2]
            self.total_distance += math.sqrt(dx * dx + dy * dy + dz * dz)
        self.last_odom = (p.x, p.y, p.z)
        self.writers['odom'].writerow([self._now(), p.x, p.y, p.z, self.total_distance])
        self.files['odom'].flush()

    def _map_metrics_cb(self, msg):
        values = self._parse_key_values(msg.data)
        self.writers['map_metrics'].writerow([
            self._now(),
            values.get('accumulated_point_count', ''),
            values.get('accumulated_voxel_count', ''),
            values.get('coverage_proxy', ''),
        ])
        self.files['map_metrics'].flush()

    def _air_status_cb(self, msg):
        self.air_status = msg.data
        self._write_status()

    def _ground_status_cb(self, msg):
        self.ground_status = msg.data
        self._write_status()

    def _executor_status_cb(self, msg):
        self.executor_status = msg.data
        self._write_status()

    def _write_status(self):
        self.writers['status'].writerow([self._now(), self.air_status, self.ground_status, self.executor_status])
        self.files['status'].flush()

    def _log_status(self):
        self.get_logger().info(
            f'e2e metrics total_distance={self.total_distance:.3f} '
            f'active_paths={self.counts["active_path"]} air_paths={self.counts["air_trajectory"]} '
            f'ground_paths={self.counts["ground_path"]}'
        )

    @staticmethod
    def _path_length(path):
        total = 0.0
        last = None
        for pose in path.poses:
            p = pose.pose.position
            if last is not None:
                dx = p.x - last[0]
                dy = p.y - last[1]
                dz = p.z - last[2]
                total += math.sqrt(dx * dx + dy * dy + dz * dz)
            last = (p.x, p.y, p.z)
        return total

    @staticmethod
    def _parse_key_values(text):
        out = {}
        for key, value in re.findall(r'([A-Za-z0-9_]+)=(\S+)', text):
            out[key] = value
        return out

    def destroy_node(self):
        for handle in self.files.values():
            handle.flush()
            handle.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = E2EMetricsLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
