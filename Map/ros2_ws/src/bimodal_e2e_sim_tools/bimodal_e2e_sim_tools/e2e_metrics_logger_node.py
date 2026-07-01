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
        self.current_mode = ''
        self.last_coverage_proxy = None
        self.first_coverage_proxy = None
        self.last_occupied_voxel_count = None
        self.first_occupied_voxel_count = None
        self.last_executed_path_length = 0.0
        self.last_active_path_count_time = None
        self.last_active_path_count = 0
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
        self._open_csv(
            'p2d_goal_quality',
            'p2d_goal_quality.csv',
            [
                'timestamp', 'mode', 'source', 'goal_x', 'goal_y', 'goal_z', 'selected_score',
                'coverage_gain_estimate', 'frontier_ring_score', 'path_usefulness_score',
                'blacklist_size', 'held_goal', 'switched_goal', 'raw_status',
            ],
        )
        self._open_csv(
            'p2d_coverage_efficiency',
            'p2d_coverage_efficiency.csv',
            ['timestamp', 'coverage_proxy', 'coverage_delta', 'executed_path_length', 'coverage_gain_per_meter', 'map_point_count'],
        )
        self._open_csv(
            'p2d_path_stability',
            'p2d_path_stability.csv',
            [
                'timestamp', 'active_path_count', 'accepted_path_update_count',
                'ignored_path_update_count', 'path_switch_rate', 'current_mode',
            ],
        )
        self._open_csv(
            'p2e_octomap_map_metrics',
            'p2e_octomap_map_metrics.csv',
            [
                'timestamp', 'occupied_voxel_count', 'map_point_count', 'coverage_proxy',
                'coverage_delta_last_10s', 'occupied_density', 'frontier_candidate_proxy',
                'unknown_boundary_proxy', 'backend_mode', 'is_real_octomap_server',
            ],
        )
        self._open_csv(
            'p2e_octomap_planner_quality',
            'p2e_octomap_planner_quality.csv',
            [
                'timestamp', 'source', 'backend_mode', 'selected_score', 'coverage_gain_estimate',
                'octomap_frontier_gain', 'unknown_boundary_gain', 'occupied_penalty',
                'path_collision_risk', 'valid_candidate_count', 'rejected_by_occupied_count',
                'low_gain_blacklist_size', 'raw_status',
            ],
        )
        self._open_csv(
            'p2e_octomap_exploration_efficiency',
            'p2e_octomap_exploration_efficiency.csv',
            [
                'timestamp', 'coverage_proxy', 'coverage_delta', 'occupied_voxel_count',
                'executed_path_length', 'coverage_gain_per_meter', 'occupied_voxel_gain_per_meter',
            ],
        )

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
        self.current_mode = msg.data.strip()
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
            now = self._now()
            self.last_active_path_count_time = now
            self.last_active_path_count = self.counts['active_path']
        elif source == 'air':
            self.counts['air_trajectory'] += 1
        elif source == 'ground':
            self.counts['ground_path'] += 1
        length = self._path_length(msg)
        if source == 'active':
            self.last_executed_path_length = max(self.last_executed_path_length, self.total_distance)
        self.writers['paths'].writerow([self._now(), source, len(msg.poses), length])
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
        coverage = self._float(values.get('coverage_proxy'), 0.0)
        occupied = self._float(values.get('occupied_voxel_count', values.get('accumulated_voxel_count')), 0.0)
        if self.first_coverage_proxy is None:
            self.first_coverage_proxy = coverage
        if self.first_occupied_voxel_count is None:
            self.first_occupied_voxel_count = occupied
        coverage_delta = 0.0 if self.last_coverage_proxy is None else coverage - self.last_coverage_proxy
        self.last_coverage_proxy = coverage
        self.last_occupied_voxel_count = occupied
        map_point_count = values.get('accumulated_point_count', '')
        gain_per_meter = 0.0
        if self.total_distance > 1e-6:
            gain_per_meter = coverage / self.total_distance
        self.writers['map_metrics'].writerow([
            self._now(),
            values.get('accumulated_point_count', ''),
            values.get('accumulated_voxel_count', ''),
            values.get('coverage_proxy', ''),
        ])
        self.files['map_metrics'].flush()
        self.writers['p2d_coverage_efficiency'].writerow([
            self._now(),
            f'{coverage:.6f}',
            f'{coverage_delta:.6f}',
            f'{self.total_distance:.3f}',
            f'{gain_per_meter:.9f}',
            map_point_count,
        ])
        self.files['p2d_coverage_efficiency'].flush()
        coverage_gain_per_meter = 0.0
        occupied_gain_per_meter = 0.0
        if self.total_distance > 1e-6:
            coverage_gain_per_meter = (coverage - self.first_coverage_proxy) / self.total_distance
            occupied_gain_per_meter = (occupied - self.first_occupied_voxel_count) / self.total_distance
        self.writers['p2e_octomap_map_metrics'].writerow([
            self._now(),
            values.get('occupied_voxel_count', values.get('accumulated_voxel_count', '')),
            values.get('map_point_count', values.get('accumulated_point_count', '')),
            values.get('coverage_proxy', ''),
            values.get('coverage_delta_last_10s', ''),
            values.get('occupied_density', ''),
            values.get('frontier_candidate_proxy', ''),
            values.get('unknown_boundary_proxy', ''),
            values.get('backend_mode', ''),
            values.get('is_real_octomap_server', ''),
        ])
        self.files['p2e_octomap_map_metrics'].flush()
        self.writers['p2e_octomap_exploration_efficiency'].writerow([
            self._now(),
            f'{coverage:.6f}',
            f'{coverage - self.first_coverage_proxy:.6f}',
            f'{occupied:.0f}',
            f'{self.total_distance:.3f}',
            f'{coverage_gain_per_meter:.9f}',
            f'{occupied_gain_per_meter:.6f}',
        ])
        self.files['p2e_octomap_exploration_efficiency'].flush()

    def _air_status_cb(self, msg):
        self.air_status = msg.data
        self._write_goal_quality('air', msg.data)
        self._write_status()

    def _ground_status_cb(self, msg):
        self.ground_status = msg.data
        self._write_goal_quality('ground', msg.data)
        self._write_status()

    def _executor_status_cb(self, msg):
        self.executor_status = msg.data
        self._write_path_stability(msg.data)
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

    def _write_goal_quality(self, source, status):
        values = self._parse_key_values(status)
        gx, gy, gz = self._parse_goal(values.get('selected_goal', ''))
        self.writers['p2d_goal_quality'].writerow([
            self._now(),
            self.current_mode,
            source,
            gx,
            gy,
            gz,
            values.get('selected_score', ''),
            values.get('coverage_gain_estimate', ''),
            values.get('frontier_ring_score', ''),
            values.get('path_usefulness_score', ''),
            values.get('low_gain_blacklist_size', values.get('blacklist_size', '')),
            values.get('held_goal', ''),
            values.get('switched_goal', ''),
            status,
        ])
        self.files['p2d_goal_quality'].flush()
        self.writers['p2e_octomap_planner_quality'].writerow([
            self._now(),
            source,
            values.get('backend_mode', ''),
            values.get('selected_score', ''),
            values.get('coverage_gain_estimate', ''),
            values.get('octomap_frontier_gain', values.get('frontier_ring_score', '')),
            values.get('unknown_boundary_gain', ''),
            values.get('occupied_penalty', ''),
            values.get('path_collision_risk', ''),
            values.get('valid_candidate_count', ''),
            values.get('rejected_by_occupied_count', ''),
            values.get('low_gain_blacklist_size', ''),
            status,
        ])
        self.files['p2e_octomap_planner_quality'].flush()

    def _write_path_stability(self, status):
        values = self._parse_key_values(status)
        accepted = self._float(values.get('accepted_path_update_count'), 0.0)
        ignored = self._float(values.get('ignored_path_update_count'), 0.0)
        path_switch_rate = 0.0
        if accepted + ignored > 0:
            path_switch_rate = accepted / (accepted + ignored)
        self.writers['p2d_path_stability'].writerow([
            self._now(),
            self.counts['active_path'],
            int(accepted),
            int(ignored),
            f'{path_switch_rate:.6f}',
            values.get('current_mode', self.current_mode),
        ])
        self.files['p2d_path_stability'].flush()

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

    @staticmethod
    def _float(value, default=0.0):
        try:
            return float(value)
        except Exception:
            return default

    @staticmethod
    def _parse_goal(text):
        nums = re.findall(r'-?\d+(?:\.\d+)?', text or '')
        vals = [float(v) for v in nums[:3]]
        while len(vals) < 3:
            vals.append('')
        return vals[0], vals[1], vals[2]

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
