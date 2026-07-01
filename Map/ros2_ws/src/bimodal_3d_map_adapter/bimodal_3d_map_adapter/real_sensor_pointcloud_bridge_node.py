import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String


class RealSensorPointcloudBridge(Node):
    def __init__(self):
        super().__init__('real_sensor_pointcloud_bridge_node')
        defaults = {
            'sensor_input_mode': 'external_pointcloud',
            'input_topic': '/points_raw',
            'output_topic': '/bimodal/points',
            'input_topics': ['/camera/depth/points', '/lidar/points', '/points_raw'],
            'selected_source': '',
            'target_frame': 'map',
            'use_tf_transform': False,
            'passthrough_frame_if_tf_missing': True,
            'min_range_m': 0.05,
            'max_range_m': 30.0,
            'max_points_per_cloud': 20000,
            'voxel_downsample_leaf_size': 0.05,
            'publish_rate_limit_hz': 10.0,
            'input_timeout_sec': 2.0,
            'enable_diagnostics': True,
            'preserve_real_header_stamp': True,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)

        self.mode = str(self.get_parameter('sensor_input_mode').value)
        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        self.external_cloud_received_count = 0
        self.output_cloud_published_count = 0
        self.input_timeout_count = 0
        self.dropped_cloud_count = 0
        self.reason_for_drop = 'none'
        self.last_input_stamp = 'none'
        self.last_output_stamp = 'none'
        self.last_input_frame = 'none'
        self.last_publish_time = 0.0
        self.last_input_time = None
        self.fallback_active = False
        self.fallback_tick = 0

        self.output_pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.status_pub = self.create_publisher(String, '/bimodal/sensor_input_status', 10)
        if self.mode in ('external_pointcloud', 'recorded_bag', 'hybrid'):
            self.create_subscription(PointCloud2, self.input_topic, self._cloud_cb, 10)
        self.create_timer(1.0, self._timer_cb)
        self.get_logger().info(
            f'real sensor pointcloud bridge active mode={self.mode} input_topic={self.input_topic} '
            f'output_topic={self.output_topic}'
        )

    def _cloud_cb(self, msg):
        if self.mode == 'virtual':
            return
        self.external_cloud_received_count += 1
        self.last_input_time = self.get_clock().now()
        self.last_input_frame = msg.header.frame_id or 'unknown'
        self.last_input_stamp = self._stamp_text(msg.header)
        points = self._filtered_downsampled_points(msg)
        if not points:
            self.dropped_cloud_count += 1
            self.reason_for_drop = 'no_valid_points_after_filter'
            self._publish_status()
            return
        self.reason_for_drop = 'none'
        self.fallback_active = False
        self._publish_points(points, msg.header)

    def _timer_cb(self):
        if self.mode == 'hybrid':
            timeout = float(self.get_parameter('input_timeout_sec').value)
            if self.last_input_time is None or self._age(self.last_input_time) > timeout:
                self.input_timeout_count += 1
                self.fallback_active = True
                self._publish_points(self._fallback_points(), None)
        self._publish_status()

    def _filtered_downsampled_points(self, msg):
        min_range = float(self.get_parameter('min_range_m').value)
        max_range = float(self.get_parameter('max_range_m').value)
        max_points = max(int(self.get_parameter('max_points_per_cloud').value), 1)
        leaf = float(self.get_parameter('voxel_downsample_leaf_size').value)
        voxel_seen = set()
        points = []
        for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            if not all(math.isfinite(v) for v in (x, y, z)):
                continue
            r = math.sqrt(x * x + y * y + z * z)
            if r < min_range or r > max_range:
                continue
            if leaf > 0:
                key = (round(x / leaf), round(y / leaf), round(z / leaf))
                if key in voxel_seen:
                    continue
                voxel_seen.add(key)
            points.append((x, y, z))
            if len(points) >= max_points:
                break
        return points

    def _fallback_points(self):
        self.fallback_tick += 1
        pts = []
        phase = self.fallback_tick * 0.08
        for ix in range(-18, 19, 2):
            for iy in range(-12, 13, 2):
                x = ix * 0.25 + 0.8 * math.sin(phase)
                y = iy * 0.25 + 0.6 * math.cos(phase * 0.7)
                z = 0.15 + ((ix + iy) % 5) * 0.08
                pts.append((x, y, z))
        for k in range(90):
            pts.append((2.0 + math.sin(phase + k * 0.1), -1.5 + math.cos(phase + k * 0.07), 0.2 + (k % 20) * 0.06))
        return pts

    def _publish_points(self, points, input_header):
        now_sec = time.monotonic()
        rate = float(self.get_parameter('publish_rate_limit_hz').value)
        if rate > 0.0 and now_sec - self.last_publish_time < 1.0 / rate:
            self.dropped_cloud_count += 1
            self.reason_for_drop = 'publish_rate_limited'
            return
        self.last_publish_time = now_sec
        header = Header()
        if input_header is not None and bool(self.get_parameter('preserve_real_header_stamp').value):
            header.stamp = input_header.stamp
        else:
            header.stamp = self.get_clock().now().to_msg()
        if bool(self.get_parameter('use_tf_transform').value):
            header.frame_id = self.target_frame
        elif bool(self.get_parameter('passthrough_frame_if_tf_missing').value) and input_header is not None and input_header.frame_id:
            header.frame_id = input_header.frame_id
        else:
            header.frame_id = self.target_frame
        self.output_pub.publish(point_cloud2.create_cloud_xyz32(header, points))
        self.output_cloud_published_count += 1
        self.last_output_stamp = self._stamp_text(header)

    def _publish_status(self):
        if not bool(self.get_parameter('enable_diagnostics').value):
            return
        msg = String()
        selected_topic = self.input_topic if self.mode != 'virtual' else 'virtual_sensor_direct'
        msg.data = (
            f'external_cloud_received_count={self.external_cloud_received_count} '
            f'output_cloud_published_count={self.output_cloud_published_count} '
            f'input_timeout_count={self.input_timeout_count} fallback_active={str(self.fallback_active).lower()} '
            f'dropped_cloud_count={self.dropped_cloud_count} selected_input_mode={self.mode} '
            f'selected_input_topic={selected_topic} output_topic={self.output_topic} last_input_stamp={self.last_input_stamp} '
            f'last_output_stamp={self.last_output_stamp} last_input_frame={self.last_input_frame} '
            f'output_frame={self.target_frame} reason_for_drop={self.reason_for_drop}'
        )
        self.status_pub.publish(msg)

    def _age(self, stamp):
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    @staticmethod
    def _stamp_text(header):
        return f'{int(header.stamp.sec)}.{int(header.stamp.nanosec):09d}'


def main(args=None):
    rclpy.init(args=args)
    node = RealSensorPointcloudBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
