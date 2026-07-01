import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class SyntheticExternalPointcloudPublisher(Node):
    def __init__(self):
        super().__init__('synthetic_external_pointcloud_publisher_node')
        self.declare_parameter('output_topic', '/points_raw')
        self.declare_parameter('frame_id', 'lidar_link')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('point_count', 1200)
        self.declare_parameter('range_m', 7.0)
        self.tick = 0
        self.pub = self.create_publisher(PointCloud2, str(self.get_parameter('output_topic').value), 10)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self._publish)
        self.get_logger().info(
            f'synthetic external pointcloud publisher active topic={self.get_parameter("output_topic").value} '
            f'frame_id={self.get_parameter("frame_id").value}'
        )

    def _publish(self):
        self.tick += 1
        frame = str(self.get_parameter('frame_id').value)
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=frame)
        points = self._points()
        self.pub.publish(point_cloud2.create_cloud_xyz32(header, points))

    def _points(self):
        count = max(int(self.get_parameter('point_count').value), 100)
        rng = float(self.get_parameter('range_m').value)
        phase = self.tick * 0.06
        pts = []
        # Moving floor patch.
        center_x = 0.08 * self.tick
        center_y = 1.5 * math.sin(phase)
        grid = int(math.sqrt(count * 0.45))
        for ix in range(-grid, grid + 1):
            for iy in range(-grid, grid + 1):
                x = center_x + ix * 0.18
                y = center_y + iy * 0.18
                if math.hypot(x, y) > rng:
                    continue
                z = 0.02 * math.sin(phase + ix * 0.2)
                pts.append((x, y, z))
        # Wall/frontier strip moving slowly through the map.
        for k in range(count // 4):
            y = -3.5 + (k % 60) * 0.12
            z = 0.1 + (k // 60) * 0.16
            x = center_x + 3.0 + 0.5 * math.sin(phase + y)
            pts.append((x, y, min(z, 2.8)))
        # Obstacle/cube shell.
        cube_cx = center_x + 1.2 * math.sin(phase * 0.5)
        cube_cy = -1.4 + 1.0 * math.cos(phase * 0.4)
        for k in range(count // 3):
            a = (k * 37 % 360) * math.pi / 180.0
            h = (k % 30) / 30.0
            radius = 0.45 + 0.08 * math.sin(k)
            pts.append((cube_cx + radius * math.cos(a), cube_cy + radius * math.sin(a), 0.15 + h * 1.6))
        return pts[:count]


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticExternalPointcloudPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
