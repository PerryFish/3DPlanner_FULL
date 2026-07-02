import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


class SyntheticExternalPointcloudPublisher(Node):
    def __init__(self):
        super().__init__('synthetic_external_pointcloud_publisher_node')
        self.declare_parameter('output_topic', '/points_raw')
        self.declare_parameter('frame_id', 'lidar_link')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('point_count', 1200)
        self.declare_parameter('range_m', 7.0)
        self.declare_parameter('scene_profile', 'default_sparse')
        self.declare_parameter('reveal_speed_mps', 0.08)
        self.declare_parameter('world_marker_topic', '/bimodal/demo_world_structure_markers')
        self.tick = 0
        self.pub = self.create_publisher(PointCloud2, str(self.get_parameter('output_topic').value), 10)
        self.world_marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('world_marker_topic').value),
            10,
        )
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self._publish)
        self.get_logger().info(
            f'synthetic external pointcloud publisher active topic={self.get_parameter("output_topic").value} '
            f'frame_id={self.get_parameter("frame_id").value} '
            f'scene_profile={self.get_parameter("scene_profile").value}'
        )

    def _publish(self):
        self.tick += 1
        frame = str(self.get_parameter('frame_id').value)
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=frame)
        points = self._points()
        self.pub.publish(point_cloud2.create_cloud_xyz32(header, points))
        if str(self.get_parameter('scene_profile').value) == 'realistic_room_corridor_v1':
            self._publish_world_structure_markers(header.stamp)

    def _points(self):
        if str(self.get_parameter('scene_profile').value) == 'realistic_room_corridor_v1':
            return self._realistic_room_corridor_points()
        if str(self.get_parameter('scene_profile').value) == 'warehouse_obstacles_v1':
            return self._warehouse_obstacle_points()
        return self._default_sparse_points()

    def _default_sparse_points(self):
        count = max(int(self.get_parameter('point_count').value), 100)
        rng = float(self.get_parameter('range_m').value)
        phase = self.tick * 0.06
        pts = []
        # Keep long-run synthetic clouds inside the bridge range while still
        # sweeping new map volume over several minutes.
        cycle = (self.tick % 1200) / 1200.0
        center_x = -6.0 + 12.0 * cycle + 0.5 * math.sin(phase * 0.41)
        center_y = 2.2 * math.sin(phase * 0.73) + 0.7 * math.sin(self.tick * 0.011)
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
            x = center_x + 2.5 + 0.6 * math.sin(phase + y)
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

    def _realistic_room_corridor_points(self):
        count = max(int(self.get_parameter('point_count').value), 800)
        rng = float(self.get_parameter('range_m').value)
        reveal_speed = float(self.get_parameter('reveal_speed_mps').value)
        reveal_x = min(9.0, -8.5 + self.tick * reveal_speed)
        phase = self.tick * 0.05
        pts = []

        def add(x, y, z):
            if x <= reveal_x + 0.2 and math.hypot(x, y) <= rng + max(reveal_x + 4.0, 0.0):
                pts.append((x, y, z))

        # Floor: sparse, slightly uneven, and revealed from the start area
        # toward the open room so the map grows instead of appearing at once.
        for ix in range(-90, 91, 2):
            x = ix * 0.1
            if x > reveal_x:
                continue
            y_limit = 2.1 if x < 1.5 else 5.0
            for iy in range(int(-y_limit * 10), int(y_limit * 10) + 1, 3):
                y = iy * 0.1
                z = 0.02 * math.sin(0.8 * x + 0.4 * y + phase)
                add(x, y, z)

        # Corridor side walls, with a doorway gap into the open area.
        for x_step in range(-80, 26, 2):
            x = x_step * 0.1
            if x > reveal_x:
                continue
            for wall_y in (-2.25, 2.25):
                for iz in range(0, 24, 2):
                    z = iz * 0.1
                    # Doorway/gap near x=0.8.
                    if 0.2 < x < 1.5 and wall_y > 0.0 and z < 1.9:
                        continue
                    add(x, wall_y, z)

        # Open-room outer walls.
        for x_step in range(15, 91, 2):
            x = x_step * 0.1
            if x > reveal_x:
                continue
            for wall_y in (-5.1, 5.1):
                for iz in range(0, 28, 2):
                    add(x, wall_y, iz * 0.1)
        for y_step in range(-50, 51, 2):
            y = y_step * 0.1
            for wall_x in (1.5, 9.1):
                if wall_x > reveal_x:
                    continue
                for iz in range(0, 28, 2):
                    # Keep the corridor doorway visible.
                    if wall_x < 2.0 and -1.2 < y < 1.2 and iz * 0.1 < 2.0:
                        continue
                    add(wall_x, y, iz * 0.1)

        # Box obstacles and a pillar make traversability/frontier decisions
        # easier to interpret in RViz.
        for cx, cy, sx, sy, h in ((3.3, -2.1, 0.9, 0.8, 1.1), (5.6, 1.6, 1.1, 0.7, 1.4), (7.3, -0.6, 0.7, 1.0, 0.9)):
            if cx - sx > reveal_x:
                continue
            self._add_box_shell(pts, add, cx, cy, sx, sy, h)
        pillar_x, pillar_y = 4.2, 3.2
        if pillar_x - 0.5 <= reveal_x:
            for k in range(90):
                angle = k * 2.399963
                radius = 0.35 + 0.03 * math.sin(k)
                z = (k % 28) * 0.09
                add(pillar_x + radius * math.cos(angle), pillar_y + radius * math.sin(angle), min(z, 2.5))

        return pts[:count]

    def _warehouse_obstacle_points(self):
        base = self._realistic_room_corridor_points()
        reveal_x = min(9.0, -8.5 + self.tick * float(self.get_parameter('reveal_speed_mps').value))
        pts = list(base)

        def add(x, y, z):
            if x <= reveal_x + 0.2:
                pts.append((x, y, z))

        for cx in (-3.0, -1.0, 2.0, 4.5, 7.0):
            for cy in (-3.2, 0.0, 3.2):
                if cx - 0.45 <= reveal_x:
                    self._add_box_shell(pts, add, cx, cy, 0.45, 0.45, 1.5)
        return pts[:max(int(self.get_parameter('point_count').value), 800)]

    @staticmethod
    def _add_box_shell(pts, add, cx, cy, sx, sy, h):
        _ = pts
        z_steps = max(int(h / 0.12), 2)
        for ix in range(-int(sx * 10), int(sx * 10) + 1, 2):
            x = cx + ix * 0.1
            for iz in range(0, z_steps + 1):
                z = min(iz * 0.12, h)
                add(x, cy - sy, z)
                add(x, cy + sy, z)
        for iy in range(-int(sy * 10), int(sy * 10) + 1, 2):
            y = cy + iy * 0.1
            for iz in range(0, z_steps + 1):
                z = min(iz * 0.12, h)
                add(cx - sx, y, z)
                add(cx + sx, y, z)
        for ix in range(-int(sx * 10), int(sx * 10) + 1, 2):
            for iy in range(-int(sy * 10), int(sy * 10) + 1, 2):
                add(cx + ix * 0.1, cy + iy * 0.1, h)

    def _publish_world_structure_markers(self, stamp):
        frame = str(self.get_parameter('frame_id').value)
        markers = [self._clear_marker(stamp, frame, 'p5b_world_structure')]
        specs = [
            ('corridor_floor', (-3.8, 0.0, -0.015), (9.5, 4.4, 0.03), (0.45, 0.45, 0.45, 0.22)),
            ('open_area_floor', (5.3, 0.0, -0.012), (7.6, 10.0, 0.03), (0.45, 0.45, 0.45, 0.18)),
            ('left_corridor_wall', (-3.8, 2.25, 1.15), (9.5, 0.12, 2.3), (0.8, 0.8, 0.8, 0.28)),
            ('right_corridor_wall', (-3.8, -2.25, 1.15), (9.5, 0.12, 2.3), (0.8, 0.8, 0.8, 0.28)),
            ('open_room_wall_left', (5.3, 5.1, 1.25), (7.6, 0.14, 2.5), (0.8, 0.8, 0.8, 0.24)),
            ('open_room_wall_right', (5.3, -5.1, 1.25), (7.6, 0.14, 2.5), (0.8, 0.8, 0.8, 0.24)),
            ('box_obstacle_a', (3.3, -2.1, 0.55), (1.8, 1.6, 1.1), (0.9, 0.25, 0.1, 0.35)),
            ('box_obstacle_b', (5.6, 1.6, 0.7), (2.2, 1.4, 1.4), (0.9, 0.25, 0.1, 0.35)),
            ('pillar', (4.2, 3.2, 1.25), (0.7, 0.7, 2.5), (0.9, 0.25, 0.1, 0.32)),
        ]
        for i, (_, pos, scale, color) in enumerate(specs, start=1):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = frame
            marker.ns = 'p5b_world_structure'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = pos
            marker.pose.orientation.w = 1.0
            marker.scale.x, marker.scale.y, marker.scale.z = scale
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
            markers.append(marker)
        self.world_marker_pub.publish(MarkerArray(markers=markers))

    @staticmethod
    def _clear_marker(stamp, frame, ns):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame
        marker.ns = ns
        marker.id = 0
        marker.action = Marker.DELETEALL
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticExternalPointcloudPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
