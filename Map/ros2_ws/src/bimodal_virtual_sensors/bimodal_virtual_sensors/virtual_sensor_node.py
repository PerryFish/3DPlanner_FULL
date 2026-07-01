import math
import struct

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker


class VirtualSensorNode(Node):
    def __init__(self):
        super().__init__('virtual_sensor_node')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('use_external_odom', False)
        self.declare_parameter('external_odom_topic', '/bimodal/odom')
        self.declare_parameter('robot_mode_test', 'AIR')
        self.declare_parameter('air_start_z', 1.5)
        self.declare_parameter('ground_start_z', 0.2)
        self.declare_parameter('sensor_range', 5.0)
        self.declare_parameter('sensor_fov_deg', 120.0)
        self.declare_parameter('publish_world_full_cloud', False)
        self.declare_parameter('publish_world_gt_cloud', False)
        self.declare_parameter('publish_local_sensor_cloud', True)
        self.declare_parameter('world_size_x', 20.0)
        self.declare_parameter('world_size_y', 20.0)
        self.declare_parameter('world_size_z', 3.0)
        self.declare_parameter('depth_width', 64)
        self.declare_parameter('depth_height', 48)

        self.publish_odom_enabled = bool(self.get_parameter('publish_odom').value)
        self.publish_tf_enabled = bool(self.get_parameter('publish_tf').value)
        self.use_external_odom = bool(self.get_parameter('use_external_odom').value)
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_z = self._start_z()
        self.current_mode = str(self.get_parameter('robot_mode_test').value).upper()
        self.last_local_count = 0

        self.points_pub = self.create_publisher(PointCloud2, '/bimodal/points', 10)
        self.world_gt_pub = self.create_publisher(PointCloud2, '/bimodal/world_gt_cloud', 10)
        self.depth_pub = self.create_publisher(Image, '/bimodal/depth/image', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/bimodal/camera_info', 10)
        self.robot_marker_pub = self.create_publisher(Marker, '/bimodal/robot_marker', 10)
        self.sensor_range_marker_pub = self.create_publisher(Marker, '/bimodal/sensor_range_marker', 10)
        self.current_pose_marker_pub = self.create_publisher(Marker, '/bimodal/current_pose_marker', 10)
        self.odom_pub = None
        if self.publish_odom_enabled and not self.use_external_odom:
            self.odom_pub = self.create_publisher(Odometry, '/bimodal/odom', 10)
        if self.use_external_odom:
            topic = str(self.get_parameter('external_odom_topic').value)
            self.create_subscription(Odometry, topic, self._external_odom_cb, 10)
        self.create_subscription(String, '/bimodal/active_mode', self._active_mode_cb, 10)

        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf_enabled else None
        self.static_broadcaster = StaticTransformBroadcaster(self) if self.publish_tf_enabled else None
        self.world_points = self._make_environment_points()
        if self.publish_tf_enabled:
            self._publish_static_tf()
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self._on_timer)
        self.status_timer = self.create_timer(5.0, self._log_status)
        self.get_logger().info(
            'virtual sensor active '
            f'publish_odom={self.publish_odom_enabled} publish_tf={self.publish_tf_enabled} '
            f'use_external_odom={self.use_external_odom} '
            f'world_points={len(self.world_points)}'
        )

    def _start_z(self):
        mode = str(self.get_parameter('robot_mode_test').value).upper()
        if mode == 'GROUND':
            return float(self.get_parameter('ground_start_z').value)
        return float(self.get_parameter('air_start_z').value)

    def _external_odom_cb(self, msg):
        self.pose_x = float(msg.pose.pose.position.x)
        self.pose_y = float(msg.pose.pose.position.y)
        self.pose_z = float(msg.pose.pose.position.z)

    def _active_mode_cb(self, msg):
        mode = msg.data.strip().upper()
        if mode in ('AIR', 'GROUND', 'IDLE'):
            self.current_mode = mode

    def _make_environment_points(self):
        pts = []
        sx = float(self.get_parameter('world_size_x').value) / 2.0
        sy = float(self.get_parameter('world_size_y').value) / 2.0
        sz = float(self.get_parameter('world_size_z').value)
        for ix in range(int(-sx * 10), int(sx * 10) + 1, 5):
            for iy in range(int(-sy * 10), int(sy * 10) + 1, 5):
                pts.append((ix / 10.0, iy / 10.0, 0.0))
        for wall_x in (-sx, sx):
            for iy in range(int(-sy * 10), int(sy * 10) + 1, 5):
                for iz in range(0, int(sz * 10) + 1, 5):
                    pts.append((wall_x, iy / 10.0, iz / 10.0))
        for wall_y in (-sy, sy):
            for ix in range(int(-sx * 10), int(sx * 10) + 1, 5):
                for iz in range(0, int(sz * 10) + 1, 5):
                    pts.append((ix / 10.0, wall_y, iz / 10.0))
        for cx, cy, w, d, h in [(-2.0, 2.5, 1.4, 1.0, 1.4), (4.0, -2.0, 1.0, 2.0, 2.4), (1.0, 5.0, 1.6, 1.2, 1.8)]:
            for ix in range(-10, 11, 2):
                for iy in range(-10, 11, 2):
                    for iz in range(0, 11, 2):
                        x = cx + ix * w / 20.0
                        y = cy + iy * d / 20.0
                        z = iz * h / 10.0
                        if abs(ix) == 10 or abs(iy) == 10 or iz in (0, 10):
                            pts.append((x, y, z))
        for k in range(360):
            x = -sx + (k * 37 % int(max(1, sx * 20))) / 10.0
            y = -sy + (k * 53 % int(max(1, sy * 20))) / 10.0
            z = 0.2 + (k * 29 % int(max(1, sz * 10))) / 10.0
            if abs(x) < 0.8 and abs(y) < 0.8:
                continue
            pts.append((x, y, min(z, sz)))
        return pts

    def _publish_static_tf(self):
        if not self.publish_tf_enabled or self.static_broadcaster is None:
            return
        now = self.get_clock().now().to_msg()
        transforms = [
            self._tf('map', 'odom', now, 0.0, 0.0, 0.0),
            self._tf('base_link', 'camera_link', now, 0.25, 0.0, 0.10),
            self._tf('base_link', 'lidar_link', now, 0.0, 0.0, 0.20),
        ]
        self.static_broadcaster.sendTransform(transforms)

    def _tf(self, parent, child, stamp, x, y, z):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.w = 1.0
        return tf

    def _on_timer(self):
        now = self.get_clock().now().to_msg()
        local_points = self._select_visible_points()
        self.last_local_count = len(local_points)
        header = Header(stamp=now, frame_id='map')
        self.points_pub.publish(point_cloud2.create_cloud_xyz32(header, local_points))
        if bool(self.get_parameter('publish_world_gt_cloud').value):
            self.world_gt_pub.publish(point_cloud2.create_cloud_xyz32(header, self.world_points))
        if self.odom_pub is not None:
            self._publish_odom(now)
        self._publish_dynamic_tf(now)
        self._publish_depth(now)
        self._publish_visual_markers(now)

    def _select_visible_points(self):
        if bool(self.get_parameter('publish_world_full_cloud').value):
            return self.world_points
        if not bool(self.get_parameter('publish_local_sensor_cloud').value):
            return []
        sensor_range = float(self.get_parameter('sensor_range').value)
        fov_rad = math.radians(float(self.get_parameter('sensor_fov_deg').value))
        cos_half_fov = math.cos(max(min(fov_rad, math.tau), 0.0) / 2.0)
        pts = []
        for x, y, z in self.world_points:
            dx = x - self.pose_x
            dy = y - self.pose_y
            dz = z - self.pose_z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist > sensor_range:
                continue
            # The virtual sensor faces +X. Keep a small near-field bubble to avoid blind startup.
            if dist > 1.0 and dist > 1e-6 and dx / dist < cos_half_fov:
                continue
            pts.append((x, y, z))
        return pts

    def _publish_odom(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.pose_x
        odom.pose.pose.position.y = self.pose_y
        odom.pose.pose.position.z = self.pose_z
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

    def _publish_dynamic_tf(self, stamp):
        if not self.publish_tf_enabled or self.tf_broadcaster is None:
            return
        tf = self._tf('odom', 'base_link', stamp, self.pose_x, self.pose_y, self.pose_z)
        self.tf_broadcaster.sendTransform(tf)

    def _publish_depth(self, stamp):
        width = int(self.get_parameter('depth_width').value)
        height = int(self.get_parameter('depth_height').value)
        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = 'camera_link'
        img.height = height
        img.width = width
        img.encoding = '32FC1'
        img.is_bigendian = 0
        img.step = width * 4
        values = []
        pose_term = 0.05 * math.sin(self.pose_x) + 0.05 * math.cos(self.pose_y)
        for v in range(height):
            for u in range(width):
                values.append(2.0 + pose_term + 0.5 * math.sin(u / 10.0) + 0.2 * math.cos(v / 8.0))
        img.data = b''.join(struct.pack('<f', value) for value in values)
        self.depth_pub.publish(img)

        info = CameraInfo()
        info.header = img.header
        info.height = height
        info.width = width
        info.k = [60.0, 0.0, width / 2.0, 0.0, 60.0, height / 2.0, 0.0, 0.0, 1.0]
        info.p = [60.0, 0.0, width / 2.0, 0.0, 0.0, 60.0, height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)

    def _publish_visual_markers(self, stamp):
        self.robot_marker_pub.publish(self._robot_marker(stamp))
        self.sensor_range_marker_pub.publish(self._sensor_range_marker(stamp))
        self.current_pose_marker_pub.publish(self._pose_marker(stamp))

    def _base_marker(self, stamp, ns, marker_id, marker_type):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = 'map'
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = self.pose_x
        marker.pose.position.y = self.pose_y
        marker.pose.position.z = self.pose_z
        marker.pose.orientation.w = 1.0
        return marker

    def _robot_marker(self, stamp):
        marker_type = Marker.SPHERE if self.current_mode == 'AIR' else Marker.CUBE
        marker = self._base_marker(stamp, 'bimodal_robot_marker', 1, marker_type)
        if self.current_mode == 'AIR':
            marker.scale.x = marker.scale.y = marker.scale.z = 0.45
            marker.color.r = 0.1
            marker.color.g = 0.45
            marker.color.b = 1.0
            marker.color.a = 0.9
        elif self.current_mode == 'GROUND':
            marker.scale.x = 0.55
            marker.scale.y = 0.35
            marker.scale.z = 0.22
            marker.color.r = 0.1
            marker.color.g = 0.85
            marker.color.b = 0.25
            marker.color.a = 0.9
        else:
            marker.scale.x = marker.scale.y = marker.scale.z = 0.35
            marker.color.r = 0.7
            marker.color.g = 0.7
            marker.color.b = 0.7
            marker.color.a = 0.7
        return marker

    def _sensor_range_marker(self, stamp):
        marker = self._base_marker(stamp, 'bimodal_sensor_range', 1, Marker.SPHERE)
        diameter = 2.0 * float(self.get_parameter('sensor_range').value)
        marker.scale.x = marker.scale.y = marker.scale.z = diameter
        marker.color.r = 1.0
        marker.color.g = 0.75
        marker.color.b = 0.1
        marker.color.a = 0.08
        return marker

    def _pose_marker(self, stamp):
        marker = self._base_marker(stamp, 'bimodal_current_pose', 1, Marker.ARROW)
        marker.scale.x = 0.8
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.9
        return marker

    def _log_status(self):
        self.get_logger().info(
            f'pose=({self.pose_x:.2f},{self.pose_y:.2f},{self.pose_z:.2f}) '
            f'local_cloud_point_count={self.last_local_count} '
            f'publish_odom_enabled={self.publish_odom_enabled} publish_tf_enabled={self.publish_tf_enabled} '
            f'use_external_odom={self.use_external_odom}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = VirtualSensorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
