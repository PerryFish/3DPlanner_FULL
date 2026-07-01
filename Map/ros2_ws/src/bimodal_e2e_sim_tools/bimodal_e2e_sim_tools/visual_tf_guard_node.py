import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class VisualTfGuard(Node):
    def __init__(self):
        super().__init__('visual_tf_guard_node')
        self.declare_parameter('enable_tf_guard', True)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('lidar_frame', 'lidar_link')
        self.declare_parameter('camera_xyz', [0.2, 0.0, 0.1])
        self.declare_parameter('lidar_xyz', [0.2, 0.0, 0.1])
        self.declare_parameter('dynamic_tf_publish_hz', 20.0)
        self.declare_parameter('static_tf_republish_period_sec', 2.0)
        self.declare_parameter('use_odom_pose', True)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 1.5)
        self.declare_parameter('initial_yaw', 0.0)

        self.enabled = bool(self.get_parameter('enable_tf_guard').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.lidar_frame = str(self.get_parameter('lidar_frame').value)
        self.camera_xyz = self._xyz_param('camera_xyz')
        self.lidar_xyz = self._xyz_param('lidar_xyz')
        self.use_odom_pose = bool(self.get_parameter('use_odom_pose').value)

        self.pose_x = float(self.get_parameter('initial_x').value)
        self.pose_y = float(self.get_parameter('initial_y').value)
        self.pose_z = float(self.get_parameter('initial_z').value)
        self.pose_qx = 0.0
        self.pose_qy = 0.0
        self.pose_qz = math.sin(float(self.get_parameter('initial_yaw').value) * 0.5)
        self.pose_qw = math.cos(float(self.get_parameter('initial_yaw').value) * 0.5)
        self.odom_received = False
        self.last_odom_time = None
        self.dynamic_tf_publish_count = 0
        self.static_tf_publish_count = 0

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.status_pub = self.create_publisher(String, '/bimodal/tf_guard_status', 10)
        self.create_subscription(Odometry, '/bimodal/odom', self._odom_cb, 10)

        dynamic_hz = float(self.get_parameter('dynamic_tf_publish_hz').value)
        static_period = float(self.get_parameter('static_tf_republish_period_sec').value)
        self.dynamic_timer = self.create_timer(1.0 / max(dynamic_hz, 0.1), self._publish_dynamic_tf)
        self.static_timer = self.create_timer(max(static_period, 0.1), self._publish_static_tf)
        self.status_timer = self.create_timer(1.0, self._publish_status)
        self._publish_static_tf()
        self._publish_status()
        self.get_logger().info(
            'visual TF guard active: '
            f'{self.map_frame}->{self.odom_frame}->{self.base_frame}, '
            f'{self.base_frame}->{self.camera_frame}/{self.lidar_frame}'
        )

    def _xyz_param(self, name):
        raw = list(self.get_parameter(name).value)
        values = [float(v) for v in raw[:3]]
        while len(values) < 3:
            values.append(0.0)
        return values

    def _odom_cb(self, msg):
        if not self.enabled or not self.use_odom_pose:
            return
        self.pose_x = float(msg.pose.pose.position.x)
        self.pose_y = float(msg.pose.pose.position.y)
        self.pose_z = float(msg.pose.pose.position.z)
        self.pose_qx = float(msg.pose.pose.orientation.x)
        self.pose_qy = float(msg.pose.pose.orientation.y)
        self.pose_qz = float(msg.pose.pose.orientation.z)
        self.pose_qw = float(msg.pose.pose.orientation.w)
        if self.pose_qw == 0.0 and self.pose_qx == 0.0 and self.pose_qy == 0.0 and self.pose_qz == 0.0:
            self.pose_qw = 1.0
        self.odom_received = True
        self.last_odom_time = self.get_clock().now()

    def _tf(self, parent, child, stamp, xyz, quat=(0.0, 0.0, 0.0, 1.0)):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tf.transform.translation.x = float(xyz[0])
        tf.transform.translation.y = float(xyz[1])
        tf.transform.translation.z = float(xyz[2])
        tf.transform.rotation.x = float(quat[0])
        tf.transform.rotation.y = float(quat[1])
        tf.transform.rotation.z = float(quat[2])
        tf.transform.rotation.w = float(quat[3])
        return tf

    def _publish_static_tf(self):
        if not self.enabled:
            return
        now = self.get_clock().now().to_msg()
        transforms = [
            self._tf(self.map_frame, self.odom_frame, now, (0.0, 0.0, 0.0)),
            self._tf(self.base_frame, self.camera_frame, now, self.camera_xyz),
            self._tf(self.base_frame, self.lidar_frame, now, self.lidar_xyz),
        ]
        self.static_broadcaster.sendTransform(transforms)
        self.static_tf_publish_count += 1

    def _publish_dynamic_tf(self):
        if not self.enabled:
            return
        now = self.get_clock().now().to_msg()
        tf = self._tf(
            self.odom_frame,
            self.base_frame,
            now,
            (self.pose_x, self.pose_y, self.pose_z),
            (self.pose_qx, self.pose_qy, self.pose_qz, self.pose_qw),
        )
        self.tf_broadcaster.sendTransform(tf)
        self.dynamic_tf_publish_count += 1

    def _last_odom_age(self):
        if self.last_odom_time is None:
            return -1.0
        return (self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9

    def _publish_status(self):
        msg = String()
        msg.data = (
            f'odom_received={str(self.odom_received).lower()} '
            f'last_odom_age_sec={self._last_odom_age():.3f} '
            f'current_pose=({self.pose_x:.3f},{self.pose_y:.3f},{self.pose_z:.3f},'
            f'{self.pose_qx:.3f},{self.pose_qy:.3f},{self.pose_qz:.3f},{self.pose_qw:.3f}) '
            f'dynamic_tf_publish_count={self.dynamic_tf_publish_count} '
            f'static_tf_publish_count={self.static_tf_publish_count} '
            f'map_frame={self.map_frame} '
            f'odom_frame={self.odom_frame} '
            f'base_frame={self.base_frame} '
            f'camera_frame={self.camera_frame} '
            f'lidar_frame={self.lidar_frame} '
            f'tf_guard_active={str(self.enabled).lower()}'
        )
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisualTfGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
