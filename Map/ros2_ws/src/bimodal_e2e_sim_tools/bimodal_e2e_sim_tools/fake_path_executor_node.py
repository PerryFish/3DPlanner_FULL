import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class FakePathExecutor(Node):
    def __init__(self):
        super().__init__('fake_path_executor_node')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z_air', 1.5)
        self.declare_parameter('initial_z_ground', 0.2)
        self.declare_parameter('air_speed_mps', 1.0)
        self.declare_parameter('ground_speed_mps', 0.4)
        self.declare_parameter('odom_publish_hz', 10.0)
        self.declare_parameter('goal_reached_radius', 0.3)
        self.declare_parameter('allow_path_z', True)
        self.declare_parameter('max_step_sec', 0.1)
        self.declare_parameter('stop_on_idle', True)
        self.declare_parameter('path_switch_hold_sec', 2.0)
        self.declare_parameter('ignore_tiny_path_changes', True)
        self.declare_parameter('path_endpoint_change_threshold', 0.5)
        self.declare_parameter('smooth_motion', True)
        self.declare_parameter('max_yaw_rate', 1.0)

        self.current_mode = 'AIR'
        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.z = float(self.get_parameter('initial_z_air').value)
        self.active_path = []
        self.target_index = 0
        self.last_path_time = None
        self.total_distance = 0.0
        self.reached_goal_count = 0
        self.idle_time_sec = 0.0
        self.executed_path = Path()
        self.executed_path.header.frame_id = 'map'
        self.accepted_path_update_count = 0
        self.ignored_path_update_count = 0
        self.last_path_accept_time = None
        self.current_path_endpoint = None

        self.create_subscription(String, '/bimodal/active_mode', self._mode_cb, 10)
        self.create_subscription(Path, '/bimodal/active_path', self._path_cb, 10)
        self.create_subscription(PoseStamped, '/bimodal/active_goal', lambda msg: None, 10)
        self.odom_pub = self.create_publisher(Odometry, '/bimodal/odom', 10)
        self.status_pub = self.create_publisher(String, '/bimodal/fake_executor_status', 10)
        self.executed_path_pub = self.create_publisher(Path, '/bimodal/executed_path', 10)
        self.executor_marker_pub = self.create_publisher(Marker, '/bimodal/executor_marker', 10)
        self.executor_status_marker_pub = self.create_publisher(Marker, '/bimodal/executor_status_marker', 10)

        hz = float(self.get_parameter('odom_publish_hz').value)
        self.last_tick_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / max(hz, 0.1), self._tick)
        self.get_logger().info('fake executor active: sim-only odom generator, no real control topics')

    def _mode_cb(self, msg):
        mode = msg.data.strip().upper()
        if mode in ('AIR', 'GROUND', 'IDLE'):
            self.current_mode = mode

    def _path_cb(self, msg):
        if len(msg.poses) < 2:
            return
        new_path = [(float(p.pose.position.x), float(p.pose.position.y), float(p.pose.position.z)) for p in msg.poses]
        now = self.get_clock().now()
        new_endpoint = new_path[-1]
        if self._should_ignore_path_update(now, new_endpoint):
            self.ignored_path_update_count += 1
            return
        self.active_path = new_path
        self.target_index = 1
        self.last_path_time = now
        self.last_path_accept_time = now
        self.current_path_endpoint = new_endpoint
        self.accepted_path_update_count += 1

    def _should_ignore_path_update(self, now, new_endpoint):
        if self.current_path_endpoint is None or self.last_path_accept_time is None:
            return False
        endpoint_delta = math.dist(self.current_path_endpoint, new_endpoint)
        threshold = float(self.get_parameter('path_endpoint_change_threshold').value)
        if bool(self.get_parameter('ignore_tiny_path_changes').value) and endpoint_delta < threshold:
            return True
        hold = float(self.get_parameter('path_switch_hold_sec').value)
        age = (now - self.last_path_accept_time).nanoseconds / 1e9
        return age < hold

    def _tick(self):
        now = self.get_clock().now()
        dt = min((now - self.last_tick_time).nanoseconds / 1e9, float(self.get_parameter('max_step_sec').value))
        self.last_tick_time = now
        if dt < 0.0 or not math.isfinite(dt):
            dt = 0.0
        self._move(dt)
        self._publish_odom(now)
        self._publish_executed_path(now)
        self._publish_status(now)
        self._publish_visual_markers(now)

    def _move(self, dt):
        if self.current_mode == 'IDLE' and bool(self.get_parameter('stop_on_idle').value):
            self.idle_time_sec += dt
            return
        if not self.active_path or self.target_index >= len(self.active_path):
            return
        tx, ty, tz = self.active_path[self.target_index]
        speed = float(self.get_parameter('ground_speed_mps').value)
        if self.current_mode == 'AIR':
            speed = float(self.get_parameter('air_speed_mps').value)
        dx = tx - self.x
        dy = ty - self.y
        target_z = tz if bool(self.get_parameter('allow_path_z').value) else self._mode_z()
        if self.current_mode == 'AIR':
            target_z = max(target_z, 1.0)
        elif self.current_mode == 'GROUND':
            target_z = float(self.get_parameter('initial_z_ground').value)
        dz = target_z - self.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist <= float(self.get_parameter('goal_reached_radius').value):
            self.reached_goal_count += 1
            self.target_index += 1
            return
        step = min(speed * dt, dist)
        if dist > 1e-6:
            self.x += dx / dist * step
            self.y += dy / dist * step
            self.z += dz / dist * step
            self.total_distance += step
        if self.current_mode == 'AIR':
            self.z = max(self.z, 1.0)
        elif self.current_mode == 'GROUND':
            self.z = float(self.get_parameter('initial_z_ground').value)

    def _mode_z(self):
        if self.current_mode == 'GROUND':
            return float(self.get_parameter('initial_z_ground').value)
        return float(self.get_parameter('initial_z_air').value)

    def _publish_odom(self, now):
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

    def _publish_executed_path(self, now):
        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = self.z
        pose.pose.orientation.w = 1.0
        if not self.executed_path.poses or self._distance_pose(self.executed_path.poses[-1], pose) > 0.05:
            self.executed_path.poses.append(pose)
            if len(self.executed_path.poses) > 5000:
                self.executed_path.poses = self.executed_path.poses[-5000:]
        self.executed_path.header.stamp = now.to_msg()
        self.executed_path_pub.publish(self.executed_path)

    def _publish_status(self, now):
        target = 'none'
        if self.active_path and self.target_index < len(self.active_path):
            tx, ty, tz = self.active_path[self.target_index]
            target = f'({tx:.3f},{ty:.3f},{tz:.3f})'
        msg = String()
        msg.data = (
            f'current_mode={self.current_mode} '
            f'accepted_path_update_count={self.accepted_path_update_count} '
            f'ignored_path_update_count={self.ignored_path_update_count} '
            f'path_switch_hold_active={str(self._path_hold_active(now)).lower()} '
            f'current_path_endpoint={self._endpoint_text()} '
            f'current_position=({self.x:.3f},{self.y:.3f},{self.z:.3f}) '
            f'current_target={target} '
            f'active_path_pose_count={len(self.active_path)} '
            f'total_distance={self.total_distance:.3f} '
            f'reached_goal_count={self.reached_goal_count} '
            f'idle_time_sec={self.idle_time_sec:.3f} '
            f'last_path_age_sec={self._age(now):.3f} '
            'executor_is_sim_only=true'
        )
        self.status_pub.publish(msg)

    def _publish_visual_markers(self, now):
        marker = Marker()
        marker.header.stamp = now.to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'bimodal_executor'
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = self.z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.75
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        if self.current_mode == 'AIR':
            marker.color.r, marker.color.g, marker.color.b = 0.1, 0.45, 1.0
        elif self.current_mode == 'GROUND':
            marker.color.r, marker.color.g, marker.color.b = 0.1, 0.9, 0.25
        else:
            marker.color.r, marker.color.g, marker.color.b = 0.7, 0.7, 0.7
        marker.color.a = 0.95
        self.executor_marker_pub.publish(marker)

        status = Marker()
        status.header.stamp = now.to_msg()
        status.header.frame_id = 'map'
        status.ns = 'bimodal_executor_status'
        status.id = 1
        status.type = Marker.TEXT_VIEW_FACING
        status.action = Marker.ADD
        status.pose.position.x = self.x
        status.pose.position.y = self.y
        status.pose.position.z = self.z + 1.1
        status.pose.orientation.w = 1.0
        status.scale.z = 0.32
        status.color.r = 1.0
        status.color.g = 1.0
        status.color.b = 1.0
        status.color.a = 1.0
        target = 'none'
        if self.active_path and self.target_index < len(self.active_path):
            tx, ty, tz = self.active_path[self.target_index]
            target = f'({tx:.1f},{ty:.1f},{tz:.1f})'
        status.text = (
            f'mode={self.current_mode}\n'
            f'distance={self.total_distance:.2f}m reached={self.reached_goal_count}\n'
            f'path_poses={len(self.active_path)} target={target}\n'
            f'accepted={self.accepted_path_update_count} ignored={self.ignored_path_update_count}\n'
            'sim_only=true'
        )
        self.executor_status_marker_pub.publish(status)

    def _age(self, now):
        if self.last_path_time is None:
            return -1.0
        return (now - self.last_path_time).nanoseconds / 1e9

    def _endpoint_text(self):
        if self.current_path_endpoint is None:
            return 'none'
        x, y, z = self.current_path_endpoint
        return f'({x:.3f},{y:.3f},{z:.3f})'

    def _path_hold_active(self, now):
        if self.last_path_accept_time is None:
            return False
        return (now - self.last_path_accept_time).nanoseconds / 1e9 < float(self.get_parameter('path_switch_hold_sec').value)

    @staticmethod
    def _distance_pose(a, b):
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        dz = a.pose.position.z - b.pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)


def main(args=None):
    rclpy.init(args=args)
    node = FakePathExecutor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
