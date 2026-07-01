import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class BimodalModeMux(Node):
    def __init__(self):
        super().__init__('bimodal_mode_mux_node')
        self.declare_parameter('republish_period_sec', 1.0)
        self.declare_parameter('valid_modes', ['AIR', 'GROUND', 'IDLE'])
        self.declare_parameter('active_path_min_pose_count', 2)

        self.valid_modes = [str(m).strip().upper() for m in self.get_parameter('valid_modes').value]
        self.min_pose_count = int(self.get_parameter('active_path_min_pose_count').value)
        self.current_mode = 'IDLE'
        self.last_air_goal = None
        self.last_air_path = None
        self.last_ground_goal = None
        self.last_ground_path = None
        self.last_active_goal = None
        self.last_active_path = None
        self.last_publish_time = None
        self.air_status = ''
        self.ground_status = ''

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        durable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(String, '/bimodal/active_mode', self._mode_cb, durable_qos)
        self.create_subscription(PoseStamped, '/air/exploration_goal', self._air_goal_cb, reliable_qos)
        self.create_subscription(Path, '/air/trajectory', self._air_path_cb, reliable_qos)
        self.create_subscription(String, '/air/planner_status', lambda m: self._store_status('air_status', m.data), reliable_qos)
        self.create_subscription(PoseStamped, '/ground/exploration_goal', self._ground_goal_cb, reliable_qos)
        self.create_subscription(Path, '/ground/path', self._ground_path_cb, reliable_qos)
        self.create_subscription(String, '/ground/planner_status', lambda m: self._store_status('ground_status', m.data), reliable_qos)
        self.goal_pub = self.create_publisher(PoseStamped, '/bimodal/active_goal', reliable_qos)
        self.path_pub = self.create_publisher(Path, '/bimodal/active_path', reliable_qos)
        self.status_pub = self.create_publisher(String, '/bimodal/mux_status', durable_qos)

        republish_period = float(self.get_parameter('republish_period_sec').value)
        self.timer = self.create_timer(max(republish_period, 0.1), self._tick)

    def _store_status(self, name, value):
        setattr(self, name, value)

    def _air_goal_cb(self, msg):
        self.last_air_goal = msg
        if self.current_mode == 'AIR':
            self._publish_active_goal(msg)

    def _air_path_cb(self, msg):
        self.last_air_path = msg
        if self.current_mode == 'AIR':
            self._publish_active_path(msg)

    def _ground_goal_cb(self, msg):
        self.last_ground_goal = msg
        if self.current_mode == 'GROUND':
            self._publish_active_goal(msg)

    def _ground_path_cb(self, msg):
        self.last_ground_path = msg
        if self.current_mode == 'GROUND':
            self._publish_active_path(msg)

    def _mode_cb(self, msg):
        self.current_mode = msg.data.strip().upper()
        self._publish_selected()

    def _tick(self):
        self._publish_selected()

    def _publish_selected(self):
        if self.current_mode == 'AIR':
            if self.last_air_goal is not None:
                self._publish_active_goal(self.last_air_goal)
            if self.last_air_path is not None:
                self._publish_active_path(self.last_air_path)
                self._publish_status('AIR')
            else:
                self._publish_status('WAITING_FOR_AIR_PATH')
        elif self.current_mode == 'GROUND':
            if self.last_ground_goal is not None:
                self._publish_active_goal(self.last_ground_goal)
            if self.last_ground_path is not None:
                self._publish_active_path(self.last_ground_path)
                self._publish_status('GROUND')
            else:
                self._publish_status('WAITING_FOR_GROUND_PATH')
        elif self.current_mode == 'IDLE':
            self._publish_status('IDLE')
        else:
            self._publish_status(f'INVALID_MODE mode={self.current_mode}')

    def _publish_active_goal(self, goal):
        goal.header.stamp = self.get_clock().now().to_msg()
        if not goal.header.frame_id:
            goal.header.frame_id = 'map'
        self.last_active_goal = goal
        self.goal_pub.publish(goal)

    def _publish_active_path(self, path):
        if len(path.poses) < self.min_pose_count:
            self._publish_status(f'WAITING_FOR_VALID_PATH pose_count={len(path.poses)}')
            return
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for pose in path.poses:
            pose.header.stamp = path.header.stamp
            if not pose.header.frame_id:
                pose.header.frame_id = 'map'
        self.last_active_path = path
        self.last_publish_time = self.get_clock().now()
        self.path_pub.publish(path)

    def _publish_status(self, state):
        status = String()
        if self.last_publish_time is None:
            age = -1.0
        else:
            age = (self.get_clock().now() - self.last_publish_time).nanoseconds / 1e9
        active_count = len(self.last_active_path.poses) if self.last_active_path is not None else 0
        status.data = (
            f'state={state} current_mode={self.current_mode} '
            f'has_air_path={self.last_air_path is not None} '
            f'has_ground_path={self.last_ground_path is not None} '
            f'active_path_pose_count={active_count} '
            f'last_publish_age_sec={age:.3f} '
            f'air_status={self.air_status} ground_status={self.ground_status}'
        )
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = BimodalModeMux()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
