import math
import re

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class DemoExplainabilityOverlay(Node):
    def __init__(self):
        super().__init__('demo_explainability_overlay_node')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('scene_profile', 'realistic_room_corridor_v1')
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('panel_x', -9.6)
        self.declare_parameter('panel_y', -8.6)
        self.declare_parameter('panel_z', 4.2)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.scene_profile = str(self.get_parameter('scene_profile').value)
        self.current_mode = 'UNKNOWN'
        self.map_metrics = {}
        self.sensor_status = {}
        self.air_status = {}
        self.ground_status = {}
        self.executor_status = {}
        self.tf_status = {}
        self.active_goal = None
        self.active_path_len = 0.0
        self.active_path_count = 0
        self.executed_path_len = 0.0
        self.executed_path_count = 0
        self.last_odom = None

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        durable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(String, '/bimodal/active_mode', self._mode_cb, durable_qos)
        self.create_subscription(String, '/bimodal/map_metrics', lambda m: self._kv_cb('map', m), reliable_qos)
        self.create_subscription(String, '/bimodal/sensor_input_status', lambda m: self._kv_cb('sensor', m), reliable_qos)
        self.create_subscription(String, '/air/planner_status', lambda m: self._kv_cb('air', m), reliable_qos)
        self.create_subscription(String, '/ground/planner_status', lambda m: self._kv_cb('ground', m), reliable_qos)
        self.create_subscription(String, '/bimodal/fake_executor_status', lambda m: self._kv_cb('executor', m), reliable_qos)
        self.create_subscription(String, '/bimodal/tf_guard_status', lambda m: self._kv_cb('tf', m), reliable_qos)
        self.create_subscription(PoseStamped, '/bimodal/active_goal', self._goal_cb, reliable_qos)
        self.create_subscription(Path, '/bimodal/active_path', self._active_path_cb, reliable_qos)
        self.create_subscription(Path, '/bimodal/executed_path', self._executed_path_cb, reliable_qos)
        self.create_subscription(Odometry, '/bimodal/odom', self._odom_cb, reliable_qos)

        self.legend_pub = self.create_publisher(MarkerArray, '/bimodal/demo_legend_markers', durable_qos)
        self.state_pub = self.create_publisher(MarkerArray, '/bimodal/exploration_state_markers', reliable_qos)
        self.status_text_pub = self.create_publisher(MarkerArray, '/bimodal/demo_status_text', reliable_qos)
        self.status_string_pub = self.create_publisher(String, '/bimodal/demo_status_string', reliable_qos)
        self.selected_goal_pub = self.create_publisher(Marker, '/bimodal/selected_goal_marker', reliable_qos)
        self.rejected_pub = self.create_publisher(MarkerArray, '/bimodal/rejected_candidate_markers', reliable_qos)

        period = max(float(self.get_parameter('publish_period_sec').value), 0.2)
        self.timer = self.create_timer(period, self._publish)
        self.get_logger().info('P5B explainability overlay active: marker/status only, sim visualization, no control topics')

    def _mode_cb(self, msg):
        self.current_mode = msg.data.strip().upper() or 'UNKNOWN'

    def _kv_cb(self, source, msg):
        values = self._parse_key_values(msg.data)
        if source == 'map':
            self.map_metrics = values
        elif source == 'sensor':
            self.sensor_status = values
        elif source == 'air':
            self.air_status = values
        elif source == 'ground':
            self.ground_status = values
        elif source == 'executor':
            self.executor_status = values
        elif source == 'tf':
            self.tf_status = values

    def _goal_cb(self, msg):
        self.active_goal = msg

    def _active_path_cb(self, msg):
        self.active_path_count += 1
        self.active_path_len = self._path_length(msg)

    def _executed_path_cb(self, msg):
        self.executed_path_count += 1
        self.executed_path_len = self._path_length(msg)

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self.last_odom = (float(p.x), float(p.y), float(p.z))

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        self.legend_pub.publish(self._legend_markers(stamp))
        self.state_pub.publish(self._state_markers(stamp))
        self.status_text_pub.publish(self._status_panel(stamp))
        self.status_string_pub.publish(self._status_string())
        self.selected_goal_pub.publish(self._selected_goal_marker(stamp))
        self.rejected_pub.publish(self._empty_rejected_markers(stamp))

    def _legend_markers(self, stamp):
        markers = [self._clear_marker(stamp, 'p5b_demo_legend')]
        lines = [
            'P5B visual legend',
            'cyan points: incoming PointCloud2 /bimodal/points',
            'orange/green voxels: built 3D occupancy + coverage',
            'orange box: exploration boundary / known search volume',
            'blue/purple markers: Air candidates and selected Air goal',
            'teal/green markers: Ground 3D frontier candidates',
            'red sphere: selected bimodal active goal',
            'bright path: active path selected by mode mux',
            'white trail: executed fake path, robot marker follows it',
            'mode text: AIR/GROUND/IDLE from /bimodal/active_mode',
        ]
        for i, text in enumerate(lines):
            marker = self._text_marker(stamp, 'p5b_demo_legend', i + 1, text)
            marker.pose.position.x = -9.6
            marker.pose.position.y = 8.2
            marker.pose.position.z = 4.2 - i * 0.32
            marker.scale.z = 0.24 if i else 0.34
            marker.color.r, marker.color.g, marker.color.b = (0.95, 0.95, 0.95) if i else (1.0, 0.85, 0.25)
            markers.append(marker)
        return MarkerArray(markers=markers)

    def _state_markers(self, stamp):
        markers = [self._clear_marker(stamp, 'p5b_exploration_state')]
        markers.append(self._boundary_outline(stamp))
        markers.append(self._mode_beacon(stamp))
        if self.last_odom is not None:
            x, y, z = self.last_odom
            marker = self._text_marker(stamp, 'p5b_exploration_state', 20, f'robot pose ({x:.1f}, {y:.1f}, {z:.1f})')
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z + 1.5
            marker.scale.z = 0.26
            marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 1.0
            markers.append(marker)
        return MarkerArray(markers=markers)

    def _status_panel(self, stamp):
        markers = [self._clear_marker(stamp, 'p5b_demo_status_text')]
        text = self._panel_text()
        panel = self._text_marker(stamp, 'p5b_demo_status_text', 1, text)
        panel.pose.position.x = float(self.get_parameter('panel_x').value)
        panel.pose.position.y = float(self.get_parameter('panel_y').value)
        panel.pose.position.z = float(self.get_parameter('panel_z').value)
        panel.scale.z = 0.28
        panel.color.r, panel.color.g, panel.color.b = 0.95, 1.0, 0.95
        markers.append(panel)
        return MarkerArray(markers=markers)

    def _selected_goal_marker(self, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.map_frame
        marker.ns = 'p5b_selected_goal'
        marker.id = 1
        if self.active_goal is None:
            marker.action = Marker.DELETE
            return marker
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = self.active_goal.pose
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.48
        marker.color.r = 1.0
        marker.color.g = 0.08
        marker.color.b = 0.05
        marker.color.a = 0.95
        return marker

    def _empty_rejected_markers(self, stamp):
        return MarkerArray(markers=[self._clear_marker(stamp, 'p5b_rejected_candidates')])

    def _boundary_outline(self, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.map_frame
        marker.ns = 'p5b_exploration_state'
        marker.id = 2
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.5
        marker.pose.orientation.w = 1.0
        marker.scale.x = 20.0
        marker.scale.y = 20.0
        marker.scale.z = 3.0
        marker.color.r = 1.0
        marker.color.g = 0.62
        marker.color.b = 0.08
        marker.color.a = 0.06
        return marker

    def _mode_beacon(self, stamp):
        marker = self._text_marker(stamp, 'p5b_exploration_state', 3, f'ACTIVE MODE: {self.current_mode}')
        marker.pose.position.x = 0.0
        marker.pose.position.y = -9.0
        marker.pose.position.z = 4.3
        marker.scale.z = 0.46
        if self.current_mode == 'AIR':
            marker.color.r, marker.color.g, marker.color.b = 0.2, 0.55, 1.0
        elif self.current_mode == 'GROUND':
            marker.color.r, marker.color.g, marker.color.b = 0.1, 1.0, 0.35
        else:
            marker.color.r, marker.color.g, marker.color.b = 0.8, 0.8, 0.8
        return marker

    def _panel_text(self):
        goal_text = 'none'
        if self.active_goal is not None:
            p = self.active_goal.pose.position
            goal_text = f'({p.x:.2f},{p.y:.2f},{p.z:.2f})'
        return (
            f'P5B Bimodal 3D Exploration Demo\n'
            f'scene={self.scene_profile} backend=octomap_style_voxel\n'
            f'mode={self.current_mode} selected_goal={goal_text}\n'
            f'map voxels={self._get(self.map_metrics, "occupied_voxel_count")} '
            f'coverage={self._get(self.map_metrics, "coverage_proxy")} '
            f'input_clouds={self._get(self.map_metrics, "input_cloud_count")}\n'
            f'Air candidates={self._get(self.air_status, "air_candidate_count")} '
            f'selected={self._get(self.air_status, "air_selected_goal_count")} '
            f'repeat={self._get(self.air_status, "air_repeat_goal_ratio")}\n'
            f'Ground candidates={self._get(self.ground_status, "ground_candidate_count")} '
            f'selected={self._get(self.ground_status, "ground_selected_goal_count")} '
            f'3d_projection={self._get(self.ground_status, "ground_uses_3d_map_projection")}\n'
            f'active_path_len={self.active_path_len:.2f}m executed_len={self.executed_path_len:.2f}m '
            f'updates={self.active_path_count}/{self.executed_path_count}\n'
            f'TF guard active={self._get(self.tf_status, "tf_guard_active")} '
            f'odom_received={self._get(self.tf_status, "odom_received")}\n'
            f'no real control topics: cmd_vel/mavros/fmu/actuator/offboard/trajectory are not used'
        )

    def _status_string(self):
        msg = String()
        msg.data = (
            f'scene_profile={self.scene_profile} current_mode={self.current_mode} '
            f'selected_goal={self._goal_text()} active_path_length={self.active_path_len:.3f} '
            f'executed_path_length={self.executed_path_len:.3f} '
            f'occupied_voxel_count={self._get(self.map_metrics, "occupied_voxel_count")} '
            f'coverage_proxy={self._get(self.map_metrics, "coverage_proxy")} '
            f'air_candidate_count={self._get(self.air_status, "air_candidate_count")} '
            f'ground_candidate_count={self._get(self.ground_status, "ground_candidate_count")} '
            f'ground_uses_3d_map_projection={self._get(self.ground_status, "ground_uses_3d_map_projection")} '
            f'tf_guard_active={self._get(self.tf_status, "tf_guard_active")}'
        )
        return msg

    def _goal_text(self):
        if self.active_goal is None:
            return 'none'
        p = self.active_goal.pose.position
        return f'({p.x:.3f},{p.y:.3f},{p.z:.3f})'

    def _text_marker(self, stamp, ns, marker_id, text):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.map_frame
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.28
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
        marker.text = text
        return marker

    def _clear_marker(self, stamp, ns):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.map_frame
        marker.ns = ns
        marker.id = 0
        marker.action = Marker.DELETEALL
        return marker

    @staticmethod
    def _parse_key_values(text):
        values = {}
        for match in re.finditer(r'([A-Za-z0-9_]+)=([^ ]+)', text or ''):
            values[match.group(1)] = match.group(2).strip()
        return values

    @staticmethod
    def _get(values, key, default='UNKNOWN'):
        value = values.get(key, default)
        return value if value not in ('', 'None') else default

    @staticmethod
    def _path_length(path):
        if len(path.poses) < 2:
            return 0.0
        total = 0.0
        prev = path.poses[0].pose.position
        for pose in path.poses[1:]:
            p = pose.pose.position
            total += math.sqrt((p.x - prev.x) ** 2 + (p.y - prev.y) ** 2 + (p.z - prev.z) ** 2)
            prev = p
        return total


def main(args=None):
    rclpy.init(args=args)
    node = DemoExplainabilityOverlay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
