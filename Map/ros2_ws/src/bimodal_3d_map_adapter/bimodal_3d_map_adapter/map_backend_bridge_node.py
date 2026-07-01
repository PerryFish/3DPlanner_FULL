import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class MapBackendBridge(Node):
    def __init__(self):
        super().__init__('map_backend_bridge')
        self.declare_parameter('backend_mode', 'fallback')
        self.declare_parameter('external_map_cloud_topic', '/external/map_3d')
        self.declare_parameter('external_esdf_cloud_topic', '/external/esdf')
        self.declare_parameter('external_boundary_topic', '/external/exploration_boundary')
        self.declare_parameter('nvblox_map_cloud_topic', '/nvblox_node/static_map_slice')
        self.declare_parameter('nvblox_esdf_topic', '/nvblox_node/esdf_pointcloud')
        self.declare_parameter('nvblox_mesh_topic', '/nvblox_node/mesh')
        self.declare_parameter('rtabmap_cloud_topic', '/rtabmap/cloud_map')
        self.declare_parameter('rtabmap_odom_topic', '/rtabmap/odom')
        self.declare_parameter('octomap_cloud_topic', '/octomap_point_cloud_centers')
        for name, value in [
            ('default_boundary_min_x', -10.0),
            ('default_boundary_max_x', 10.0),
            ('default_boundary_min_y', -10.0),
            ('default_boundary_max_y', 10.0),
            ('default_boundary_min_z', 0.0),
            ('default_boundary_max_z', 3.0),
        ]:
            self.declare_parameter(name, value)
        self.declare_parameter('status_publish_period_sec', 1.0)
        self.declare_parameter('max_map_age_sec', 3.0)
        self.declare_parameter('min_map_point_count', 10)
        self.declare_parameter('require_frame_id_map', True)

        self.backend_mode = str(self.get_parameter('backend_mode').value).strip()
        self.last_map_time = None
        self.last_esdf_time = None
        self.last_boundary_time = None
        self.map_received = False
        self.esdf_received = False
        self.boundary_received = False
        self.last_map_point_count = 0
        self.last_esdf_point_count = 0

        self.map_pub = self.create_publisher(PointCloud2, '/bimodal/map_3d', 10)
        self.esdf_pub = self.create_publisher(PointCloud2, '/bimodal/esdf', 10)
        self.boundary_pub = self.create_publisher(MarkerArray, '/bimodal/exploration_boundary', 10)
        self.status_pub = self.create_publisher(String, '/bimodal/map_backend_status', 10)

        if self.backend_mode == 'fallback':
            self.get_logger().info('backend_mode=fallback: bridge idle; use fallback_3d_map_adapter_node')
        else:
            map_topic, esdf_topic = self._select_cloud_topics()
            self.create_subscription(PointCloud2, map_topic, self._map_cb, 10)
            if esdf_topic:
                self.create_subscription(PointCloud2, esdf_topic, self._esdf_cb, 10)
            boundary_topic = str(self.get_parameter('external_boundary_topic').value)
            self.create_subscription(MarkerArray, boundary_topic, self._boundary_cb, 10)
            self.get_logger().info(
                f'map backend bridge active mode={self.backend_mode} map_topic={map_topic} esdf_topic={esdf_topic}'
            )

        period = float(self.get_parameter('status_publish_period_sec').value)
        self.status_timer = self.create_timer(max(period, 0.1), self._on_status_timer)

    def _select_cloud_topics(self):
        if self.backend_mode == 'nvblox_ready':
            return (
                str(self.get_parameter('nvblox_map_cloud_topic').value),
                str(self.get_parameter('nvblox_esdf_topic').value),
            )
        if self.backend_mode == 'rtabmap_ready':
            return str(self.get_parameter('rtabmap_cloud_topic').value), ''
        if self.backend_mode == 'octomap_ready':
            return str(self.get_parameter('octomap_cloud_topic').value), ''
        return (
            str(self.get_parameter('external_map_cloud_topic').value),
            str(self.get_parameter('external_esdf_cloud_topic').value),
        )

    def _map_cb(self, msg):
        self.last_map_point_count = self._point_count(msg)
        if not self._valid_cloud(msg, self.last_map_point_count):
            self._publish_status(extra_state='INVALID_EXTERNAL_MAP')
            return
        msg.header.stamp = self.get_clock().now().to_msg()
        if self._require_map_frame():
            msg.header.frame_id = 'map'
        self.last_map_time = self.get_clock().now()
        self.map_received = True
        self.map_pub.publish(msg)

    def _esdf_cb(self, msg):
        self.last_esdf_point_count = self._point_count(msg)
        if self.last_esdf_point_count <= 0:
            self._publish_status(extra_state='INVALID_EXTERNAL_ESDF')
            return
        msg.header.stamp = self.get_clock().now().to_msg()
        if self._require_map_frame():
            msg.header.frame_id = 'map'
        self.last_esdf_time = self.get_clock().now()
        self.esdf_received = True
        self.esdf_pub.publish(msg)

    def _boundary_cb(self, msg):
        self.last_boundary_time = self.get_clock().now()
        self.boundary_received = bool(msg.markers)
        if msg.markers:
            for marker in msg.markers:
                marker.header.stamp = self.get_clock().now().to_msg()
                if not marker.header.frame_id or self._require_map_frame():
                    marker.header.frame_id = 'map'
            self.boundary_pub.publish(msg)

    def _on_status_timer(self):
        if self.backend_mode != 'fallback' and not self.boundary_received:
            self._publish_default_boundary()
        self._publish_status()

    def _publish_status(self, extra_state='OK'):
        msg = String()
        state = extra_state
        if self.backend_mode == 'fallback':
            state = 'BRIDGE_IDLE_USE_FALLBACK'
        elif self.backend_mode == 'nvblox_ready' and not self.map_received:
            state = 'WAITING_FOR_NVBOX'
        elif self.backend_mode == 'rtabmap_ready' and not self.map_received:
            state = 'WAITING_FOR_RTABMAP'
        elif self.backend_mode == 'octomap_ready':
            state = 'OCTOMAP_POINTCLOUD_BRIDGE_PENDING' if not self.map_received else extra_state
        elif self.backend_mode == 'external_pointcloud' and not self.map_received:
            state = 'WAITING_FOR_EXTERNAL_MAP'
        if self.map_received and not self.esdf_received and self.backend_mode != 'octomap_ready':
            state = 'ESDF_MISSING'

        msg.data = (
            f'state={state} backend_mode={self.backend_mode} '
            f'map_received={self.map_received} esdf_received={self.esdf_received} '
            f'boundary_received={self.boundary_received} '
            f'last_map_age_sec={self._age(self.last_map_time):.3f} '
            f'last_esdf_age_sec={self._age(self.last_esdf_time):.3f} '
            f'map_point_count={self.last_map_point_count} '
            f'esdf_point_count={self.last_esdf_point_count}'
        )
        self.status_pub.publish(msg)

    def _publish_default_boundary(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'bimodal_backend_default_boundary'
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        min_x = float(self.get_parameter('default_boundary_min_x').value)
        max_x = float(self.get_parameter('default_boundary_max_x').value)
        min_y = float(self.get_parameter('default_boundary_min_y').value)
        max_y = float(self.get_parameter('default_boundary_max_y').value)
        min_z = float(self.get_parameter('default_boundary_min_z').value)
        max_z = float(self.get_parameter('default_boundary_max_z').value)
        marker.pose.position.x = (min_x + max_x) / 2.0
        marker.pose.position.y = (min_y + max_y) / 2.0
        marker.pose.position.z = (min_z + max_z) / 2.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = max_x - min_x
        marker.scale.y = max_y - min_y
        marker.scale.z = max_z - min_z
        marker.color.r = 0.3
        marker.color.g = 0.9
        marker.color.b = 0.5
        marker.color.a = 0.10
        self.boundary_pub.publish(MarkerArray(markers=[marker]))

    def _valid_cloud(self, msg, point_count):
        min_count = int(self.get_parameter('min_map_point_count').value)
        if point_count < min_count:
            return False
        if self._require_map_frame() and msg.header.frame_id and msg.header.frame_id != 'map':
            return False
        return True

    def _require_map_frame(self):
        return bool(self.get_parameter('require_frame_id_map').value)

    @staticmethod
    def _point_count(msg):
        return int(msg.width) * int(msg.height)

    def _age(self, stamp):
        if stamp is None:
            return -1.0
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return age if math.isfinite(age) else -1.0


def main(args=None):
    rclpy.init(args=args)
    node = MapBackendBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
