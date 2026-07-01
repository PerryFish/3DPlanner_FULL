import math

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray


class Fallback3DMapAdapter(Node):
    def __init__(self):
        super().__init__('fallback_3d_map_adapter_node')
        for name, value in [
            ('boundary_min_x', -10.0),
            ('boundary_max_x', 10.0),
            ('boundary_min_y', -10.0),
            ('boundary_max_y', 10.0),
            ('boundary_min_z', 0.0),
            ('boundary_max_z', 3.0),
            ('coverage_boundary_min_x', -10.0),
            ('coverage_boundary_max_x', 10.0),
            ('coverage_boundary_min_y', -10.0),
            ('coverage_boundary_max_y', 10.0),
            ('coverage_boundary_min_z', 0.0),
            ('coverage_boundary_max_z', 3.0),
        ]:
            self.declare_parameter(name, value)
        self.declare_parameter('voxel_resolution', 0.2)
        self.declare_parameter('voxel_size', 0.2)
        self.declare_parameter('accumulate_map', True)
        self.declare_parameter('max_accumulated_points', 200000)
        self.declare_parameter('map_publish_period_sec', 1.0)
        self.declare_parameter('min_input_points', 10)
        self.declare_parameter('coverage_resolution', 0.5)
        self.declare_parameter('status_publish_period_sec', 1.0)
        self.declare_parameter('max_coverage_marker_count', 3000)

        self.current_points = []
        self.accumulated_voxels = {}
        self.input_cloud_count = 0
        self.last_input_time = None
        self.last_map_time = None
        self.coverage_history = []
        self.create_subscription(PointCloud2, '/bimodal/points', self._points_cb, 10)
        self.create_subscription(Odometry, '/bimodal/odom', lambda msg: None, 10)
        self.map_pub = self.create_publisher(PointCloud2, '/bimodal/map_3d', 10)
        self.esdf_pub = self.create_publisher(PointCloud2, '/bimodal/esdf', 10)
        self.boundary_pub = self.create_publisher(MarkerArray, '/bimodal/exploration_boundary', 10)
        self.status_pub = self.create_publisher(String, '/bimodal/map_backend_status', 10)
        self.metrics_pub = self.create_publisher(String, '/bimodal/map_metrics', 10)
        self.coverage_marker_pub = self.create_publisher(MarkerArray, '/bimodal/coverage_markers', 10)
        self.map_status_marker_pub = self.create_publisher(Marker, '/bimodal/map_status_marker', 10)

        map_period = float(self.get_parameter('map_publish_period_sec').value)
        status_period = float(self.get_parameter('status_publish_period_sec').value)
        self.map_timer = self.create_timer(max(map_period, 0.1), self._publish_map_outputs)
        self.boundary_timer = self.create_timer(1.0, self._publish_boundary)
        self.status_timer = self.create_timer(max(status_period, 0.1), self._publish_status_and_metrics)
        self.get_logger().info(
            'fallback map adapter active: backend_mode=fallback_accumulated '
            f'accumulate_map={bool(self.get_parameter("accumulate_map").value)}'
        )

    def _points_cb(self, msg):
        pts = []
        voxel = self._voxel_size()
        seen = set()
        for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            if not all(math.isfinite(v) for v in (x, y, z)):
                continue
            key = self._voxel_key(x, y, z, voxel)
            if key in seen:
                continue
            seen.add(key)
            pts.append((x, y, z))

        min_input = int(self.get_parameter('min_input_points').value)
        if len(pts) < min_input:
            return
        self.input_cloud_count += 1
        self.last_input_time = self.get_clock().now()
        if bool(self.get_parameter('accumulate_map').value):
            max_points = int(self.get_parameter('max_accumulated_points').value)
            for x, y, z in pts:
                if len(self.accumulated_voxels) >= max_points:
                    break
                self.accumulated_voxels.setdefault(self._voxel_key(x, y, z, voxel), (x, y, z))
            self.current_points = list(self.accumulated_voxels.values())
        else:
            self.current_points = pts
        self._publish_map_outputs()

    def _publish_map_outputs(self):
        if not self.current_points:
            self._publish_boundary()
            self._publish_status_and_metrics()
            return
        now = self.get_clock().now().to_msg()
        header = Header(stamp=now, frame_id='map')
        self.map_pub.publish(point_cloud2.create_cloud_xyz32(header, self.current_points))
        stride = max(len(self.current_points) // 1200, 1)
        esdf_pts = [(x, y, min(z + 0.15, 3.0)) for x, y, z in self.current_points[::stride]]
        self.esdf_pub.publish(point_cloud2.create_cloud_xyz32(header, esdf_pts))
        self.last_map_time = self.get_clock().now()
        self._publish_boundary()
        self._publish_status_and_metrics()
        self._publish_coverage_markers()
        self._publish_map_status_marker()

    def _publish_boundary(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'bimodal_boundary'
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        min_x = float(self.get_parameter('boundary_min_x').value)
        max_x = float(self.get_parameter('boundary_max_x').value)
        min_y = float(self.get_parameter('boundary_min_y').value)
        max_y = float(self.get_parameter('boundary_max_y').value)
        min_z = float(self.get_parameter('boundary_min_z').value)
        max_z = float(self.get_parameter('boundary_max_z').value)
        marker.pose.position.x = (min_x + max_x) / 2.0
        marker.pose.position.y = (min_y + max_y) / 2.0
        marker.pose.position.z = (min_z + max_z) / 2.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = max_x - min_x
        marker.scale.y = max_y - min_y
        marker.scale.z = max_z - min_z
        marker.color.r = 0.1
        marker.color.g = 0.7
        marker.color.b = 1.0
        marker.color.a = 0.12
        self.boundary_pub.publish(MarkerArray(markers=[marker]))

    def _publish_status_and_metrics(self):
        metrics = self._metrics_string()
        status = String()
        status.data = (
            'backend_mode=fallback_accumulated '
            f'map_received={self.last_map_time is not None} '
            'esdf_is_fallback=True '
            f'point_count={len(self.current_points)} '
            f'last_map_age_sec={self._age(self.last_map_time):.3f} '
            f'{metrics}'
        )
        self.status_pub.publish(status)
        msg = String()
        msg.data = metrics
        self.metrics_pub.publish(msg)
        self._publish_map_status_marker()

    def _metrics_string(self):
        voxel_count = len(self.accumulated_voxels) if bool(self.get_parameter('accumulate_map').value) else len(self.current_points)
        point_count = len(self.current_points)
        boundary_volume = self._boundary_volume()
        coverage = self._coverage_proxy(voxel_count)
        now_sec = self.get_clock().now().nanoseconds / 1e9
        self.coverage_history.append((now_sec, coverage))
        self.coverage_history = [(t, c) for t, c in self.coverage_history if now_sec - t <= 12.0]
        old = self.coverage_history[0][1] if self.coverage_history else coverage
        coverage_delta = coverage - old
        return (
            f'accumulated_point_count={point_count} '
            f'accumulated_voxel_count={voxel_count} '
            f'coverage_proxy={coverage:.6f} '
            f'coverage_delta_last_10s={coverage_delta:.6f} '
            f'input_cloud_count={self.input_cloud_count} '
            f'last_input_age_sec={self._age(self.last_input_time):.3f} '
            'esdf_is_fallback=True '
            f'boundary_volume={boundary_volume:.3f} '
            'backend_mode=fallback_accumulated'
        )

    def _coverage_proxy(self, observed_voxels):
        res = float(self.get_parameter('coverage_resolution').value)
        min_x = float(self.get_parameter('coverage_boundary_min_x').value)
        max_x = float(self.get_parameter('coverage_boundary_max_x').value)
        min_y = float(self.get_parameter('coverage_boundary_min_y').value)
        max_y = float(self.get_parameter('coverage_boundary_max_y').value)
        min_z = float(self.get_parameter('coverage_boundary_min_z').value)
        max_z = float(self.get_parameter('coverage_boundary_max_z').value)
        nx = max(int((max_x - min_x) / max(res, 1e-6)), 1)
        ny = max(int((max_y - min_y) / max(res, 1e-6)), 1)
        nz = max(int((max_z - min_z) / max(res, 1e-6)), 1)
        return min(max(observed_voxels / float(nx * ny * nz), 0.0), 1.0)

    def _boundary_volume(self):
        return (
            (float(self.get_parameter('coverage_boundary_max_x').value) - float(self.get_parameter('coverage_boundary_min_x').value))
            * (float(self.get_parameter('coverage_boundary_max_y').value) - float(self.get_parameter('coverage_boundary_min_y').value))
            * (float(self.get_parameter('coverage_boundary_max_z').value) - float(self.get_parameter('coverage_boundary_min_z').value))
        )

    def _voxel_size(self):
        if self.has_parameter('voxel_size'):
            return float(self.get_parameter('voxel_size').value)
        return float(self.get_parameter('voxel_resolution').value)

    @staticmethod
    def _voxel_key(x, y, z, voxel):
        return (round(x / voxel), round(y / voxel), round(z / voxel))

    def _age(self, stamp):
        if stamp is None:
            return -1.0
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return age if math.isfinite(age) else -1.0

    def _publish_coverage_markers(self):
        now = self.get_clock().now().to_msg()
        markers = []
        clear = Marker()
        clear.header.stamp = now
        clear.header.frame_id = 'map'
        clear.ns = 'bimodal_coverage'
        clear.id = 0
        clear.action = Marker.DELETEALL
        markers.append(clear)

        observed = Marker()
        observed.header.stamp = now
        observed.header.frame_id = 'map'
        observed.ns = 'bimodal_coverage'
        observed.id = 1
        observed.type = Marker.CUBE_LIST
        observed.action = Marker.ADD
        scale = max(float(self.get_parameter('coverage_resolution').value), 0.05)
        observed.scale.x = observed.scale.y = observed.scale.z = min(scale, 0.35)
        observed.color.r = 0.1
        observed.color.g = 0.9
        observed.color.b = 0.35
        observed.color.a = 0.45

        pts = self.current_points
        max_count = int(self.get_parameter('max_coverage_marker_count').value)
        stride = max(len(pts) // max(max_count, 1), 1)
        for x, y, z in pts[::stride][:max_count]:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = float(z)
            observed.points.append(p)
        markers.append(observed)

        self.coverage_marker_pub.publish(MarkerArray(markers=markers))

    def _publish_map_status_marker(self):
        voxel_count = len(self.accumulated_voxels) if bool(self.get_parameter('accumulate_map').value) else len(self.current_points)
        coverage = self._coverage_proxy(voxel_count)
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'bimodal_map_status'
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.get_parameter('coverage_boundary_min_x').value)
        marker.pose.position.y = float(self.get_parameter('coverage_boundary_min_y').value)
        marker.pose.position.z = float(self.get_parameter('coverage_boundary_max_z').value) + 0.8
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.42
        marker.color.r = 0.95
        marker.color.g = 0.95
        marker.color.b = 0.95
        marker.color.a = 1.0
        marker.text = (
            'Map fallback accumulated\n'
            f'points={len(self.current_points)} voxels={voxel_count}\n'
            f'coverage={coverage:.3f} esdf_fallback=yes\n'
            f'input_clouds={self.input_cloud_count}'
        )
        self.map_status_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = Fallback3DMapAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
