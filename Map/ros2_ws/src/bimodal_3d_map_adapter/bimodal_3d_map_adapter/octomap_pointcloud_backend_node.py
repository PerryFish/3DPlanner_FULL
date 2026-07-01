import math
from collections import OrderedDict

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray


class OctomapPointcloudBackend(Node):
    def __init__(self):
        super().__init__('octomap_pointcloud_backend_node')
        for name, value in [
            ('backend_mode', 'octomap_style_voxel'),
            ('input_points_topic', '/bimodal/points'),
            ('odom_topic', '/bimodal/odom'),
            ('map_frame', 'map'),
            ('resolution', 0.2),
            ('max_occupied_voxels', 300000),
            ('publish_period_sec', 1.0),
            ('boundary_min_x', -10.0),
            ('boundary_max_x', 10.0),
            ('boundary_min_y', -10.0),
            ('boundary_max_y', 10.0),
            ('boundary_min_z', 0.0),
            ('boundary_max_z', 3.0),
            ('coverage_resolution', 0.5),
            ('esdf_is_fallback', True),
            ('marker_max_count', 5000),
        ]:
            self.declare_parameter(name, value)

        self.occupied = OrderedDict()
        self.input_cloud_count = 0
        self.last_input_time = None
        self.last_output_time = None
        self.last_odom = None
        self.coverage_history = []

        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('input_points_topic').value),
            self._points_cb,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_cb,
            10,
        )
        self.map_pub = self.create_publisher(PointCloud2, '/bimodal/map_3d', 10)
        self.esdf_pub = self.create_publisher(PointCloud2, '/bimodal/esdf', 10)
        self.boundary_pub = self.create_publisher(MarkerArray, '/bimodal/exploration_boundary', 10)
        self.status_pub = self.create_publisher(String, '/bimodal/map_backend_status', 10)
        self.metrics_pub = self.create_publisher(String, '/bimodal/map_metrics', 10)
        self.octomap_marker_pub = self.create_publisher(MarkerArray, '/bimodal/octomap_occupied_markers', 10)
        self.coverage_marker_pub = self.create_publisher(MarkerArray, '/bimodal/coverage_markers', 10)
        self.map_status_marker_pub = self.create_publisher(Marker, '/bimodal/map_status_marker', 10)

        period = max(float(self.get_parameter('publish_period_sec').value), 0.1)
        self.create_timer(period, self._publish_outputs)
        self.create_timer(1.0, self._publish_status_and_metrics)
        self.get_logger().info(
            'OctoMap-style PointCloud backend active: '
            f'backend_mode={self.get_parameter("backend_mode").value} '
            'is_real_octomap_server=false'
        )

    def _odom_cb(self, msg):
        self.last_odom = msg

    def _points_cb(self, msg):
        resolution = self._resolution()
        max_voxels = int(self.get_parameter('max_occupied_voxels').value)
        added = 0
        for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            if not self._inside_boundary(x, y, z):
                continue
            key = self._voxel_key(x, y, z, resolution)
            if key in self.occupied:
                continue
            if len(self.occupied) >= max_voxels:
                self.occupied.popitem(last=False)
            self.occupied[key] = self._voxel_center(key, resolution)
            added += 1
        self.input_cloud_count += 1
        self.last_input_time = self.get_clock().now()
        if added > 0:
            self._publish_outputs()

    def _publish_outputs(self):
        frame = str(self.get_parameter('map_frame').value)
        now = self.get_clock().now().to_msg()
        header = Header(stamp=now, frame_id=frame)
        points = list(self.occupied.values())
        self.map_pub.publish(point_cloud2.create_cloud_xyz32(header, points))
        self.esdf_pub.publish(point_cloud2.create_cloud_xyz32(header, self._esdf_points(points)))
        self.last_output_time = self.get_clock().now()
        self._publish_boundary(now, frame)
        self._publish_octomap_markers(now, frame, points)
        self._publish_coverage_markers(now, frame, points)
        self._publish_status_marker(now, frame)
        self._publish_status_and_metrics()

    def _esdf_points(self, points):
        if not points:
            return []
        stride = max(len(points) // 1500, 1)
        max_z = float(self.get_parameter('boundary_max_z').value)
        return [(x, y, min(z + self._resolution(), max_z)) for x, y, z in points[::stride]]

    def _publish_boundary(self, stamp, frame):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame
        marker.ns = 'bimodal_octomap_boundary'
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
        marker.color.r = 0.95
        marker.color.g = 0.55
        marker.color.b = 0.1
        marker.color.a = 0.12
        self.boundary_pub.publish(MarkerArray(markers=[marker]))

    def _publish_octomap_markers(self, stamp, frame, points):
        markers = [self._clear_marker(stamp, frame, 'bimodal_octomap_occupied')]
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame
        marker.ns = 'bimodal_octomap_occupied'
        marker.id = 1
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        scale = self._resolution()
        marker.scale.x = marker.scale.y = marker.scale.z = scale
        marker.color.r = 1.0
        marker.color.g = 0.55
        marker.color.b = 0.1
        marker.color.a = 0.45
        max_count = max(int(self.get_parameter('marker_max_count').value), 1)
        stride = max(len(points) // max_count, 1)
        for x, y, z in points[::stride][:max_count]:
            point = Point()
            point.x, point.y, point.z = float(x), float(y), float(z)
            marker.points.append(point)
        markers.append(marker)
        self.octomap_marker_pub.publish(MarkerArray(markers=markers))

    def _publish_coverage_markers(self, stamp, frame, points):
        markers = [self._clear_marker(stamp, frame, 'bimodal_coverage')]
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame
        marker.ns = 'bimodal_coverage'
        marker.id = 1
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        scale = max(float(self.get_parameter('coverage_resolution').value), 0.05)
        marker.scale.x = marker.scale.y = marker.scale.z = min(scale, 0.35)
        marker.color.r = 0.1
        marker.color.g = 0.85
        marker.color.b = 0.35
        marker.color.a = 0.42
        max_count = max(int(self.get_parameter('marker_max_count').value), 1)
        stride = max(len(points) // max_count, 1)
        for x, y, z in points[::stride][:max_count]:
            point = Point()
            point.x, point.y, point.z = float(x), float(y), float(z)
            marker.points.append(point)
        markers.append(marker)
        self.coverage_marker_pub.publish(MarkerArray(markers=markers))

    def _clear_marker(self, stamp, frame, ns):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame
        marker.ns = ns
        marker.id = 0
        marker.action = Marker.DELETEALL
        return marker

    def _publish_status_marker(self, stamp, frame):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame
        marker.ns = 'bimodal_map_status'
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.get_parameter('boundary_min_x').value)
        marker.pose.position.y = float(self.get_parameter('boundary_min_y').value)
        marker.pose.position.z = float(self.get_parameter('boundary_max_z').value) + 0.8
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.42
        marker.color.r = 0.98
        marker.color.g = 0.98
        marker.color.b = 0.98
        marker.color.a = 1.0
        marker.text = (
            'OctoMap-style PointCloud backend\n'
            f'voxels={len(self.occupied)} coverage={self._coverage_proxy():.3f}\n'
            f'input_clouds={self.input_cloud_count} real_octomap=no'
        )
        self.map_status_marker_pub.publish(marker)

    def _publish_status_and_metrics(self):
        metrics = self._metrics_string()
        input_received = self.last_input_time is not None
        output_published = self.last_output_time is not None
        status = String()
        status.data = (
            f'backend_mode={self.get_parameter("backend_mode").value} '
            f'input_received={str(input_received).lower()} '
            f'output_published={str(output_published).lower()} '
            f'occupied_voxel_count={len(self.occupied)} '
            f'coverage_proxy={self._coverage_proxy():.6f} '
            f'frame_id={self.get_parameter("map_frame").value} '
            f'last_input_age_sec={self._age(self.last_input_time):.3f} '
            'is_real_octomap_server=false '
            f'esdf_is_fallback={str(bool(self.get_parameter("esdf_is_fallback").value)).lower()} '
            f'{metrics}'
        )
        self.status_pub.publish(status)
        msg = String()
        msg.data = metrics
        self.metrics_pub.publish(msg)

    def _metrics_string(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        coverage = self._coverage_proxy()
        self.coverage_history.append((now_sec, coverage))
        self.coverage_history = [(t, c) for t, c in self.coverage_history if now_sec - t <= 12.0]
        old = self.coverage_history[0][1] if self.coverage_history else coverage
        return (
            f'backend_mode={self.get_parameter("backend_mode").value} '
            f'occupied_voxel_count={len(self.occupied)} '
            f'accumulated_voxel_count={len(self.occupied)} '
            f'map_point_count={len(self.occupied)} '
            f'accumulated_point_count={len(self.occupied)} '
            f'coverage_proxy={coverage:.6f} '
            f'coverage_delta_last_10s={coverage - old:.6f} '
            f'input_cloud_count={self.input_cloud_count} '
            f'last_input_age_sec={self._age(self.last_input_time):.3f} '
            f'resolution={self._resolution():.3f} '
            f'esdf_is_fallback={str(bool(self.get_parameter("esdf_is_fallback").value)).lower()} '
            'is_real_octomap_server=false'
        )

    def _coverage_proxy(self):
        res = max(float(self.get_parameter('coverage_resolution').value), 1e-6)
        min_x = float(self.get_parameter('boundary_min_x').value)
        max_x = float(self.get_parameter('boundary_max_x').value)
        min_y = float(self.get_parameter('boundary_min_y').value)
        max_y = float(self.get_parameter('boundary_max_y').value)
        min_z = float(self.get_parameter('boundary_min_z').value)
        max_z = float(self.get_parameter('boundary_max_z').value)
        nx = max(int((max_x - min_x) / res), 1)
        ny = max(int((max_y - min_y) / res), 1)
        nz = max(int((max_z - min_z) / res), 1)
        coarse = set()
        for x, y, z in self.occupied.values():
            coarse.add((round((x - min_x) / res), round((y - min_y) / res), round((z - min_z) / res)))
        return min(max(len(coarse) / float(nx * ny * nz), 0.0), 1.0)

    def _inside_boundary(self, x, y, z):
        if not all(math.isfinite(v) for v in (x, y, z)):
            return False
        return (
            float(self.get_parameter('boundary_min_x').value) <= x <= float(self.get_parameter('boundary_max_x').value)
            and float(self.get_parameter('boundary_min_y').value) <= y <= float(self.get_parameter('boundary_max_y').value)
            and float(self.get_parameter('boundary_min_z').value) <= z <= float(self.get_parameter('boundary_max_z').value)
        )

    def _resolution(self):
        return max(float(self.get_parameter('resolution').value), 1e-4)

    @staticmethod
    def _voxel_key(x, y, z, resolution):
        return (math.floor(x / resolution), math.floor(y / resolution), math.floor(z / resolution))

    @staticmethod
    def _voxel_center(key, resolution):
        return tuple((float(v) + 0.5) * resolution for v in key)

    def _age(self, stamp):
        if stamp is None:
            return -1.0
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return age if math.isfinite(age) else -1.0


def main(args=None):
    rclpy.init(args=args)
    node = OctomapPointcloudBackend()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
