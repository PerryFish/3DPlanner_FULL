import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class Ground3DFrontier(Node):
    def __init__(self):
        super().__init__('ground_3d_frontier_node')
        defaults = {
            'ground_z': 0.2, 'candidate_z': 0.2, 'min_goal_distance': 1.0, 'max_goal_distance': 8.0,
            'safety_radius': 0.4, 'boundary_min_x': -10.0, 'boundary_max_x': 10.0,
            'boundary_min_y': -10.0, 'boundary_max_y': 10.0, 'boundary_min_z': 0.0,
            'boundary_max_z': 3.0, 'candidate_grid_resolution': 1.0, 'update_period_sec': 5.0,
            'unknown_gain_weight': 1.0, 'distance_gain_weight': 0.25, 'obstacle_penalty_weight': 1.0,
            'revisit_penalty_weight': 0.8, 'max_candidate_count': 250, 'selected_goal_hold_sec': 5.0,
            'visual_exploration_mode': True, 'max_candidate_markers': 120,
            'min_goal_separation': 1.2, 'blacklist_radius': 1.0, 'blacklist_ttl_sec': 60.0,
            'frontier_bias_enabled': True, 'sector_balance_enabled': True, 'sector_count': 8,
            'frontier_gain_weight': 2.0, 'sector_balance_weight': 1.0,
            'coverage_gain_weight': 2.5, 'path_usefulness_weight': 1.5,
            'goal_switch_min_improvement': 0.4, 'path_collision_check_enabled': True,
            'low_gain_window_sec': 20.0, 'min_coverage_gain_per_goal': 0.004,
            'low_gain_blacklist_ttl_sec': 120.0, 'min_useful_path_length': 0.8,
            'recent_goal_history_size': 50,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.odom = None
        self.points = []
        self.map_voxels = set()
        self.last_coverage_proxy = 0.0
        self.esdf_received = False
        self.boundary_received = False
        self.visited = []
        self.last_goal = None
        self.last_goal_time = 0.0
        self.status = 'WAITING'
        self.last_candidate_count = 0
        self.last_valid_candidate_count = 0
        self.last_selected_score = 0.0
        self.last_rejected_collision = 0
        self.last_rejected_revisit = 0
        self.last_rejected_blacklist = 0
        self.last_rejected_low_gain = 0
        self.last_path_collision_risk = False
        self.last_marker_array = None
        self.blacklist = []
        self.sector_visit_counts = {}
        self.last_selected_sector = -1
        self.last_held_goal = False
        self.last_switched_goal = False
        self.held_goal_count = 0
        self.switched_goal_count = 0
        self.last_goal_score = 0.0
        self.low_gain_blacklist = []
        self.active_goal_monitor = None
        self.last_coverage_gain_estimate = 0.0
        self.last_frontier_ring_score = 0.0
        self.last_path_usefulness_score = 0.0
        self.last_oscillation_penalty = 0.0
        self.create_subscription(Odometry, '/bimodal/odom', self._odom_cb, 10)
        self.create_subscription(PointCloud2, '/bimodal/map_3d', self._map_cb, 10)
        self.create_subscription(PointCloud2, '/bimodal/esdf', lambda msg: setattr(self, 'esdf_received', True), 10)
        self.create_subscription(MarkerArray, '/bimodal/exploration_boundary', self._boundary_cb, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/ground/exploration_goal', 10)
        self.path_pub = self.create_publisher(Path, '/ground/path', 10)
        self.status_pub = self.create_publisher(String, '/ground/planner_status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/ground/frontier_candidates', 10)
        self.timer = self.create_timer(float(self.get_parameter('update_period_sec').value), self._plan)
        self.status_timer = self.create_timer(1.0, self._publish_status)

    def _odom_cb(self, msg):
        self.odom = msg

    def _map_cb(self, msg):
        self.points = [(float(p[0]), float(p[1]), float(p[2])) for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)]
        voxel = 0.5
        self.map_voxels = {(round(x / voxel), round(y / voxel), round(z / voxel)) for x, y, z in self.points}
        self.last_coverage_proxy = min(len(self.map_voxels) / 9600.0, 1.0)

    def _boundary_cb(self, msg):
        if not msg.markers:
            return
        m = msg.markers[0]
        self.set_parameters([])
        self.boundary = (
            m.pose.position.x - m.scale.x / 2.0, m.pose.position.x + m.scale.x / 2.0,
            m.pose.position.y - m.scale.y / 2.0, m.pose.position.y + m.scale.y / 2.0,
            m.pose.position.z - m.scale.z / 2.0, m.pose.position.z + m.scale.z / 2.0,
        )
        self.boundary_received = True

    def _bounds(self):
        if hasattr(self, 'boundary'):
            return self.boundary
        return tuple(float(self.get_parameter(k).value) for k in ['boundary_min_x', 'boundary_max_x', 'boundary_min_y', 'boundary_max_y', 'boundary_min_z', 'boundary_max_z'])

    def _robot_xy(self):
        p = self.odom.pose.pose.position
        return (float(p.x), float(p.y))

    def _clearance(self, x, y, z):
        nearest = 99.0
        ground_z = float(self.get_parameter('ground_z').value)
        for px, py, pz in self.points[:: max(len(self.points) // 1200, 1)]:
            if pz <= ground_z + 0.12:
                continue
            nearest = min(nearest, math.dist((x, y, z), (px, py, pz)))
        return nearest

    def _unknown_gain(self, x, y, z):
        near = 0
        ground_z = float(self.get_parameter('ground_z').value)
        for px, py, pz in self.points[:: max(len(self.points) // 1200, 1)]:
            if pz <= ground_z + 0.12:
                continue
            if math.dist((x, y, z), (px, py, pz)) < 1.2:
                near += 1
        return 1.0 / (1.0 + near)

    def _coverage_gain_estimate(self, x, y, z):
        radius = 2.0
        voxel = 0.5
        unknown = 0
        total = 0
        steps = range(-int(radius / voxel), int(radius / voxel) + 1)
        for ix in steps:
            for iy in steps:
                px = x + ix * voxel
                py = y + iy * voxel
                pz = z
                if math.hypot(px - x, py - y) > radius:
                    continue
                total += 1
                if (round(px / voxel), round(py / voxel), round(pz / voxel)) not in self.map_voxels:
                    unknown += 1
        return 0.0 if total <= 0 else unknown / total

    def _path_usefulness(self, x, y, z, d, collision_risk):
        coverage_gain = self._coverage_gain_estimate(x, y, z)
        short_low_gain_penalty = 0.6 if d < float(self.get_parameter('min_useful_path_length').value) and coverage_gain < 0.25 else 0.0
        collision_penalty = 0.5 if collision_risk else 0.0
        return max(0.0, min(1.0, coverage_gain + min(d / 5.0, 0.4) - short_low_gain_penalty - collision_penalty))

    def _oscillation_penalty(self, x, y):
        if len(self.visited) < 2:
            return 0.0
        rx, ry = self._robot_xy()
        last_x, last_y = self.visited[-1]
        prev_x, prev_y = self.visited[-2]
        vx1, vy1 = last_x - prev_x, last_y - prev_y
        vx2, vy2 = x - rx, y - ry
        n1 = math.hypot(vx1, vy1)
        n2 = math.hypot(vx2, vy2)
        if n1 < 1e-6 or n2 < 1e-6:
            return 0.0
        cosine = (vx1 * vx2 + vy1 * vy2) / (n1 * n2)
        return max(0.0, -cosine)

    def _frontier_gain(self, x, y, z):
        near = 0
        mid = 0
        ground_z = float(self.get_parameter('ground_z').value)
        for px, py, pz in self.points[:: max(len(self.points) // 1200, 1)]:
            if pz <= ground_z + 0.12:
                continue
            dist = math.dist((x, y, z), (px, py, pz))
            if dist < 1.4:
                near += 1
            elif dist < 2.6:
                mid += 1
        return max(0.0, min(min(mid / 10.0, 1.0) + 1.0 / (1.0 + near), 1.0))

    def _sector(self, x, y):
        rx, ry = self._robot_xy()
        angle = math.atan2(y - ry, x - rx)
        sectors = max(int(self.get_parameter('sector_count').value), 1)
        return int(((angle + math.pi) / (2.0 * math.pi)) * sectors) % sectors

    def _sector_gain(self, x, y):
        sector = self._sector(x, y)
        if not self.sector_visit_counts:
            return 1.0
        max_count = max(self.sector_visit_counts.values())
        return max(0.0, (max_count - self.sector_visit_counts.get(sector, 0) + 1.0) / (max_count + 1.0))

    def _prune_blacklist(self):
        ttl = float(self.get_parameter('blacklist_ttl_sec').value)
        now = time.time()
        self.blacklist = [(x, y, z, t) for x, y, z, t in self.blacklist if now - t < ttl]

    def _prune_low_gain_blacklist(self):
        ttl = float(self.get_parameter('low_gain_blacklist_ttl_sec').value)
        now = time.time()
        self.low_gain_blacklist = [(x, y, z, t) for x, y, z, t in self.low_gain_blacklist if now - t < ttl]

    def _add_blacklist(self, goal):
        if goal is None:
            return
        self.blacklist.append((float(goal[0]), float(goal[1]), float(goal[2]), time.time()))
        self.blacklist = self.blacklist[-80:]

    def _is_blacklisted(self, x, y, z):
        radius = float(self.get_parameter('blacklist_radius').value)
        return any(math.dist((x, y, z), (bx, by, bz)) < radius for bx, by, bz, _ in self.blacklist)

    def _is_low_gain_blacklisted(self, x, y, z):
        radius = float(self.get_parameter('blacklist_radius').value)
        return any(math.dist((x, y, z), (bx, by, bz)) < radius for bx, by, bz, _ in self.low_gain_blacklist)

    def _update_active_goal_monitor(self):
        if self.active_goal_monitor is None:
            return
        age = time.time() - self.active_goal_monitor['start_time']
        if age < float(self.get_parameter('low_gain_window_sec').value):
            return
        gain = self.last_coverage_proxy - float(self.active_goal_monitor['start_coverage'])
        if gain < float(self.get_parameter('min_coverage_gain_per_goal').value):
            gx, gy, gz = self.active_goal_monitor['goal']
            self.low_gain_blacklist.append((gx, gy, gz, time.time()))
            self.low_gain_blacklist = self.low_gain_blacklist[-80:]
        self.active_goal_monitor = None

    def _line_collision_risk(self, sx, sy, sz, gx, gy, gz):
        steps = 12
        safety = float(self.get_parameter('safety_radius').value)
        for i in range(steps + 1):
            t = i / steps
            x = sx + (gx - sx) * t
            y = sy + (gy - sy) * t
            z = sz + (gz - sz) * t
            if self._clearance(x, y, z) < safety:
                return True
        return False

    def _plan(self):
        self._prune_blacklist()
        self._prune_low_gain_blacklist()
        self._update_active_goal_monitor()
        self.last_held_goal = False
        self.last_switched_goal = False
        self.last_rejected_low_gain = 0
        if self.odom is None:
            self.status = 'WAITING_FOR_ODOM'
            self._publish_status()
            return
        if not self.points:
            self.status = 'WAITING_FOR_MAP'
            self._publish_status()
            return
        if self._should_hold_goal():
            self.status = 'HOLDING_SELECTED_GOAL'
            self.last_held_goal = True
            self.held_goal_count += 1
            self._publish_status()
            return
        min_x, max_x, min_y, max_y, _, _ = self._bounds()
        res = float(self.get_parameter('candidate_grid_resolution').value)
        candidate_z = float(self.get_parameter('candidate_z').value)
        safety = float(self.get_parameter('safety_radius').value)
        min_d = float(self.get_parameter('min_goal_distance').value)
        max_d = float(self.get_parameter('max_goal_distance').value)
        rx, ry = self._robot_xy()
        scored = []
        rejected = []
        rejected_collision = 0
        rejected_revisit = 0
        rejected_blacklist = 0
        max_count = int(self.get_parameter('max_candidate_count').value)
        x = min_x
        while x <= max_x and len(scored) + len(rejected) < max_count:
            y = min_y
            while y <= max_y and len(scored) + len(rejected) < max_count:
                d = math.hypot(x - rx, y - ry)
                clearance = self._clearance(x, y, candidate_z)
                if d < min_d or d > max_d or clearance < safety:
                    rejected_collision += 1
                    rejected.append((x, y, candidate_z, 'collision_or_distance'))
                    y += res
                    continue
                if self._is_blacklisted(x, y, candidate_z):
                    rejected_blacklist += 1
                    rejected.append((x, y, candidate_z, 'blacklist'))
                    y += res
                    continue
                if self._is_low_gain_blacklisted(x, y, candidate_z):
                    self.last_rejected_low_gain += 1
                    rejected.append((x, y, candidate_z, 'low_gain'))
                    y += res
                    continue
                unknown_gain = self._unknown_gain(x, y, candidate_z)
                frontier_gain = self._frontier_gain(x, y, candidate_z) if bool(self.get_parameter('frontier_bias_enabled').value) else 0.0
                coverage_gain = self._coverage_gain_estimate(x, y, candidate_z)
                sector_gain = self._sector_gain(x, y) if bool(self.get_parameter('sector_balance_enabled').value) else 0.0
                distance_gain = min(d / max(max_d, 0.1), 1.0)
                obstacle_penalty = 1.0 / max(clearance, 0.05)
                revisit_penalty = sum(max(0.0, 1.0 - math.hypot(x - vx, y - vy) / 1.5) for vx, vy in self.visited)
                oscillation_penalty = self._oscillation_penalty(x, y)
                if revisit_penalty > 0.85:
                    rejected_revisit += 1
                    rejected.append((x, y, candidate_z, 'revisit'))
                    y += res
                    continue
                collision_risk = bool(self.get_parameter('path_collision_check_enabled').value) and self._line_collision_risk(rx, ry, float(self.odom.pose.pose.position.z), x, y, candidate_z)
                path_usefulness = self._path_usefulness(x, y, candidate_z, d, collision_risk)
                if d < float(self.get_parameter('min_useful_path_length').value) and coverage_gain < 0.20:
                    rejected_collision += 1
                    rejected.append((x, y, candidate_z, 'low_gain_short_path'))
                    y += res
                    continue
                score = (
                    float(self.get_parameter('unknown_gain_weight').value) * unknown_gain
                    + float(self.get_parameter('frontier_gain_weight').value) * frontier_gain
                    + float(self.get_parameter('coverage_gain_weight').value) * coverage_gain
                    + float(self.get_parameter('path_usefulness_weight').value) * path_usefulness
                    + float(self.get_parameter('sector_balance_weight').value) * sector_gain
                    + float(self.get_parameter('distance_gain_weight').value) * distance_gain
                    - float(self.get_parameter('obstacle_penalty_weight').value) * obstacle_penalty
                    - float(self.get_parameter('revisit_penalty_weight').value) * revisit_penalty
                    - oscillation_penalty
                )
                if collision_risk:
                    score -= 2.0
                    rejected.append((x, y, candidate_z, 'path_collision_risk'))
                scored.append((score, x, y, candidate_z, clearance, revisit_penalty, frontier_gain, coverage_gain, path_usefulness, oscillation_penalty))
                y += res
            x += res
        if not scored:
            self.status = 'NO_VALID_GROUND_CANDIDATE'
            self.last_candidate_count = len(scored) + len(rejected)
            self.last_valid_candidate_count = 0
            self.last_rejected_collision = rejected_collision
            self.last_rejected_revisit = rejected_revisit
            self.last_rejected_blacklist = rejected_blacklist
            self._publish_markers([], rejected, None)
            self._publish_status()
            return
        scored.sort(reverse=True, key=lambda item: item[0])
        best = scored[0]
        _, gx, gy, gz, _, _, frontier_gain, coverage_gain, path_usefulness, oscillation_penalty = best
        if self._should_keep_current_goal(best[0]):
            self.status = 'HOLDING_CURRENT_GOAL_PROGRESS_GATE'
            self.last_held_goal = True
            self.held_goal_count += 1
            self._publish_markers(scored[:40], rejected[:40], None)
            self._publish_status()
            return
        if self.last_goal is not None:
            self._add_blacklist(self.last_goal)
        self.last_goal_score = float(best[0])
        self.last_frontier_ring_score = float(frontier_gain)
        self.last_coverage_gain_estimate = float(coverage_gain)
        self.last_path_usefulness_score = float(path_usefulness)
        self.last_oscillation_penalty = float(oscillation_penalty)
        self.last_switched_goal = self.last_goal is not None
        if self.last_switched_goal:
            self.switched_goal_count += 1
        self.last_goal = (gx, gy, gz)
        self.last_goal_time = time.time()
        self.active_goal_monitor = {
            'goal': self.last_goal,
            'start_time': time.time(),
            'start_coverage': self.last_coverage_proxy,
        }
        self.last_selected_sector = self._sector(gx, gy)
        self.sector_visit_counts[self.last_selected_sector] = self.sector_visit_counts.get(self.last_selected_sector, 0) + 1
        self.visited.append((gx, gy))
        self.visited = self.visited[-int(self.get_parameter('recent_goal_history_size').value):]
        self.last_candidate_count = len(scored) + len(rejected)
        self.last_valid_candidate_count = len(scored)
        self.last_selected_score = float(best[0])
        self.last_rejected_collision = rejected_collision
        self.last_rejected_revisit = rejected_revisit
        self.last_rejected_blacklist = rejected_blacklist
        self._publish_goal_and_path(gx, gy, gz)
        self._publish_markers(scored[:40], rejected[:40], best)
        self._publish_status()

    def _should_hold_goal(self):
        if self.last_goal is None or self.odom is None:
            return False
        age = time.time() - self.last_goal_time
        if age >= float(self.get_parameter('selected_goal_hold_sec').value):
            return False
        rx, ry = self._robot_xy()
        gx, gy, _ = self.last_goal
        if math.hypot(gx - rx, gy - ry) < float(self.get_parameter('min_goal_distance').value) * 0.5:
            return False
        return True

    def _should_keep_current_goal(self, best_score):
        if self.last_goal is None:
            return False
        improvement = float(best_score) - float(self.last_goal_score)
        return improvement < float(self.get_parameter('goal_switch_min_improvement').value)

    def _publish_goal_and_path(self, gx, gy, gz):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.position.z = gz
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        start = PoseStamped()
        start.header = goal.header
        start.pose = self.odom.pose.pose
        path = Path()
        path.header = goal.header
        path.poses = [start, goal]
        self.path_pub.publish(path)
        self.last_path_collision_risk = self._line_collision_risk(
            start.pose.position.x, start.pose.position.y, start.pose.position.z, gx, gy, gz
        )
        self.status = 'PATH_COLLISION_RISK' if self.last_path_collision_risk else 'OK'

    def _publish_markers(self, scored, rejected, selected):
        markers = []
        now = self.get_clock().now().to_msg()
        marker_id = 0
        max_markers = int(self.get_parameter('max_candidate_markers').value)
        for item in scored[:max_markers]:
            _, x, y, z, _, revisit, frontier_gain, coverage_gain, _, _ = item
            if frontier_gain > 0.75 or coverage_gain > 0.75:
                color = (0.0, 1.0, 0.35)
            else:
                color = (0.1, 0.8 if revisit < 0.1 else 0.4, 0.2)
            markers.append(self._marker(marker_id, x, y, z, color[0], color[1], color[2], 0.6, now))
            marker_id += 1
        for x, y, z, reason in rejected[:max(0, max_markers - len(scored[:max_markers]))]:
            if reason == 'revisit':
                color = (1.0, 0.55, 0.1)
            elif reason == 'blacklist':
                color = (0.8, 0.1, 0.9)
            elif reason == 'path_collision_risk':
                color = (0.9, 0.1, 0.9)
            elif reason == 'low_gain':
                color = (0.45, 0.0, 0.7)
            else:
                color = (0.9, 0.1, 0.1)
            markers.append(self._marker(marker_id, x, y, z, color[0], color[1], color[2], 0.35, now))
            marker_id += 1
        if selected:
            _, x, y, z, *_ = selected
            markers.append(self._marker(marker_id, x, y, z, 0.1, 0.2, 1.0, 1.0, now, scale=0.35))
        self.last_marker_array = MarkerArray(markers=markers)
        self.marker_pub.publish(self.last_marker_array)

    def _marker(self, marker_id, x, y, z, r, g, b, a, stamp, scale=0.18):
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = 'map'
        m.ns = 'ground_frontier_candidates'
        m.id = marker_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = a
        return m

    def _publish_status(self):
        msg = String()
        age = time.time() - self.last_goal_time if self.last_goal_time else -1.0
        msg.data = (
            f'{self.status} odom_received={self.odom is not None} map_received={bool(self.points)} '
            f'esdf_received={self.esdf_received} boundary_received={self.boundary_received} '
            f'candidate_count={self.last_candidate_count} valid_candidate_count={self.last_valid_candidate_count} '
            f'selected_score={self.last_selected_score:.3f} selected_goal={self.last_goal} '
            f'coverage_gain_estimate={self.last_coverage_gain_estimate:.3f} frontier_ring_score={self.last_frontier_ring_score:.3f} '
            f'path_usefulness_score={self.last_path_usefulness_score:.3f} oscillation_penalty={self.last_oscillation_penalty:.3f} '
            f'held_goal={str(self.last_held_goal).lower()} switched_goal={str(self.last_switched_goal).lower()} '
            f'blacklist_size={len(self.blacklist)} low_gain_blacklist_size={len(self.low_gain_blacklist)} selected_sector={self.last_selected_sector} '
            f'rejected_by_collision_count={self.last_rejected_collision} rejected_by_revisit_count={self.last_rejected_revisit} '
            f'rejected_by_blacklist_count={self.last_rejected_blacklist} rejected_by_low_gain_count={self.last_rejected_low_gain} '
            f'path_collision_risk={self.last_path_collision_risk} map_point_count={len(self.points)} '
            f'held_goal_count={self.held_goal_count} switched_goal_count={self.switched_goal_count} '
            f'planning_mode=P2D_ground_frontier_quality last_goal_age_sec={age:.2f}'
        )
        self.status_pub.publish(msg)
        if self.last_marker_array is not None:
            stamp = self.get_clock().now().to_msg()
            for marker in self.last_marker_array.markers:
                marker.header.stamp = stamp
            self.marker_pub.publish(self.last_marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = Ground3DFrontier()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
