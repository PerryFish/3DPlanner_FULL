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
            'planner_mode': 'stub', 'enable_stub_fallback': True,
            'octomap_adaptive_scoring': True, 'unknown_boundary_weight': 2.0,
            'occupied_penalty_weight': 4.0, 'path_collision_penalty_weight': 2.5,
            'oscillation_penalty_weight': 1.5,
            'ground_quality_profile': 'p4a_baseline',
            'ground_candidate_radii': [1.5, 2.5, 4.0, 6.0],
            'ground_candidate_yaw_samples': 24,
            'ground_base_height_m': 0.2,
            'ground_max_step_height_m': 0.35,
            'ground_min_candidate_separation_m': 0.45,
            'ground_max_candidate_count': 384,
            'ground_sector_balance_enable': True,
            'ground_expand_radius_on_low_gain': True,
            'ground_local_escape_enable': True,
            'ground_goal_min_hold_sec': 3.0,
            'ground_goal_max_hold_sec': 32.0,
            'ground_low_gain_retire_sec': 18.0,
            'ground_no_progress_timeout_sec': 22.0,
            'ground_blacklist_radius_m': 1.0,
            'ground_blacklist_duration_sec': 90.0,
            'ground_stale_path_timeout_sec': 35.0,
            'ground_sector_escape_enable': True,
            'novelty_gain_weight': 1.2,
            'clearance_gain_weight': 1.0,
            'path_length_penalty_weight': 0.45,
            'stale_region_penalty_weight': 1.0,
            'near_obstacle_penalty_weight': 1.2,
            'local_escape_gain_weight': 0.8,
            'ground_min_clearance_m': 0.42,
            'ground_endpoint_to_goal_max_m': 0.5,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.odom = None
        self.points = []
        self.map_voxels = set()
        self.map_metrics = {}
        self.map_backend_status = {}
        self.backend_mode = 'unknown'
        self.map_resolution = 0.5
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
        self.last_octomap_frontier_gain = 0.0
        self.last_unknown_boundary_gain = 0.0
        self.last_occupied_penalty = 0.0
        self.last_rejected_occupied = 0
        self.selected_goal_count = 0
        self.path_feasible_count = 0
        self.path_infeasible_count = 0
        self.path_length_sum = 0.0
        self.endpoint_error_sum = 0.0
        self.fallback_active = False
        self.failure_reason = 'NONE'
        self.tick_count = 0
        self.last_candidate_sector_count = 0
        self.goal_retire_count = 0
        self.low_gain_retire_count = 0
        self.stale_goal_retire_count = 0
        self.traversability_checked_count = 0
        self.traversability_reject_count = 0
        self.clearance_reject_count = 0
        self.step_height_reject_count = 0
        self.support_reject_count = 0
        self.unreachable_reject_count = 0
        self.collision_reject_count = 0
        self.boundary_reject_count = 0
        self.height_reject_count = 0
        self.last_traversability_score = 0.0
        self.last_clearance_score = 0.0
        self.last_novelty_gain = 0.0
        self.last_local_escape_gain = 0.0
        self.last_selected_terms = {}
        self.last_score_terms = {
            'frontier_max': 0.0, 'frontier_avg': 0.0,
            'unknown_max': 0.0, 'unknown_avg': 0.0,
            'coverage_max': 0.0, 'coverage_avg': 0.0,
            'traversability_max': 0.0, 'traversability_avg': 0.0,
            'clearance_max': 0.0, 'clearance_avg': 0.0,
            'novelty_max': 0.0, 'novelty_avg': 0.0,
        }
        self.low_gain_since = None
        self.last_goal_retire_reason = 'NONE'
        self.last_selected_goal_age_max_sec = 0.0
        self.last_stale_path_sec = 0.0
        self.last_path_signature = None
        self.last_path_signature_time = 0.0
        self.sector_escape_offset = 0
        self.boundary = (
            float(defaults['boundary_min_x']), float(defaults['boundary_max_x']),
            float(defaults['boundary_min_y']), float(defaults['boundary_max_y']),
            float(defaults['boundary_min_z']), float(defaults['boundary_max_z']),
        )
        self.create_subscription(Odometry, '/bimodal/odom', self._odom_cb, 10)
        self.create_subscription(PointCloud2, '/bimodal/map_3d', self._map_cb, 10)
        self.create_subscription(String, '/bimodal/map_metrics', self._map_metrics_cb, 10)
        self.create_subscription(String, '/bimodal/map_backend_status', self._map_backend_status_cb, 10)
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
        voxel = self.map_resolution
        self.map_voxels = {(round(x / voxel), round(y / voxel), round(z / voxel)) for x, y, z in self.points}
        self.last_coverage_proxy = min(len(self.map_voxels) / 9600.0, 1.0)

    def _map_metrics_cb(self, msg):
        self.map_metrics = self._parse_key_values(msg.data)
        self.backend_mode = self.map_metrics.get('backend_mode', self.backend_mode)
        self.map_resolution = self._float(self.map_metrics.get('resolution'), self.map_resolution)
        self.last_coverage_proxy = self._float(self.map_metrics.get('coverage_proxy'), self.last_coverage_proxy)

    def _map_backend_status_cb(self, msg):
        self.map_backend_status = self._parse_key_values(msg.data)
        self.backend_mode = self.map_backend_status.get('backend_mode', self.backend_mode)

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
        voxel = max(self.map_resolution, 0.2)
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

    def _octomap_active(self):
        return bool(self.get_parameter('octomap_adaptive_scoring').value) and self.backend_mode == 'octomap_style_voxel'

    def _planner_mode(self):
        mode = str(self.get_parameter('planner_mode').value).strip()
        return mode if mode in ('stub', 'ground_3d_frontier_v0', 'external_wrapper_placeholder') else 'stub'

    def _wrapper_active(self):
        return self._planner_mode() == 'ground_3d_frontier_v0'

    def _ground_quality_profile(self):
        return str(self.get_parameter('ground_quality_profile').value).strip()

    def _p4b_optimized(self):
        return self._wrapper_active() and self._ground_quality_profile() == 'p4b_optimized'

    def _occupied_penalty(self, x, y, z, clearance):
        safety = float(self.get_parameter('safety_radius').value)
        near = 0
        ground_z = float(self.get_parameter('ground_z').value)
        for px, py, pz in self.points[:: max(len(self.points) // 1500, 1)]:
            if pz <= ground_z + 0.12:
                continue
            if math.dist((x, y, z), (px, py, pz)) < safety * 1.8:
                near += 1
        density_penalty = min(near / 10.0, 1.0)
        clearance_penalty = max(0.0, 1.0 - clearance / max(safety, 0.05))
        return max(density_penalty, clearance_penalty)

    def _unknown_boundary_gain(self, x, y, z):
        near = 0
        ring = 0
        ground_z = float(self.get_parameter('ground_z').value)
        for px, py, pz in self.points[:: max(len(self.points) // 1500, 1)]:
            if pz <= ground_z + 0.12:
                continue
            dist = math.dist((x, y, z), (px, py, pz))
            if dist < 1.0:
                near += 1
            elif dist < 2.8:
                ring += 1
        local_edge = min(ring / 14.0, 1.0) * (1.0 / (1.0 + near / 4.0))
        global_unknown = self._float(self.map_metrics.get('unknown_boundary_proxy'), 0.0)
        global_frontier = self._float(self.map_metrics.get('frontier_candidate_proxy'), 0.0)
        return max(0.0, min(0.60 * local_edge + 0.25 * global_unknown + 0.15 * global_frontier, 1.0))

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

    def _candidate_points_p4b(self):
        min_x, max_x, min_y, max_y, _, _ = self._bounds()
        rx, ry = self._robot_xy()
        min_d = float(self.get_parameter('min_goal_distance').value)
        max_d = float(self.get_parameter('max_goal_distance').value)
        z = float(self.get_parameter('ground_base_height_m').value)
        radii = [self._clamp(float(r), min_d, max_d) for r in self._float_list_param('ground_candidate_radii', [1.5, 2.5, 4.0, 6.0])]
        if bool(self.get_parameter('ground_expand_radius_on_low_gain').value) and (self.last_valid_candidate_count < 12 or self.low_gain_since is not None):
            radii.append(max_d)
        radii = sorted({round(r, 2) for r in radii if min_d <= r <= max_d})
        if not radii:
            radii = [self._clamp((min_d + max_d) * 0.5, min_d, max_d)]
        yaw_samples = max(int(self.get_parameter('ground_candidate_yaw_samples').value), 8)
        max_count = max(int(self.get_parameter('ground_max_candidate_count').value), int(self.get_parameter('max_candidate_count').value))
        min_sep = max(float(self.get_parameter('ground_min_candidate_separation_m').value), 0.05)
        offset = (self.tick_count * 0.29 + self.sector_escape_offset * (2.0 * math.pi / yaw_samples)) % (2.0 * math.pi)
        pts = []
        seen = set()
        for r_idx, radius in enumerate(radii):
            yaw_order = list(range(yaw_samples))
            if bool(self.get_parameter('ground_sector_escape_enable').value) and self.sector_visit_counts:
                sectors = max(int(self.get_parameter('sector_count').value), 1)
                yaw_order.sort(key=lambda i: self.sector_visit_counts.get(i % sectors, 0))
            for yaw_i in yaw_order:
                angle = offset + (2.0 * math.pi * yaw_i / yaw_samples) + r_idx * 0.09
                x = self._clamp(rx + radius * math.cos(angle), min_x, max_x)
                y = self._clamp(ry + radius * math.sin(angle), min_y, max_y)
                if math.hypot(x - rx, y - ry) < min_d * 0.9:
                    continue
                key = (round(x / min_sep), round(y / min_sep), round(z / min_sep))
                if key in seen:
                    continue
                seen.add(key)
                pts.append((x, y, z))
                if len(pts) >= max_count:
                    self.last_candidate_sector_count = min(yaw_samples, max(int(self.get_parameter('sector_count').value), 1))
                    return pts
        if bool(self.get_parameter('ground_local_escape_enable').value) and self.last_goal is not None:
            back_angle = math.atan2(ry - self.last_goal[1], rx - self.last_goal[0])
            for delta in (-0.8, 0.0, 0.8):
                x = self._clamp(rx + min(max_d, 3.0) * math.cos(back_angle + delta), min_x, max_x)
                y = self._clamp(ry + min(max_d, 3.0) * math.sin(back_angle + delta), min_y, max_y)
                key = (round(x / min_sep), round(y / min_sep), round(z / min_sep))
                if key not in seen:
                    seen.add(key)
                    pts.append((x, y, z))
        self.last_candidate_sector_count = min(yaw_samples, max(int(self.get_parameter('sector_count').value), 1))
        return pts

    def _within_boundary(self, candidate):
        x, y, z = candidate
        min_x, max_x, min_y, max_y, min_z, max_z = self._bounds()
        return min_x <= x <= max_x and min_y <= y <= max_y and min_z <= z <= max_z

    def _traversability(self, candidate, clearance):
        self.traversability_checked_count += 1
        x, y, z = candidate
        ground_z = float(self.get_parameter('ground_z').value)
        max_step = float(self.get_parameter('ground_max_step_height_m').value)
        low_hits = 0
        obstacle_hits = 0
        for px, py, pz in self.points[:: max(len(self.points) // 1500, 1)]:
            horizontal = math.hypot(x - px, y - py)
            if horizontal > max(float(self.get_parameter('safety_radius').value) * 1.4, 0.45):
                continue
            if pz <= ground_z + max_step:
                low_hits += 1
            elif pz <= z + 1.2:
                obstacle_hits += 1
        if low_hits > 4:
            self.step_height_reject_count += 1
            self.traversability_reject_count += 1
            return False, 0.0, 'step_height_reject'
        min_clearance = max(float(self.get_parameter('safety_radius').value), float(self.get_parameter('ground_min_clearance_m').value))
        if clearance < min_clearance:
            self.clearance_reject_count += 1
            self.traversability_reject_count += 1
            return False, 0.0, 'clearance_reject'
        density_penalty = min(obstacle_hits / 8.0, 1.0)
        clearance_score = min(clearance / max(min_clearance * 2.5, 0.1), 1.0)
        score = max(0.0, min(1.0, 0.65 * clearance_score + 0.35 * (1.0 - density_penalty)))
        return True, score, 'OK'

    def _novelty_gain(self, candidate):
        x, y, _ = candidate
        if not self.visited:
            return 1.0
        nearest = min(math.hypot(x - vx, y - vy) for vx, vy in self.visited[-20:])
        return max(0.0, min(nearest / max(float(self.get_parameter('min_goal_separation').value) * 2.0, 0.1), 1.0))

    def _stale_region_penalty(self, candidate):
        if self.last_goal is None:
            return 0.0
        age = time.time() - self.last_goal_time
        if age < float(self.get_parameter('ground_stale_path_timeout_sec').value):
            return 0.0
        return max(0.0, 1.0 - math.dist(candidate, self.last_goal) / max(float(self.get_parameter('ground_blacklist_radius_m').value) * 3.0, 0.1))

    def _local_escape_gain(self, candidate):
        if not bool(self.get_parameter('ground_local_escape_enable').value) or len(self.visited) < 2:
            return 0.0
        rx, ry = self._robot_xy()
        last_x, last_y = self.visited[-1]
        prev_x, prev_y = self.visited[-2]
        away_x, away_y = last_x - prev_x, last_y - prev_y
        cand_x, cand_y = candidate[0] - rx, candidate[1] - ry
        n1 = math.hypot(away_x, away_y)
        n2 = math.hypot(cand_x, cand_y)
        if n1 < 1e-6 or n2 < 1e-6:
            return 0.0
        cosine = (away_x * cand_x + away_y * cand_y) / (n1 * n2)
        return max(0.0, -cosine)

    def _prune_blacklist(self):
        ttl = float(self.get_parameter('ground_blacklist_duration_sec').value) if self._p4b_optimized() else float(self.get_parameter('blacklist_ttl_sec').value)
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
        radius = float(self.get_parameter('ground_blacklist_radius_m').value) if self._p4b_optimized() else float(self.get_parameter('blacklist_radius').value)
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
            if self.low_gain_since is None:
                self.low_gain_since = time.time()
            self.last_goal_retire_reason = 'LOW_GAIN_MONITOR'
        else:
            self.low_gain_since = None
        self.active_goal_monitor = None

    def _line_collision_risk(self, sx, sy, sz, gx, gy, gz):
        steps = max(int(math.dist((sx, sy, sz), (gx, gy, gz)) / max(self.map_resolution, 0.2)), 8)
        safety = max(float(self.get_parameter('safety_radius').value), float(self.get_parameter('ground_min_clearance_m').value) if self._p4b_optimized() else 0.0)
        for i in range(steps + 1):
            t = i / steps
            x = sx + (gx - sx) * t
            y = sy + (gy - sy) * t
            z = sz + (gz - sz) * t
            if self._clearance(x, y, z) < safety:
                return True
        return False

    def _detour_midpoint(self, start, goal):
        sx, sy, sz = start
        gx, gy, gz = goal
        dx, dy = gx - sx, gy - sy
        n = max(math.hypot(dx, dy), 1e-6)
        side = 1.0 if (self.tick_count + self.sector_escape_offset) % 2 == 0 else -1.0
        offset = min(max(n * 0.35, 0.5), 1.4) * side
        return ((sx + gx) * 0.5 - dy / n * offset, (sy + gy) * 0.5 + dx / n * offset, gz)

    def _detour_path_feasible(self, start, goal):
        mid = self._detour_midpoint(start, goal)
        return self._within_boundary(mid) and not self._line_collision_risk(*start, *mid) and not self._line_collision_risk(*mid, *goal)

    def _path_feasibility(self, candidate):
        rx, ry = self._robot_xy()
        rz = float(self.odom.pose.pose.position.z)
        gx, gy, gz = candidate
        path_length = math.dist((rx, ry, rz), candidate)
        endpoint_error = 0.0
        if not self._within_boundary(candidate):
            self.boundary_reject_count += 1
            return False, path_length, endpoint_error, 'boundary_reject'
        if abs(gz - float(self.get_parameter('ground_base_height_m').value)) > max(float(self.get_parameter('ground_max_step_height_m').value), 0.1):
            self.height_reject_count += 1
            return False, path_length, endpoint_error, 'height_reject'
        if self._line_collision_risk(rx, ry, rz, gx, gy, gz):
            if self._detour_path_feasible((rx, ry, rz), candidate):
                mid = self._detour_midpoint((rx, ry, rz), candidate)
                path_length = math.dist((rx, ry, rz), mid) + math.dist(mid, candidate)
                return True, path_length, endpoint_error, 'OK_DETOUR'
            self.collision_reject_count += 1
            self.unreachable_reject_count += 1
            return False, path_length, endpoint_error, 'path_collision'
        if endpoint_error > float(self.get_parameter('ground_endpoint_to_goal_max_m').value):
            self.boundary_reject_count += 1
            return False, path_length, endpoint_error, 'endpoint_reject'
        return True, path_length, endpoint_error, 'OK'

    def _update_score_term_stats(self, scored):
        if not scored:
            self.last_score_terms = {k: 0.0 for k in self.last_score_terms}
            return
        frontier = [float(item[6]) for item in scored]
        coverage = [float(item[7]) for item in scored]
        unknown = [float(item[10]) for item in scored]
        traversability = [float(item[13].get('traversability_score', 0.0)) for item in scored]
        clearance = [float(item[13].get('clearance_score', 0.0)) for item in scored]
        novelty = [float(item[13].get('novelty_gain', 0.0)) for item in scored]
        self.last_score_terms = {
            'frontier_max': max(frontier), 'frontier_avg': sum(frontier) / len(frontier),
            'unknown_max': max(unknown), 'unknown_avg': sum(unknown) / len(unknown),
            'coverage_max': max(coverage), 'coverage_avg': sum(coverage) / len(coverage),
            'traversability_max': max(traversability), 'traversability_avg': sum(traversability) / len(traversability),
            'clearance_max': max(clearance), 'clearance_avg': sum(clearance) / len(clearance),
            'novelty_max': max(novelty), 'novelty_avg': sum(novelty) / len(novelty),
        }

    def _should_force_retire_current_goal(self):
        if not self._p4b_optimized() or self.last_goal is None:
            return False
        age = time.time() - self.last_goal_time
        self.last_selected_goal_age_max_sec = max(self.last_selected_goal_age_max_sec, age)
        if age > float(self.get_parameter('ground_goal_max_hold_sec').value):
            self.last_goal_retire_reason = 'MAX_HOLD_TIMEOUT'
            return True
        if self.low_gain_since is not None and time.time() - self.low_gain_since > float(self.get_parameter('ground_low_gain_retire_sec').value):
            self.last_goal_retire_reason = 'LOW_GAIN_RETIRE'
            return True
        if self.active_goal_monitor is not None:
            rx, ry = self._robot_xy()
            dist = math.dist((rx, ry, float(self.odom.pose.pose.position.z)), self.last_goal)
            progress = max(0.0, float(self.active_goal_monitor.get('start_distance', 0.0)) - dist)
            if time.time() - float(self.active_goal_monitor['start_time']) > float(self.get_parameter('ground_no_progress_timeout_sec').value) and progress < 0.2:
                self.last_goal_retire_reason = 'NO_PROGRESS_TIMEOUT'
                return True
        if self.last_path_signature is not None and time.time() - self.last_path_signature_time > float(self.get_parameter('ground_stale_path_timeout_sec').value):
            self.last_goal_retire_reason = 'STALE_PATH_TIMEOUT'
            return True
        return False

    def _retire_current_goal(self, reason):
        if self.last_goal is None:
            return
        self.goal_retire_count += 1
        if reason == 'LOW_GAIN_RETIRE':
            self.low_gain_retire_count += 1
            gx, gy, gz = self.last_goal
            self.low_gain_blacklist.append((gx, gy, gz, time.time()))
            self.low_gain_blacklist = self.low_gain_blacklist[-80:]
        elif reason in ('STALE_PATH_TIMEOUT', 'NO_PROGRESS_TIMEOUT', 'MAX_HOLD_TIMEOUT'):
            self.stale_goal_retire_count += 1
        self._add_blacklist(self.last_goal)
        self.sector_escape_offset += 1
        self.failure_reason = reason
        self.last_goal = None
        self.last_goal_time = 0.0
        self.active_goal_monitor = None
        self.low_gain_since = None

    def _plan(self):
        self.tick_count += 1
        self._prune_blacklist()
        self._prune_low_gain_blacklist()
        self._update_active_goal_monitor()
        self.last_held_goal = False
        self.last_switched_goal = False
        self.last_rejected_low_gain = 0
        self.last_rejected_occupied = 0
        self.fallback_active = False
        self.failure_reason = 'NONE'
        if self.odom is None:
            self.status = 'WAITING_FOR_ODOM'
            self.failure_reason = 'WAITING_FOR_ODOM'
            self._publish_status()
            return
        if not self.points:
            self.status = 'WAITING_FOR_MAP'
            self.failure_reason = 'WAITING_FOR_MAP'
            self._publish_status()
            return
        if self.last_goal is not None and self._should_force_retire_current_goal():
            self._retire_current_goal(self.last_goal_retire_reason)
        if self._should_hold_goal():
            self.status = 'HOLDING_SELECTED_GOAL'
            self.last_held_goal = True
            self.held_goal_count += 1
            self._publish_status()
            return

        min_x, max_x, min_y, max_y, _, _ = self._bounds()
        res = float(self.get_parameter('candidate_grid_resolution').value)
        candidate_z = float(self.get_parameter('ground_base_height_m').value) if self._p4b_optimized() else float(self.get_parameter('candidate_z').value)
        safety = max(float(self.get_parameter('safety_radius').value), float(self.get_parameter('ground_min_clearance_m').value) if self._p4b_optimized() else 0.0)
        min_d = float(self.get_parameter('min_goal_distance').value)
        max_d = float(self.get_parameter('max_goal_distance').value)
        rx, ry = self._robot_xy()
        scored = []
        rejected = []
        rejected_collision = 0
        rejected_revisit = 0
        rejected_blacklist = 0
        candidates = self._candidate_points_p4b() if self._p4b_optimized() else []
        if not candidates:
            max_count = int(self.get_parameter('max_candidate_count').value)
            x = min_x
            while x <= max_x and len(candidates) < max_count:
                y = min_y
                while y <= max_y and len(candidates) < max_count:
                    candidates.append((x, y, candidate_z))
                    y += res
                x += res

        for x, y, candidate_z in candidates:
            d = math.hypot(x - rx, y - ry)
            clearance = self._clearance(x, y, candidate_z)
            if d < min_d or d > max_d:
                rejected_collision += 1
                self.boundary_reject_count += 1
                rejected.append((x, y, candidate_z, 'distance_reject'))
                continue
            if self._p4b_optimized():
                ok, traversability_score, traversability_reason = self._traversability((x, y, candidate_z), clearance)
                if not ok:
                    rejected.append((x, y, candidate_z, traversability_reason))
                    continue
            else:
                traversability_score = 1.0
            if clearance < safety:
                rejected_collision += 1
                self.clearance_reject_count += 1
                rejected.append((x, y, candidate_z, 'clearance_reject'))
                continue
            if self._is_blacklisted(x, y, candidate_z):
                rejected_blacklist += 1
                rejected.append((x, y, candidate_z, 'blacklist'))
                continue
            if self._is_low_gain_blacklisted(x, y, candidate_z):
                self.last_rejected_low_gain += 1
                rejected.append((x, y, candidate_z, 'low_gain'))
                continue
            unknown_gain = self._unknown_gain(x, y, candidate_z)
            frontier_gain = self._frontier_gain(x, y, candidate_z) if bool(self.get_parameter('frontier_bias_enabled').value) else 0.0
            unknown_boundary_gain = self._unknown_boundary_gain(x, y, candidate_z) if self._octomap_active() else 0.0
            octomap_frontier_gain = max(frontier_gain, unknown_boundary_gain * 0.7) if self._octomap_active() else frontier_gain
            coverage_gain = self._coverage_gain_estimate(x, y, candidate_z)
            sector_param = 'ground_sector_balance_enable' if self._p4b_optimized() else 'sector_balance_enabled'
            sector_gain = self._sector_gain(x, y) if bool(self.get_parameter(sector_param).value) else 0.0
            distance_gain = min(d / max(max_d, 0.1), 1.0)
            obstacle_penalty = 1.0 / max(clearance, 0.05)
            occupied_penalty = self._occupied_penalty(x, y, candidate_z, clearance) if self._octomap_active() else obstacle_penalty
            if self._octomap_active() and occupied_penalty > 0.88:
                self.last_rejected_occupied += 1
                self.collision_reject_count += 1
                rejected.append((x, y, candidate_z, 'rejected_occupied'))
                continue
            revisit_penalty = sum(max(0.0, 1.0 - math.hypot(x - vx, y - vy) / 1.5) for vx, vy in self.visited)
            oscillation_penalty = self._oscillation_penalty(x, y)
            if revisit_penalty > 0.85:
                rejected_revisit += 1
                rejected.append((x, y, candidate_z, 'revisit'))
                continue
            collision_risk = False
            path_length = math.dist((rx, ry, float(self.odom.pose.pose.position.z)), (x, y, candidate_z))
            endpoint_error = 0.0
            if self._wrapper_active():
                feasible, path_length, endpoint_error, reason = self._path_feasibility((x, y, candidate_z))
                if not feasible:
                    self.path_infeasible_count += 1
                    rejected.append((x, y, candidate_z, reason))
                    continue
            path_usefulness = self._path_usefulness(x, y, candidate_z, d, collision_risk)
            if d < float(self.get_parameter('min_useful_path_length').value) and coverage_gain < 0.20:
                rejected_collision += 1
                rejected.append((x, y, candidate_z, 'low_gain_short_path'))
                continue
            novelty_gain = self._novelty_gain((x, y, candidate_z)) if self._p4b_optimized() else 0.0
            clearance_score = min(clearance / max(safety * 2.5, 0.1), 1.0) if self._p4b_optimized() else 0.0
            stale_penalty = self._stale_region_penalty((x, y, candidate_z)) if self._p4b_optimized() else 0.0
            local_escape_gain = self._local_escape_gain((x, y, candidate_z)) if self._p4b_optimized() else 0.0
            path_length_cost = min(path_length / max(max_d, 0.1), 1.5)
            score = (
                float(self.get_parameter('unknown_gain_weight').value) * unknown_gain
                + float(self.get_parameter('frontier_gain_weight').value) * octomap_frontier_gain
                + float(self.get_parameter('unknown_boundary_weight').value) * unknown_boundary_gain
                + float(self.get_parameter('coverage_gain_weight').value) * coverage_gain
                + float(self.get_parameter('path_usefulness_weight').value) * path_usefulness
                + float(self.get_parameter('sector_balance_weight').value) * sector_gain
                + float(self.get_parameter('distance_gain_weight').value) * distance_gain
                + float(self.get_parameter('coverage_gain_weight').value) * 0.25 * traversability_score
                + float(self.get_parameter('novelty_gain_weight').value) * novelty_gain
                + float(self.get_parameter('clearance_gain_weight').value) * clearance_score
                + float(self.get_parameter('local_escape_gain_weight').value) * local_escape_gain
                - float(self.get_parameter('occupied_penalty_weight' if self._octomap_active() else 'obstacle_penalty_weight').value) * occupied_penalty
                - float(self.get_parameter('path_collision_penalty_weight').value) * (1.0 if collision_risk else 0.0)
                - float(self.get_parameter('revisit_penalty_weight').value) * revisit_penalty
                - float(self.get_parameter('oscillation_penalty_weight').value) * oscillation_penalty
                - float(self.get_parameter('stale_region_penalty_weight').value) * stale_penalty
                - float(self.get_parameter('near_obstacle_penalty_weight').value) * occupied_penalty
                - float(self.get_parameter('path_length_penalty_weight').value) * path_length_cost
            )
            terms = {
                'unknown_gain': unknown_gain, 'frontier_gain': octomap_frontier_gain,
                'coverage_gain': coverage_gain, 'traversability_score': traversability_score,
                'clearance_score': clearance_score, 'novelty_gain': novelty_gain,
                'sector_gain': sector_gain, 'local_escape_gain': local_escape_gain,
                'distance_cost': distance_gain, 'path_length_cost': path_length_cost,
                'repeat_penalty': revisit_penalty, 'stale_penalty': stale_penalty,
                'near_obstacle_penalty': occupied_penalty,
            }
            if self._wrapper_active():
                self.path_feasible_count += 1
                self.path_length_sum += path_length
                self.endpoint_error_sum += endpoint_error
            scored.append((score, x, y, candidate_z, clearance, revisit_penalty, octomap_frontier_gain, coverage_gain, path_usefulness, oscillation_penalty, unknown_boundary_gain, occupied_penalty, collision_risk, terms))

        self._update_score_term_stats(scored)
        if not scored:
            self.status = 'NO_VALID_GROUND_CANDIDATE'
            self.failure_reason = 'NO_VALID_WRAPPER_CANDIDATE' if self._wrapper_active() else 'NO_VALID_GROUND_CANDIDATE'
            self.fallback_active = False
            self.last_candidate_count = len(candidates)
            self.last_valid_candidate_count = 0
            self.last_rejected_collision = rejected_collision
            self.last_rejected_revisit = rejected_revisit
            self.last_rejected_blacklist = rejected_blacklist
            self._publish_markers([], rejected, None)
            self._publish_status()
            return
        scored.sort(reverse=True, key=lambda item: item[0])
        best = scored[0]
        _, gx, gy, gz, _, _, frontier_gain, coverage_gain, path_usefulness, oscillation_penalty, unknown_boundary_gain, occupied_penalty, collision_risk, terms = best
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
        self.last_octomap_frontier_gain = float(frontier_gain)
        self.last_coverage_gain_estimate = float(coverage_gain)
        self.last_path_usefulness_score = float(path_usefulness)
        self.last_oscillation_penalty = float(oscillation_penalty)
        self.last_unknown_boundary_gain = float(unknown_boundary_gain)
        self.last_occupied_penalty = float(occupied_penalty)
        self.last_path_collision_risk = bool(collision_risk)
        self.last_traversability_score = float(terms.get('traversability_score', 0.0))
        self.last_clearance_score = float(terms.get('clearance_score', 0.0))
        self.last_novelty_gain = float(terms.get('novelty_gain', 0.0))
        self.last_local_escape_gain = float(terms.get('local_escape_gain', 0.0))
        self.last_selected_terms = {k: float(v) for k, v in terms.items()}
        self.last_switched_goal = self.last_goal is not None
        if self.last_switched_goal:
            self.switched_goal_count += 1
        self.last_goal = (gx, gy, gz)
        self.last_goal_time = time.time()
        self.selected_goal_count += 1
        self.active_goal_monitor = {
            'goal': self.last_goal,
            'start_time': time.time(),
            'start_coverage': self.last_coverage_proxy,
            'start_distance': math.dist((rx, ry, float(self.odom.pose.pose.position.z)), self.last_goal),
        }
        self.last_selected_sector = self._sector(gx, gy)
        self.sector_visit_counts[self.last_selected_sector] = self.sector_visit_counts.get(self.last_selected_sector, 0) + 1
        self.visited.append((gx, gy))
        self.visited = self.visited[-int(self.get_parameter('recent_goal_history_size').value):]
        self.last_candidate_count = len(candidates)
        self.last_valid_candidate_count = len(scored)
        self.last_selected_score = float(best[0])
        self.last_rejected_collision = rejected_collision
        self.last_rejected_revisit = rejected_revisit
        self.last_rejected_blacklist = rejected_blacklist
        self.failure_reason = 'NONE'
        self._publish_goal_and_path(gx, gy, gz)
        self._publish_markers(scored[:40], rejected[:40], best)
        self._publish_status()

    def _should_hold_goal(self):
        if self.last_goal is None or self.odom is None:
            return False
        age = time.time() - self.last_goal_time
        hold_sec = float(self.get_parameter('ground_goal_min_hold_sec').value) if self._p4b_optimized() else float(self.get_parameter('selected_goal_hold_sec').value)
        if age >= hold_sec:
            return False
        rx, ry = self._robot_xy()
        gx, gy, _ = self.last_goal
        if math.hypot(gx - rx, gy - ry) < float(self.get_parameter('min_goal_distance').value) * 0.5:
            return False
        return True

    def _should_keep_current_goal(self, best_score):
        if self.last_goal is None:
            return False
        if self._p4b_optimized() and time.time() - self.last_goal_time > float(self.get_parameter('ground_goal_max_hold_sec').value):
            return False
        if self._p4b_optimized() and self.low_gain_since is not None:
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
        signature = (round(start.pose.position.x, 1), round(start.pose.position.y, 1), round(gx, 1), round(gy, 1), round(gz, 1))
        if signature != self.last_path_signature:
            self.last_path_signature = signature
            self.last_path_signature_time = time.time()
            self.last_stale_path_sec = 0.0
        else:
            self.last_stale_path_sec = time.time() - self.last_path_signature_time
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
            _, x, y, z, _, revisit, frontier_gain, coverage_gain, _, _, unknown_boundary_gain, occupied_penalty, collision_risk, _ = item
            if occupied_penalty > 0.75 or collision_risk:
                color = (1.0, 0.1, 0.1)
            elif unknown_boundary_gain > 0.65:
                color = (0.0, 0.95, 1.0)
            elif frontier_gain > 0.75 or coverage_gain > 0.75:
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
            elif reason == 'rejected_occupied':
                color = (1.0, 0.0, 0.0)
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
        octomap_active = self._octomap_active()
        planner_mode = self._planner_mode()
        planning_mode = 'P4B_ground_3d_quality_optimized' if self._p4b_optimized() else ('P4A_ground_3d_frontier_v0' if planner_mode == 'ground_3d_frontier_v0' else ('P2E_ground_octomap_frontier_quality' if octomap_active else 'P2D_ground_frontier_quality'))
        path_length_avg = self.path_length_sum / max(self.path_feasible_count, 1)
        endpoint_error_avg = self.endpoint_error_sum / max(self.path_feasible_count, 1)
        selected_terms = ','.join(f'{k}:{v:.3f}' for k, v in sorted(self.last_selected_terms.items())) if self.last_selected_terms else 'NONE'
        repeat_ratio = 0.0
        if self.visited:
            keys = {(round(x), round(y)) for x, y in self.visited}
            repeat_ratio = max(0.0, 1.0 - len(keys) / max(len(self.visited), 1))
        msg.data = (
            f'{self.status} odom_received={self.odom is not None} map_received={bool(self.points)} '
            f'planner_mode={planner_mode} ground_planner_mode={planner_mode} '
            f'ground_quality_profile={self._ground_quality_profile()} '
            f'fallback_active={str(self.fallback_active).lower()} failure_reason={self.failure_reason} '
            f'esdf_received={self.esdf_received} boundary_received={self.boundary_received} '
            f'candidate_count={self.last_candidate_count} valid_candidate_count={self.last_valid_candidate_count} '
            f'selected_score={self.last_selected_score:.3f} selected_goal={self.last_goal} '
            f'selected_score_terms={selected_terms} '
            f'coverage_gain_estimate={self.last_coverage_gain_estimate:.3f} frontier_ring_score={self.last_frontier_ring_score:.3f} '
            f'octomap_frontier_gain={self.last_octomap_frontier_gain:.3f} unknown_boundary_gain={self.last_unknown_boundary_gain:.3f} '
            f'occupied_penalty={self.last_occupied_penalty:.3f} '
            f'traversability_score={self.last_traversability_score:.3f} clearance_score={self.last_clearance_score:.3f} '
            f'novelty_gain={self.last_novelty_gain:.3f} local_escape_gain={self.last_local_escape_gain:.3f} '
            f'path_usefulness_score={self.last_path_usefulness_score:.3f} oscillation_penalty={self.last_oscillation_penalty:.3f} '
            f'held_goal={str(self.last_held_goal).lower()} switched_goal={str(self.last_switched_goal).lower()} '
            f'blacklist_size={len(self.blacklist)} low_gain_blacklist_size={len(self.low_gain_blacklist)} selected_sector={self.last_selected_sector} '
            f'rejected_by_collision_count={self.last_rejected_collision} rejected_by_revisit_count={self.last_rejected_revisit} '
            f'rejected_by_occupied_count={self.last_rejected_occupied} '
            f'rejected_by_blacklist_count={self.last_rejected_blacklist} rejected_by_low_gain_count={self.last_rejected_low_gain} '
            f'path_collision_risk={self.last_path_collision_risk} map_point_count={len(self.points)} '
            f'held_goal_count={self.held_goal_count} switched_goal_count={self.switched_goal_count} '
            f'ground_candidate_count={self.last_candidate_count} ground_valid_candidate_count={self.last_valid_candidate_count} '
            f'ground_candidate_sector_count={self.last_candidate_sector_count} '
            f'ground_traversability_checked_count={self.traversability_checked_count} '
            f'ground_traversability_reject_count={self.traversability_reject_count} '
            f'ground_clearance_reject_count={self.clearance_reject_count} '
            f'ground_step_height_reject_count={self.step_height_reject_count} '
            f'ground_support_reject_count={self.support_reject_count} '
            f'ground_unreachable_reject_count={self.unreachable_reject_count} '
            f'ground_selected_goal_count={self.selected_goal_count} ground_path_feasible_count={self.path_feasible_count} '
            f'ground_path_infeasible_count={self.path_infeasible_count} '
            f'ground_goal_retire_count={self.goal_retire_count} ground_low_gain_retire_count={self.low_gain_retire_count} '
            f'ground_stale_goal_retire_count={self.stale_goal_retire_count} '
            f'ground_goal_blacklist_count={len(self.blacklist) + len(self.low_gain_blacklist)} '
            f'ground_repeat_goal_ratio={repeat_ratio:.3f} ground_frontier_gain_max={self.last_score_terms["frontier_max"]:.3f} '
            f'ground_frontier_gain_avg={self.last_score_terms["frontier_avg"]:.3f} '
            f'ground_unknown_boundary_gain_max={self.last_score_terms["unknown_max"]:.3f} '
            f'ground_unknown_boundary_gain_avg={self.last_score_terms["unknown_avg"]:.3f} '
            f'ground_expected_coverage_gain_max={self.last_score_terms["coverage_max"]:.3f} '
            f'ground_expected_coverage_gain_avg={self.last_score_terms["coverage_avg"]:.3f} '
            f'ground_traversability_score_max={self.last_score_terms["traversability_max"]:.3f} '
            f'ground_traversability_score_avg={self.last_score_terms["traversability_avg"]:.3f} '
            f'ground_clearance_score_max={self.last_score_terms["clearance_max"]:.3f} '
            f'ground_clearance_score_avg={self.last_score_terms["clearance_avg"]:.3f} '
            f'ground_novelty_gain_max={self.last_score_terms["novelty_max"]:.3f} '
            f'ground_novelty_gain_avg={self.last_score_terms["novelty_avg"]:.3f} '
            f'ground_collision_reject_count={self.collision_reject_count} '
            f'ground_boundary_reject_count={self.boundary_reject_count} ground_height_reject_count={self.height_reject_count} '
            f'ground_path_length_avg={path_length_avg:.3f} ground_endpoint_to_goal_distance_avg={endpoint_error_avg:.3f} '
            f'ground_selected_goal_age_max_sec={self.last_selected_goal_age_max_sec:.2f} ground_stale_path_max_sec={self.last_stale_path_sec:.2f} '
            f'ground_uses_3d_map_projection=true ground_2d_slam_dependency_detected=false '
            f'backend_mode={self.backend_mode} octomap_adaptive_scoring={str(octomap_active).lower()} '
            f'planning_mode={planning_mode} last_goal_age_sec={age:.2f}'
        )
        self.status_pub.publish(msg)
        if self.last_marker_array is not None:
            stamp = self.get_clock().now().to_msg()
            for marker in self.last_marker_array.markers:
                marker.header.stamp = stamp
            self.marker_pub.publish(self.last_marker_array)

    @staticmethod
    def _parse_key_values(text):
        out = {}
        for token in text.split():
            if '=' in token:
                key, value = token.split('=', 1)
                out[key] = value
        return out

    @staticmethod
    def _float(value, default=0.0):
        try:
            return float(value)
        except Exception:
            return default

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))

    def _float_list_param(self, name, default):
        value = self.get_parameter(name).value
        if isinstance(value, (list, tuple)):
            return [float(v) for v in value]
        if isinstance(value, str):
            return [float(v.strip()) for v in value.strip('[]').split(',') if v.strip()]
        return list(default)


def main(args=None):
    rclpy.init(args=args)
    node = Ground3DFrontier()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
