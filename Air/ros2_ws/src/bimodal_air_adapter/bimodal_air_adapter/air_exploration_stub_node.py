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


class AirExplorationStub(Node):
    def __init__(self):
        super().__init__('air_exploration_stub_node')
        self.declare_parameter('air_goal_z', 1.5)
        self.declare_parameter('safety_radius', 0.5)
        self.declare_parameter('update_period_sec', 5.0)
        self.declare_parameter('max_candidate_count', 96)
        self.declare_parameter('visual_exploration_mode', True)
        self.declare_parameter('candidate_count', 80)
        self.declare_parameter('min_goal_distance', 1.0)
        self.declare_parameter('max_goal_distance', 5.0)
        self.declare_parameter('unknown_gain_weight', 2.0)
        self.declare_parameter('forward_gain_weight', 0.8)
        self.declare_parameter('distance_gain_weight', 0.5)
        self.declare_parameter('obstacle_penalty_weight', 2.0)
        self.declare_parameter('revisit_penalty_weight', 1.5)
        self.declare_parameter('goal_hold_sec', 5.0)
        self.declare_parameter('max_candidate_markers', 80)
        self.declare_parameter('min_goal_separation', 1.5)
        self.declare_parameter('blacklist_radius', 1.0)
        self.declare_parameter('blacklist_ttl_sec', 45.0)
        self.declare_parameter('sector_balance_enabled', True)
        self.declare_parameter('sector_count', 8)
        self.declare_parameter('frontier_bias_enabled', True)
        self.declare_parameter('coverage_gain_radius', 2.0)
        self.declare_parameter('frontier_gain_weight', 2.0)
        self.declare_parameter('coverage_gain_weight', 2.5)
        self.declare_parameter('sector_balance_weight', 1.0)
        self.declare_parameter('goal_switch_min_improvement', 0.4)
        self.declare_parameter('low_gain_window_sec', 15.0)
        self.declare_parameter('min_coverage_gain_per_goal', 0.005)
        self.declare_parameter('low_gain_blacklist_ttl_sec', 90.0)
        self.declare_parameter('recent_goal_history_size', 50)
        self.declare_parameter('octomap_adaptive_scoring', True)
        self.declare_parameter('planner_mode', 'stub')
        self.declare_parameter('enable_stub_fallback', True)
        self.declare_parameter('air_octomap_safety_radius', 0.6)
        self.declare_parameter('fuel_style_z_min', 0.8)
        self.declare_parameter('fuel_style_z_nominal', 1.4)
        self.declare_parameter('fuel_style_z_max', 2.2)
        self.declare_parameter('unknown_boundary_weight', 2.0)
        self.declare_parameter('occupied_penalty_weight', 4.0)
        self.declare_parameter('low_gain_penalty_weight', 1.0)
        self.odom = None
        self.points = []
        self.map_voxels = set()
        self.map_metrics = {}
        self.map_backend_status = {}
        self.backend_mode = 'unknown'
        self.map_resolution = 0.5
        self.last_coverage_proxy = 0.0
        self.boundary = (-10.0, 10.0, -10.0, 10.0, 0.0, 3.0)
        self.boundary_received = False
        self.last_goal_time = 0.0
        self.last_goal = None
        self.visited_goals = []
        self.tick_count = 0
        self.last_candidate_count = 0
        self.last_valid_candidate_count = 0
        self.last_selected_score = 0.0
        self.last_rejected_collision = 0
        self.last_rejected_revisit = 0
        self.last_candidate_marker_array = None
        self.last_selected_goal_marker = None
        self.blacklist = []
        self.sector_visit_counts = {}
        self.last_selected_sector = -1
        self.last_rejected_blacklist = 0
        self.last_rejected_low_gain = 0
        self.last_held_goal = False
        self.last_switched_goal = False
        self.held_goal_count = 0
        self.switched_goal_count = 0
        self.last_goal_score = 0.0
        self.low_gain_blacklist = []
        self.active_goal_monitor = None
        self.last_frontier_ring_score = 0.0
        self.last_octomap_frontier_gain = 0.0
        self.last_unknown_boundary_gain = 0.0
        self.last_occupied_penalty = 0.0
        self.last_coverage_gain_estimate = 0.0
        self.last_goal_progress_distance = 0.0
        self.last_rejected_occupied = 0
        self.selected_goal_count = 0
        self.path_feasible_count = 0
        self.path_infeasible_count = 0
        self.path_length_sum = 0.0
        self.endpoint_error_sum = 0.0
        self.fallback_active = False
        self.failure_reason = 'NONE'
        self.create_subscription(Odometry, '/bimodal/odom', self._odom_cb, 10)
        self.create_subscription(PointCloud2, '/bimodal/map_3d', self._map_cb, 10)
        self.create_subscription(String, '/bimodal/map_metrics', self._map_metrics_cb, 10)
        self.create_subscription(String, '/bimodal/map_backend_status', self._map_backend_status_cb, 10)
        self.create_subscription(PointCloud2, '/bimodal/esdf', lambda msg: None, 10)
        self.create_subscription(MarkerArray, '/bimodal/exploration_boundary', self._boundary_cb, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/air/exploration_goal', 10)
        self.path_pub = self.create_publisher(Path, '/air/trajectory', 10)
        self.status_pub = self.create_publisher(String, '/air/planner_status', 10)
        self.candidate_marker_pub = self.create_publisher(MarkerArray, '/air/candidate_markers', 10)
        self.selected_goal_marker_pub = self.create_publisher(Marker, '/air/selected_goal_marker', 10)
        period = float(self.get_parameter('update_period_sec').value)
        self.timer = self.create_timer(max(period, 0.5), self._plan)
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
        self.boundary = (
            m.pose.position.x - m.scale.x / 2.0, m.pose.position.x + m.scale.x / 2.0,
            m.pose.position.y - m.scale.y / 2.0, m.pose.position.y + m.scale.y / 2.0,
            m.pose.position.z - m.scale.z / 2.0, m.pose.position.z + m.scale.z / 2.0,
        )
        self.boundary_received = True

    def _candidate_points(self):
        min_x, max_x, min_y, max_y, _, _ = self.boundary
        if self._planner_mode() == 'fuel_style_v0':
            z = self._clamp(
                float(self.get_parameter('fuel_style_z_nominal').value),
                float(self.get_parameter('fuel_style_z_min').value),
                float(self.get_parameter('fuel_style_z_max').value),
            )
        else:
            z = float(self.get_parameter('air_goal_z').value)
        count = int(self.get_parameter('candidate_count').value)
        pts = []
        rx, ry, _ = self._robot_xyz()
        min_d = float(self.get_parameter('min_goal_distance').value)
        max_d = float(self.get_parameter('max_goal_distance').value)
        for i in range(count):
            a = (self.tick_count * 17 + i * 47) % 360
            band = max(max_d - min_d, 0.1)
            r = min_d + ((i * 19 + self.tick_count * 7) % 100) / 100.0 * band
            x = max(min_x, min(max_x, rx + r * math.cos(math.radians(a))))
            y = max(min_y, min(max_y, ry + r * math.sin(math.radians(a))))
            pts.append((x, y, z))
        return pts

    def _robot_xyz(self):
        if self.odom is None:
            return (0.0, 0.0, float(self.get_parameter('air_goal_z').value))
        p = self.odom.pose.pose.position
        return (float(p.x), float(p.y), float(p.z))

    def _clearance(self, candidate):
        cx, cy, cz = candidate
        nearest = 99.0
        for x, y, z in self.points[:: max(len(self.points) // 900, 1)]:
            nearest = min(nearest, math.dist((cx, cy, cz), (x, y, z)))
        return nearest

    def _unknown_gain(self, candidate):
        cx, cy, cz = candidate
        near = 0
        for x, y, z in self.points[:: max(len(self.points) // 1200, 1)]:
            if math.dist((cx, cy, cz), (x, y, z)) < 1.5:
                near += 1
        return 1.0 / (1.0 + near)

    def _coverage_gain_estimate(self, candidate):
        cx, cy, cz = candidate
        radius = float(self.get_parameter('coverage_gain_radius').value)
        voxel = max(self.map_resolution, 0.2)
        unknown = 0
        total = 0
        steps = range(-int(radius / voxel), int(radius / voxel) + 1)
        for ix in steps:
            for iy in steps:
                for iz in (-1, 0, 1):
                    x = cx + ix * voxel
                    y = cy + iy * voxel
                    z = cz + iz * voxel
                    if math.dist((cx, cy, cz), (x, y, z)) > radius:
                        continue
                    total += 1
                    if (round(x / voxel), round(y / voxel), round(z / voxel)) not in self.map_voxels:
                        unknown += 1
        if total <= 0:
            return 0.0
        return unknown / total

    def _octomap_active(self):
        return bool(self.get_parameter('octomap_adaptive_scoring').value) and self.backend_mode == 'octomap_style_voxel'

    def _planner_mode(self):
        mode = str(self.get_parameter('planner_mode').value).strip()
        return mode if mode in ('stub', 'fuel_style_v0', 'external_wrapper_placeholder') else 'stub'

    def _wrapper_active(self):
        return self._planner_mode() == 'fuel_style_v0'

    def _occupied_penalty(self, candidate):
        cx, cy, cz = candidate
        safety = float(self.get_parameter('air_octomap_safety_radius').value)
        clearance = self._clearance(candidate)
        near = 0
        for x, y, z in self.points[:: max(len(self.points) // 1200, 1)]:
            if math.dist((cx, cy, cz), (x, y, z)) < safety * 1.5:
                near += 1
        density_penalty = min(near / 12.0, 1.0)
        clearance_penalty = max(0.0, 1.0 - clearance / max(safety, 0.05))
        return max(density_penalty, clearance_penalty)

    def _unknown_boundary_gain(self, candidate):
        cx, cy, cz = candidate
        radius = float(self.get_parameter('coverage_gain_radius').value)
        near = 0
        ring = 0
        for x, y, z in self.points[:: max(len(self.points) // 1500, 1)]:
            dist = math.dist((cx, cy, cz), (x, y, z))
            if dist < radius * 0.7:
                near += 1
            elif dist < radius * 1.8:
                ring += 1
        local_edge = min(ring / 16.0, 1.0) * (1.0 / (1.0 + near / 5.0))
        global_unknown = self._float(self.map_metrics.get('unknown_boundary_proxy'), 0.0)
        global_frontier = self._float(self.map_metrics.get('frontier_candidate_proxy'), 0.0)
        return max(0.0, min(0.55 * local_edge + 0.25 * global_unknown + 0.20 * global_frontier, 1.0))

    def _revisit_penalty(self, candidate):
        cx, cy, _ = candidate
        return sum(max(0.0, 1.0 - math.hypot(cx - gx, cy - gy) / 1.8) for gx, gy in self.visited_goals)

    def _score_candidate(self, candidate):
        rx, ry, _ = self._robot_xyz()
        cx, cy, _ = candidate
        d = max(math.hypot(cx - rx, cy - ry), 1e-6)
        max_d = max(float(self.get_parameter('max_goal_distance').value), 0.1)
        unknown_gain = self._unknown_gain(candidate)
        frontier_gain = self._frontier_gain(candidate) if bool(self.get_parameter('frontier_bias_enabled').value) else 0.0
        coverage_gain = self._coverage_gain_estimate(candidate)
        sector_gain = self._sector_gain(candidate) if bool(self.get_parameter('sector_balance_enabled').value) else 0.0
        forward_gain = max((cx - rx) / d, 0.0)
        distance_gain = min(d / max_d, 1.0)
        clearance = self._clearance(candidate)
        obstacle_penalty = 1.0 / max(clearance, 0.05)
        revisit_penalty = self._revisit_penalty(candidate)
        unknown_boundary_gain = self._unknown_boundary_gain(candidate) if self._octomap_active() else 0.0
        occupied_penalty = self._occupied_penalty(candidate) if self._octomap_active() else obstacle_penalty
        low_gain_penalty = 1.0 if self._is_low_gain_blacklisted(candidate) else 0.0
        octomap_frontier_gain = max(frontier_gain, unknown_boundary_gain * 0.7) if self._octomap_active() else frontier_gain
        score = (
            float(self.get_parameter('unknown_gain_weight').value) * unknown_gain
            + float(self.get_parameter('frontier_gain_weight').value) * octomap_frontier_gain
            + float(self.get_parameter('unknown_boundary_weight').value) * unknown_boundary_gain
            + float(self.get_parameter('coverage_gain_weight').value) * coverage_gain
            + float(self.get_parameter('sector_balance_weight').value) * sector_gain
            + float(self.get_parameter('forward_gain_weight').value) * forward_gain
            + float(self.get_parameter('distance_gain_weight').value) * distance_gain
            - float(self.get_parameter('occupied_penalty_weight' if self._octomap_active() else 'obstacle_penalty_weight').value) * occupied_penalty
            - float(self.get_parameter('revisit_penalty_weight').value) * revisit_penalty
            - float(self.get_parameter('low_gain_penalty_weight').value) * low_gain_penalty
        )
        return score, clearance, revisit_penalty, octomap_frontier_gain, coverage_gain, unknown_boundary_gain, occupied_penalty

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
        if self.odom is None or not self.points or not self.boundary_received:
            self.failure_reason = 'WAITING_FOR_INPUT'
            self._publish_status()
            return
        if self.last_goal is not None and time.time() - self.last_goal_time < float(self.get_parameter('goal_hold_sec').value):
            self.last_held_goal = True
            self.held_goal_count += 1
            self._publish_status()
            return
        rejected_collision = 0
        rejected_revisit = 0
        rejected_blacklist = 0
        scored = []
        rejected = []
        safety = float(self.get_parameter('safety_radius').value)
        min_d = float(self.get_parameter('min_goal_distance').value)
        max_d = float(self.get_parameter('max_goal_distance').value)
        rx, ry, _ = self._robot_xyz()
        candidates = self._candidate_points()
        for cand in candidates:
            d = math.hypot(cand[0] - rx, cand[1] - ry)
            score, clearance, revisit_penalty, frontier_gain, coverage_gain, unknown_boundary_gain, occupied_penalty = self._score_candidate(cand)
            if d < min_d or d > max_d or clearance < safety:
                rejected_collision += 1
                rejected.append((cand, 'collision_or_distance'))
                continue
            if self._octomap_active() and occupied_penalty > 0.88:
                self.last_rejected_occupied += 1
                rejected.append((cand, 'rejected_occupied'))
                continue
            if self._is_blacklisted(cand):
                rejected_blacklist += 1
                rejected.append((cand, 'blacklist'))
                continue
            if self._is_low_gain_blacklisted(cand):
                self.last_rejected_low_gain += 1
                rejected.append((cand, 'low_gain_blacklist'))
                continue
            if revisit_penalty > 0.85:
                rejected_revisit += 1
                rejected.append((cand, 'revisit'))
                continue
            if self._recent_goal_penalty(cand) > 0.85:
                rejected_revisit += 1
                rejected.append((cand, 'recent_goal'))
                continue
            if self._wrapper_active():
                feasible, path_length, endpoint_error = self._path_feasibility(cand)
                if not feasible:
                    self.path_infeasible_count += 1
                    rejected.append((cand, 'path_infeasible'))
                    continue
                self.path_feasible_count += 1
                self.path_length_sum += path_length
                self.endpoint_error_sum += endpoint_error
            scored.append((score, cand, clearance, revisit_penalty, frontier_gain, coverage_gain, unknown_boundary_gain, occupied_penalty))
        if not scored:
            self.failure_reason = 'NO_VALID_WRAPPER_CANDIDATE' if self._wrapper_active() else 'NO_COLLISION_FREE_CANDIDATE'
            self.fallback_active = self._wrapper_active() and bool(self.get_parameter('enable_stub_fallback').value)
            self.last_candidate_count = len(candidates)
            self.last_valid_candidate_count = 0
            self.last_rejected_collision = rejected_collision
            self.last_rejected_revisit = rejected_revisit
            self.last_rejected_blacklist = rejected_blacklist
            self._publish_candidate_markers([], rejected, None)
            self._publish_status(extra='NO_COLLISION_FREE_CANDIDATE')
            return
        scored.sort(reverse=True, key=lambda item: item[0])
        best = scored[0]
        selected = best[1]
        if self._should_keep_current_goal(best[0]):
            self.last_held_goal = True
            self.held_goal_count += 1
            self._publish_candidate_markers(scored, rejected, None)
            self._publish_status(extra='HOLDING_CURRENT_GOAL_PROGRESS_GATE')
            return
        if self.last_goal is not None:
            self._add_blacklist(self.last_goal)
        self.last_goal_score = float(best[0])
        self.last_frontier_ring_score = float(best[4])
        self.last_octomap_frontier_gain = float(best[4])
        self.last_coverage_gain_estimate = float(best[5])
        self.last_unknown_boundary_gain = float(best[6])
        self.last_occupied_penalty = float(best[7])
        self.last_switched_goal = self.last_goal is not None
        if self.last_switched_goal:
            self.switched_goal_count += 1
        self.last_goal = selected
        self.last_goal_time = time.time()
        self.selected_goal_count += 1
        self.active_goal_monitor = {
            'goal': selected,
            'start_time': time.time(),
            'start_coverage': self.last_coverage_proxy,
            'start_distance': math.dist(self._robot_xyz(), selected),
        }
        self.last_selected_sector = self._sector(selected)
        self.sector_visit_counts[self.last_selected_sector] = self.sector_visit_counts.get(self.last_selected_sector, 0) + 1
        self.visited_goals.append((selected[0], selected[1]))
        self.visited_goals = self.visited_goals[-int(self.get_parameter('recent_goal_history_size').value):]
        self.last_candidate_count = len(candidates)
        self.last_valid_candidate_count = len(scored)
        self.last_selected_score = float(best[0])
        self.last_rejected_collision = rejected_collision
        self.last_rejected_revisit = rejected_revisit
        self.last_rejected_blacklist = rejected_blacklist
        self._publish_candidate_markers(scored, rejected, best)
        self._publish_goal_and_path(selected)
        self._publish_status()

    def _path_feasibility(self, selected):
        rx, ry, rz = self._robot_xyz()
        gx, gy, gz = selected
        path_length = math.dist((rx, ry, rz), selected)
        endpoint_error = 0.0
        if gz < float(self.get_parameter('fuel_style_z_min').value) or gz > float(self.get_parameter('fuel_style_z_max').value):
            return False, path_length, endpoint_error
        if self._line_collision_risk(rx, ry, rz, gx, gy, gz):
            return False, path_length, endpoint_error
        return True, path_length, endpoint_error

    def _line_collision_risk(self, sx, sy, sz, gx, gy, gz):
        steps = max(int(math.dist((sx, sy, sz), (gx, gy, gz)) / max(self.map_resolution, 0.2)), 8)
        safety = float(self.get_parameter('air_octomap_safety_radius').value)
        for i in range(steps + 1):
            t = i / steps
            x = sx + (gx - sx) * t
            y = sy + (gy - sy) * t
            z = sz + (gz - sz) * t
            if self._clearance((x, y, z)) < safety:
                return True
        return False

    def _frontier_gain(self, candidate):
        cx, cy, cz = candidate
        radius = float(self.get_parameter('coverage_gain_radius').value)
        near = 0
        mid = 0
        for x, y, z in self.points[:: max(len(self.points) // 1200, 1)]:
            dist = math.dist((cx, cy, cz), (x, y, z))
            if dist < radius:
                near += 1
            if radius <= dist < radius * 1.8:
                mid += 1
        known_edge = min(mid / 12.0, 1.0)
        sparse_center = 1.0 / (1.0 + near)
        return max(0.0, min(known_edge + sparse_center, 1.0))

    def _recent_goal_penalty(self, candidate):
        cx, cy, _ = candidate
        sep = float(self.get_parameter('min_goal_separation').value)
        if sep <= 0:
            return 0.0
        return sum(max(0.0, 1.0 - math.hypot(cx - gx, cy - gy) / sep) for gx, gy in self.visited_goals[-12:])

    def _sector(self, candidate):
        rx, ry, _ = self._robot_xyz()
        angle = math.atan2(candidate[1] - ry, candidate[0] - rx)
        sectors = max(int(self.get_parameter('sector_count').value), 1)
        return int(((angle + math.pi) / (2.0 * math.pi)) * sectors) % sectors

    def _sector_gain(self, candidate):
        sector = self._sector(candidate)
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

    def _add_blacklist(self, goal):
        if goal is None:
            return
        self.blacklist.append((float(goal[0]), float(goal[1]), float(goal[2]), time.time()))
        self.blacklist = self.blacklist[-80:]

    def _is_blacklisted(self, candidate):
        radius = float(self.get_parameter('blacklist_radius').value)
        cx, cy, cz = candidate
        return any(math.dist((cx, cy, cz), (x, y, z)) < radius for x, y, z, _ in self.blacklist)

    def _is_low_gain_blacklisted(self, candidate):
        radius = float(self.get_parameter('blacklist_radius').value)
        cx, cy, cz = candidate
        return any(math.dist((cx, cy, cz), (x, y, z)) < radius for x, y, z, _ in self.low_gain_blacklist)

    def _should_keep_current_goal(self, best_score):
        if self.last_goal is None or self.odom is None:
            return False
        improvement = float(best_score) - float(self.last_goal_score)
        min_improvement = float(self.get_parameter('goal_switch_min_improvement').value)
        if time.time() - self.last_goal_time < float(self.get_parameter('goal_hold_sec').value):
            return improvement < min_improvement
        dist = math.dist(self._robot_xyz(), self.last_goal)
        self.last_goal_progress_distance = 0.0
        if self.active_goal_monitor is not None:
            self.last_goal_progress_distance = max(0.0, float(self.active_goal_monitor['start_distance']) - dist)
        return self.last_goal_progress_distance < 0.2 and improvement < min_improvement

    def _publish_goal_and_path(self, selected):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = selected
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        path = Path()
        path.header = goal.header
        start = PoseStamped()
        start.header = goal.header
        start.pose = self.odom.pose.pose
        start.pose.position.z = self.odom.pose.pose.position.z
        path.poses = [start, goal]
        self.path_pub.publish(path)
        self._publish_selected_goal_marker(goal)

    def _publish_candidate_markers(self, scored, rejected, selected):
        markers = []
        now = self.get_clock().now().to_msg()
        clear = Marker()
        clear.header.stamp = now
        clear.header.frame_id = 'map'
        clear.ns = 'air_candidates'
        clear.id = 0
        clear.action = Marker.DELETEALL
        markers.append(clear)
        marker_id = 1
        max_markers = int(self.get_parameter('max_candidate_markers').value)
        for item in scored[:max_markers]:
            score, cand, _, revisit, frontier_gain, coverage_gain, unknown_boundary_gain, occupied_penalty = item
            if occupied_penalty > 0.75:
                r, g, b = (1.0, 0.1, 0.1)
            elif unknown_boundary_gain > 0.65:
                r, g, b = (0.0, 0.95, 1.0)
            elif frontier_gain > 0.75 or coverage_gain > 0.75:
                r, g, b = (0.0, 1.0, 0.35)
            else:
                r, g, b = (0.15, 0.75, 1.0) if revisit < 0.2 else (1.0, 0.75, 0.1)
            markers.append(self._candidate_marker(marker_id, cand, r, g, b, 0.55, now, scale=0.18 + min(max(score, 0.0), 1.0) * 0.06))
            marker_id += 1
        for cand, reason in rejected[:max(0, max_markers - len(scored[:max_markers]))]:
            if reason == 'revisit':
                color = (1.0, 0.55, 0.1)
            elif reason == 'blacklist':
                color = (0.8, 0.1, 0.9)
            elif reason == 'low_gain_blacklist':
                color = (0.45, 0.0, 0.7)
            elif reason == 'rejected_occupied':
                color = (1.0, 0.0, 0.0)
            else:
                color = (1.0, 0.1, 0.1)
            markers.append(self._candidate_marker(marker_id, cand, color[0], color[1], color[2], 0.35, now, scale=0.14))
            marker_id += 1
        if selected:
            markers.append(self._candidate_marker(marker_id, selected[1], 0.05, 0.15, 1.0, 1.0, now, scale=0.38))
        self.last_candidate_marker_array = MarkerArray(markers=markers)
        self.candidate_marker_pub.publish(self.last_candidate_marker_array)

    def _candidate_marker(self, marker_id, candidate, r, g, b, a, stamp, scale=0.18):
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = 'map'
        m.ns = 'air_candidates'
        m.id = marker_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = candidate
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = a
        return m

    def _publish_selected_goal_marker(self, goal):
        marker = Marker()
        marker.header = goal.header
        marker.ns = 'air_selected_goal'
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = goal.pose
        marker.scale.x = 0.7
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        marker.color.r = 0.05
        marker.color.g = 0.25
        marker.color.b = 1.0
        marker.color.a = 1.0
        self.last_selected_goal_marker = marker
        self.selected_goal_marker_pub.publish(marker)

    def _publish_status(self, extra='OK'):
        msg = String()
        age = time.time() - self.last_goal_time if self.last_goal_time else -1.0
        octomap_active = self._octomap_active()
        planner_mode = self._planner_mode()
        planning_mode = 'P3A_air_fuel_style_v0' if planner_mode == 'fuel_style_v0' else ('P2E_air_octomap_frontier_quality' if octomap_active else 'P2D_air_frontier_quality')
        if self.odom is None:
            state = 'WAITING_FOR_ODOM'
        elif not self.points:
            state = 'WAITING_FOR_MAP'
        elif not self.boundary_received:
            state = 'WAITING_FOR_BOUNDARY'
        else:
            state = extra
        path_length_avg = self.path_length_sum / max(self.path_feasible_count, 1)
        endpoint_error_avg = self.endpoint_error_sum / max(self.path_feasible_count, 1)
        repeat_ratio = 0.0
        if self.visited_goals:
            keys = {(round(x), round(y)) for x, y in self.visited_goals}
            repeat_ratio = max(0.0, 1.0 - len(keys) / max(len(self.visited_goals), 1))
        msg.data = (
            f'{state} odom_received={self.odom is not None} map_received={bool(self.points)} '
            f'planner_mode={planner_mode} air_planner_mode={planner_mode} '
            f'fallback_active={str(self.fallback_active).lower()} failure_reason={self.failure_reason} '
            f'boundary_received={self.boundary_received} candidate_count={self.last_candidate_count or int(self.get_parameter("candidate_count").value)} '
            f'valid_candidate_count={self.last_valid_candidate_count} selected_score={self.last_selected_score:.3f} '
            f'coverage_gain_estimate={self.last_coverage_gain_estimate:.3f} frontier_ring_score={self.last_frontier_ring_score:.3f} '
            f'octomap_frontier_gain={self.last_octomap_frontier_gain:.3f} unknown_boundary_gain={self.last_unknown_boundary_gain:.3f} '
            f'occupied_penalty={self.last_occupied_penalty:.3f} '
            f'selected_goal={self.last_goal} held_goal={str(self.last_held_goal).lower()} switched_goal={str(self.last_switched_goal).lower()} '
            f'blacklist_size={len(self.blacklist)} low_gain_blacklist_size={len(self.low_gain_blacklist)} selected_sector={self.last_selected_sector} '
            f'goal_progress_distance={self.last_goal_progress_distance:.3f} '
            f'rejected_by_collision_count={self.last_rejected_collision} '
            f'rejected_by_occupied_count={self.last_rejected_occupied} '
            f'rejected_by_revisit_count={self.last_rejected_revisit} rejected_by_blacklist_count={self.last_rejected_blacklist} '
            f'rejected_by_low_gain_count={self.last_rejected_low_gain} rejected_by_low_gain_blacklist_count={self.last_rejected_low_gain} '
            f'held_goal_count={self.held_goal_count} switched_goal_count={self.switched_goal_count} '
            f'air_candidate_count={self.last_candidate_count or int(self.get_parameter("candidate_count").value)} '
            f'air_valid_candidate_count={self.last_valid_candidate_count} air_selected_goal_count={self.selected_goal_count} '
            f'air_path_feasible_count={self.path_feasible_count} air_path_infeasible_count={self.path_infeasible_count} '
            f'air_goal_blacklist_count={len(self.blacklist) + len(self.low_gain_blacklist)} '
            f'air_repeat_goal_ratio={repeat_ratio:.3f} air_frontier_gain_max={self.last_octomap_frontier_gain:.3f} '
            f'air_unknown_boundary_gain_max={self.last_unknown_boundary_gain:.3f} '
            f'air_path_length_avg={path_length_avg:.3f} air_endpoint_to_goal_distance_avg={endpoint_error_avg:.3f} '
            f'map_point_count={len(self.points)} backend_mode={self.backend_mode} '
            f'octomap_adaptive_scoring={str(octomap_active).lower()} planning_mode={planning_mode} last_goal_age_sec={age:.2f}'
        )
        self.status_pub.publish(msg)
        if self.last_candidate_marker_array is not None:
            stamp = self.get_clock().now().to_msg()
            for marker in self.last_candidate_marker_array.markers:
                marker.header.stamp = stamp
            self.candidate_marker_pub.publish(self.last_candidate_marker_array)
        if self.last_selected_goal_marker is not None:
            self.last_selected_goal_marker.header.stamp = self.get_clock().now().to_msg()
            self.selected_goal_marker_pub.publish(self.last_selected_goal_marker)

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


def main(args=None):
    rclpy.init(args=args)
    node = AirExplorationStub()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
