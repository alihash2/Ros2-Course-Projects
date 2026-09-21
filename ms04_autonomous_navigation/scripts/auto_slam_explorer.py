#!/usr/bin/env python3
"""
Autonomous SLAM Frontier Explorer & Automated Map Saver.

Industry-standard architecture: the explorer detects frontiers from the live
occupancy grid, scores them (gain/size + potential/distance + orientation),
and sends the best one to Nav2 via the NavigateToPose action server. Nav2 owns
ALL motion, path planning, obstacle avoidance and recovery — exactly how
explore-lite / m-explore drive a robot. Reached or aborted goal poses are
blacklisted permanently so the robot never oscillates on a sliver (the
explore-lite anti-oscillation behavior).

Scoring is the classic three-term frontier cost:
    cost = gain_scale  * size
         + potential_scale * route_distance
         + orientation_scale * angle
Defaults follow explore-lite: gain=1.0, potential=1e-3, orientation=0.0.
"""

import math
import os
import sys
import time
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from slam_toolbox.srv import SaveMap


class AutoSlamExplorer(Node):
    def __init__(self):
        super().__init__('auto_slam_explorer')

        # Parameters
        self.declare_parameter('map_save_path', '')
        self.declare_parameter('max_exploration_time', 900.0)  # seconds
        self.declare_parameter('min_frontier_size', 6)         # cells
        self.declare_parameter('auto_save', True)
        self.declare_parameter('gain_scale', 1.0)
        self.declare_parameter('potential_scale', 0.001)
        self.declare_parameter('orientation_scale', 0.0)
        self.declare_parameter('progress_timeout', 300.0)      # abort a goal if no success
        self.declare_parameter('stuck_timeout', 45.0)          # cancel a goal that barely moves
        self.declare_parameter('stuck_min_move', 1.2)          # m travelled before stuck_timeout is judged OK
        self.declare_parameter('push_into_unknown', 0.6)       # m, push frontier goals past the boundary
        self.declare_parameter('goal_yaw_tolerance', 0.6)      # rad, yaw we request at the goal

        self.map_save_path = self.get_parameter('map_save_path').get_parameter_value().string_value
        self.max_exploration_time = self.get_parameter('max_exploration_time').get_parameter_value().double_value
        self.min_frontier_size = self.get_parameter('min_frontier_size').get_parameter_value().integer_value
        self.auto_save = self.get_parameter('auto_save').get_parameter_value().bool_value
        self.gain_scale = self.get_parameter('gain_scale').get_parameter_value().double_value
        self.potential_scale = self.get_parameter('potential_scale').get_parameter_value().double_value
        self.orientation_scale = self.get_parameter('orientation_scale').get_parameter_value().double_value
        self.progress_timeout = self.get_parameter('progress_timeout').get_parameter_value().double_value
        self.stuck_timeout = self.get_parameter('stuck_timeout').get_parameter_value().double_value
        self.stuck_min_move = self.get_parameter('stuck_min_move').get_parameter_value().double_value
        self.push_into_unknown = self.get_parameter('push_into_unknown').get_parameter_value().double_value
        self.goal_yaw_tolerance = self.get_parameter('goal_yaw_tolerance').get_parameter_value().double_value

        # Stale maps from a previous run silently overwrite the good one only
        # if a previous run saved before finishing (or was interrupted too
        # early). Remove the exact target files at startup so a fresh run
        # always starts clean — no manual cleanup between runs.
        if self.auto_save and self.map_save_path:
            for suffix in ('.pgm', '.yaml'):
                stale = self.map_save_path + suffix
                if os.path.exists(stale):
                    os.remove(stale)
                    self.get_logger().info(f'Removed stale map file: {stale}')

        # Blacklist (explore-lite style, permanent for the run)
        self.visited_frontiers = []   # (x, y, timestamp)
        self.failed_frontiers = []    # (x, y, timestamp)
        self.blacklist_radius = 1.0   # meters
        self.suppressed_frontiers = []  # (x, y, timestamp) hard-vetoed in-place slivers

        # Robot + map state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False
        self.current_map = None
        self.map_received = False

        # Office-completion state
        self.finalize_mode = False
        self.min_finalize_cluster_cells = 10
        self.finalize_trigger_pct = 0.90   # office known% that arms completion phase
        self.finalize_goal_pct = 0.95       # office known% that concludes the run
        self._office_pct_cache = (0.0, 0.0)
        self._last_free_count = 0
        self._last_growth_check = 0.0
        self.last_growth_delta = 0
        self.warmup_seconds = 30.0

        # Nav2 goal state
        self.nav_handle = None            # active NavigateToPose goal handle
        self.nav_goal_target = None       # (x, y) goal pose in map frame
        self.nav_goal_start = 0.0         # wall time the goal was accepted
        self.nav_goal_start_pose = None   # (x, y) robot pose at send
        self._goal_travel = 0.0           # cumulative odom distance this goal
        self._prev_odom_pos = None        # last odom pose for travel accumulation
        self._goal_track_time = 0.0       # wall time of last motion tracking
        self._goal_track_pos = None       # robot pose at last tracking
        self._last_stuck_check = 0.0
        self._pending_goal = None         # (x, y) sent but not yet accepted
        self._pending_frontier = None     # frontier centroid for the goal
        self._reject_until = 0.0          # wall time to hold before retrying
        self.next_diag_time = 0.0

        self.start_time = self.get_clock().now()
        self.is_exploring = True
        self.map_saved = False

        # Boundaries (fixed office envelope, overridable per-map via launch)
        self.declare_parameter('office_lox', -8.0)
        self.declare_parameter('office_loy', -7.0)
        self.declare_parameter('office_hix', 8.0)
        self.declare_parameter('office_hiy', 7.0)
        self.office_lox = self.get_parameter('office_lox').get_parameter_value().double_value
        self.office_loy = self.get_parameter('office_loy').get_parameter_value().double_value
        self.office_hix = self.get_parameter('office_hix').get_parameter_value().double_value
        self.office_hiy = self.get_parameter('office_hiy').get_parameter_value().double_value

        # QoS Profiles
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribers
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, sensor_qos)

        # Nav2 NavigateToPose action client — Nav2 owns ALL motion
        self.nav_action = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Service client for saving map
        self.save_map_client = self.create_client(SaveMap, '/slam_toolbox/save_map')

        # Control loop timer (5 Hz)
        self.control_timer = self.create_timer(0.2, self.control_loop)

        self.get_logger().info(
            f'AutoSlamExplorer initialized (Nav2-driven). Target map save path: '
            f'{self.map_save_path or "(None specified)"}'
        )
        self.get_logger().info('Run configuration:\n' + self.run_config_text())

    # ------------------------------------------------------------------ data
    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        if self._prev_odom_pos is not None:
            self._goal_travel += math.hypot(
                self.robot_x - self._prev_odom_pos[0],
                self.robot_y - self._prev_odom_pos[1])
        self._prev_odom_pos = (self.robot_x, self.robot_y)
        self.odom_received = True

    def map_callback(self, msg: OccupancyGrid):
        self.current_map = msg
        self.map_received = True
    # ------------------------------------------------------------ frontiers
    def find_frontiers(self):
        """Extract frontier cells (free cells adjacent to unknown cells) and
        cluster them into connected components with world centroids."""
        if not self.current_map:
            return []

        grid = self.current_map.data
        width = self.current_map.info.width
        height = self.current_map.info.height
        res = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y

        frontier_cells = []

        # Find frontier cells (free cell with an unknown 8-neighbor)
        for y in range(1, height - 1):
            row_idx = y * width
            for x in range(1, width - 1):
                idx = row_idx + x
                if grid[idx] != 0:
                    continue
                is_frontier = False
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        if grid[(y + dy) * width + (x + dx)] == -1:
                            is_frontier = True
                            break
                    if is_frontier:
                        break
                if is_frontier:
                    frontier_cells.append((x, y))

        if not frontier_cells:
            return []

        # Cluster adjacent frontier cells via BFS
        frontier_set = set(frontier_cells)
        clusters = []
        for cell in frontier_cells:
            if cell not in frontier_set:
                continue
            cluster = []
            queue = deque([cell])
            frontier_set.remove(cell)
            while queue:
                cx, cy = queue.popleft()
                cluster.append((cx, cy))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        neighbor = (cx + dx, cy + dy)
                        if neighbor in frontier_set:
                            frontier_set.remove(neighbor)
                            queue.append(neighbor)
            if len(cluster) >= self.min_frontier_size:
                avg_x = sum(c[0] for c in cluster) / len(cluster)
                avg_y = sum(c[1] for c in cluster) / len(cluster)
                clusters.append((origin_x + avg_x * res,
                                 origin_y + avg_y * res,
                                 len(cluster),
                                 list(cluster)))
        return clusters

    # ------------------------------------------------------------- flooding
    def flood(self):
        """BFS distances from the robot over known-FREE cells. Returns
        (dist, width, height, res, ox, oy) or None when not usable."""
        if not self.current_map:
            return None
        info = self.current_map.info
        res = info.resolution
        width = int(info.width)
        height = int(info.height)
        grid = self.current_map.data
        ox = info.origin.position.x
        oy = info.origin.position.y
        rc = int(round((self.robot_x - ox) / res))
        rr = int(round((self.robot_y - oy) / res))
        if rc < 0 or rc >= width or rr < 0 or rr >= height:
            return None
        start = rr * width + rc
        dist = {start: 0}
        dq = deque([start])
        while dq:
            cur = dq.popleft()
            r = cur // width
            c = cur % width
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nr = r + dr
                nc = c + dc
                if nr < 0 or nr >= height or nc < 0 or nc >= width:
                    continue
                idx = nr * width + nc
                if idx in dist:
                    continue
                if grid[idx] != 0:
                    continue
                dist[idx] = dist[cur] + 1
                dq.append(idx)
        return dist, width, height, res, ox, oy

    def approach_cell(self, flood, wx, wy):
        """Nearest flooded (reachable, known-free) cell to a frontier world
        centroid, used as the Nav2 goal — Nav2 plans from current pose to this
        free cell, so the robot always navigates on mapped free space. Returns
        (world_x, world_y, bfs_dist) or None if unreachable."""
        dist, width, height, res, ox, oy = flood
        gx = int(round((wx - ox) / res))
        gy = int(round((wy - oy) / res))
        radius = max(1, int(round(1.5 / res)))
        best = None
        best_d = 1 << 30
        for dy in range(-radius, radius + 1):
            yy = gy + dy
            if yy < 0 or yy >= height:
                continue
            for dx in range(-radius, radius + 1):
                xx = gx + dx
                if xx < 0 or xx >= width:
                    continue
                d = dist.get(yy * width + xx)
                if d is None or d < 1:
                    continue
                if d < best_d:
                    best_d = d
                    best = (ox + xx * res, oy + yy * res, d)
        return best

    # ----------------------------------------------------------- office math
    def _office_known_pct(self):
        """Cached office-window known fraction (free + occupied over the fixed
        world rect x in [-8,8], y in [-7,7])."""
        now_sec = self.get_clock().now().nanoseconds / 1e9
        cached, cached_at = self._office_pct_cache
        if now_sec - cached_at < 2.0 and cached_at > 0.0:
            return cached
        pct = 100.0
        if self.current_map is not None:
            g = self.current_map
            known = 0
            total = 0
            for iy in range(g.info.height):
                wy = g.info.origin.position.y + iy * g.info.resolution
                if wy < self.office_loy or wy > self.office_hiy:
                    continue
                row = iy * g.info.width
                for ix in range(g.info.width):
                    wx = g.info.origin.position.x + ix * g.info.resolution
                    if wx < self.office_lox or wx > self.office_hix:
                        continue
                    total += 1
                    v = g.data[row + ix]
                    if v == 0 or v == 100:
                        known += 1
            pct = 0.0 if not total else 100.0 * known / total
        self._office_pct_cache = (pct, now_sec)
        return pct

    def _office_unknown_targets(self, top_n=8):
        """Unknown regions inside the office window, list of
        (world_x, world_y, area_cells) sorted largest-first. Used by the
        completion phase to close the last coverage gaps."""
        g = self.current_map
        if g is None:
            return []
        ox = g.info.origin.position.x
        oy = g.info.origin.position.y
        res = g.info.resolution
        width = g.info.width
        unvisited = {}
        for r in range(g.info.height):
            wy = oy + r * res
            if wy < self.office_loy or wy > self.office_hiy + 0.0001:
                continue
            row = r * width
            for c in range(g.info.width):
                wx = ox + c * res
                if wx < self.office_lox or wx > self.office_hix + 0.0001:
                    continue
                v = g.data[row + c]
                if v != 0 and v != 100:
                    unvisited[r * width + c] = None
        clusters = []
        while unvisited:
            seed = next(iter(unvisited))
            comp = []
            stack = [seed]
            del unvisited[seed]
            while stack:
                idx = stack.pop()
                r = idx // width
                c = idx % width
                comp.append(idx)
                if r > 0:
                    n = idx - width
                    if n in unvisited:
                        del unvisited[n]
                        stack.append(n)
                if r < g.info.height - 1:
                    n = idx + width
                    if n in unvisited:
                        del unvisited[n]
                        stack.append(n)
                if c > 0:
                    n = idx - 1
                    if n in unvisited:
                        del unvisited[n]
                        stack.append(n)
                if c < width - 1:
                    n = idx + 1
                    if n in unvisited:
                        del unvisited[n]
                        stack.append(n)
            size = len(comp)
            if size < self.min_finalize_cluster_cells:
                continue
            cx = sum(ox + (i % width) * res for i in comp) / size
            cy = sum(oy + (i // width) * res for i in comp) / size
            clusters.append((cx, cy, size))
        clusters.sort(key=lambda t: t[2], reverse=True)
        return clusters[:top_n]

    # ------------------------------------------------------------- blacklist
    def _blacklisted(self, wx, wy):
        """True if a frontier centroid is too close to a previously reached or
        failed goal (permanent explore-lite blacklist)."""
        for (vx, vy, _) in self.visited_frontiers + self.failed_frontiers:
            if math.hypot(wx - vx, wy - vy) < self.blacklist_radius:
                return True
        return False

    def record_visited(self, wx, wy, sys_time=None, wall_time=None):
        if sys_time is None:
            sys_time = self.get_clock().now().nanoseconds / 1e9
        self.visited_frontiers.append((wx, wy, sys_time))

    def record_failed(self, wx, wy, sys_time=None):
        if sys_time is None:
            sys_time = self.get_clock().now().nanoseconds / 1e9
        self.failed_frontiers.append((wx, wy, sys_time))

    # ------------------------------------------------------------- selection
    def select_best_frontier(self, clusters):
        """Industry-standard 3-term frontier scoring:
            cost = gain_scale * size
                 + potential_scale * route_distance
                 + orientation_scale * angle
        Returns (goal_x, goal_y, yaw, size, bfs_dist) or None.

        The Nav2 goal is the frontier CENTROID itself, i.e. a pose at the edge
        of the known map that looks into unknown space — exactly what
        explore-lite sends to the planner. Navfn runs with allow_unknown=true
        so it plans into that sliver; DWB + local costmap keep the robot clear
        as it crosses it, and 12 m lidar uncovers it on arrival.
        `approach_cell` is used ONLY as a reachability check: if no known-free
        path leads to within 1.5 m of the centroid, the frontier is skipped.
        """
        flood = self.flood()
        if flood is None:
            return None
        best = None
        best_score = -1.0
        for (wx, wy, size, cells) in clusters:
            if self._blacklisted(wx, wy):
                continue
            appr = self.approach_cell(flood, wx, wy)
            if appr is None:
                continue  # not reachable through known free space
            # Straight-line distance from robot to centroid; makes the score
            # prefer large, reasonably-close frontiers.
            route_m = math.hypot(wx - self.robot_x, wy - self.robot_y)
            angle = 0.0
            cost = (self.gain_scale * size
                    + self.potential_scale * route_m
                    + self.orientation_scale * angle)
            if cost > best_score:
                best_score = cost
                yaw = math.atan2(wy - self.robot_y, wx - self.robot_x)
                best = (wx, wy, yaw, size, int(round(route_m / flood[3])), appr, cells)
        if best is None:
            return None
        wx, wy, yaw, size, bd, appr, cells = best
        # Aim the goal at the centre of the UNKNOWN region behind this frontier
        # (the far end / corners of the room beyond), rather than at the
        # boundary itself. Driving to the middle of the unseen area lets the
        # 12 m lidar open the entire room in one visit. If the unknown blob is
        # too small to matter, fall back to a fixed nudge past the boundary.
        room = self._unknown_room_centroid(wx, wy, cells)
        if room is not None:
            gx, gy, area = room
            yaw = math.atan2(gy - self.robot_y, gx - self.robot_x)
            self.get_logger().info(
                f'Frontier goal aimed at unknown room '
                f'({gx:.2f}, {gy:.2f}) area={area}')
        else:
            gx, gy, _ = appr
            dx = wx - gx
            dy = wy - gy
            dist_m = math.hypot(dx, dy)
            if dist_m > 1e-6:
                ux, uy = dx / dist_m, dy / dist_m
            else:
                ux = math.cos(yaw)
                uy = math.sin(yaw)
            gx = wx + ux * self.push_into_unknown
            gy = wy + uy * self.push_into_unknown
            yaw = math.atan2(gy - self.robot_y, gx - self.robot_x)
        return (gx, gy, yaw, size, bd, wx, wy)

    def _unknown_room_centroid(self, wx, wy, cells, max_radius=6.0, max_cells=4000):
        """Centroid of the contiguous unknown region reachable from the
        frontier cluster (wx, wy, cells), used as a deep-room exploration
        goal. Seeds a BFS from every unknown cell adjacent to any frontier
        cell, then returns the centroid of the connected unknown blob those
        seeds grow into. Returns (world_x, world_y, area_cells) or None."""
        g = self.current_map
        if g is None:
            return None
        ox = g.info.origin.position.x
        oy = g.info.origin.position.y
        res = g.info.resolution
        width = g.info.width
        gx = int(round((wx - ox) / res))
        gy = int(round((wy - oy) / res))
        def is_unknown(xx, yy):
            if xx < 0 or xx >= width or yy < 0 or yy >= g.info.height:
                return False
            v = g.data[yy * width + xx]
            return v == -1
        # Seeds: unknown cells adjacent to this cluster's frontier cells
        # (frontier cells ARE free cells next to unknown, so this always
        # finds the room beyond the boundary).
        radius = max(1, int(round(max_radius / res)))
        radius2_big = radius * radius
        seeds = set()
        for (fx, fy) in cells:
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    if is_unknown(fx + dx, fy + dy):
                        d2 = (fx + dx - gx) ** 2 + (fy + dy - gy) ** 2
                        if d2 <= radius2_big:
                            seeds.add((fx + dx, fy + dy))
        if not seeds:
            return None
        dq = deque(seeds)
        seen = set(seeds)
        while dq:
            cx, cy = dq.popleft()
            if (cx - gx) ** 2 + (cy - gy) ** 2 > radius2_big:
                continue
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                xx, yy = cx + dx, cy + dy
                if (xx, yy) in seen:
                    continue
                if (xx - gx) ** 2 + (yy - gy) ** 2 > radius2_big:
                    continue
                if is_unknown(xx, yy):
                    seen.add((xx, yy))
                    dq.append((xx, yy))
        if len(seen) < 8:
            return None  # too small to aim at
        cells_out = list(seen)
        if len(cells_out) > max_cells:
            cells_out = cells_out[:max_cells]
        cx = sum(c[0] for c in cells_out) / len(cells_out)
        cy = sum(c[1] for c in cells_out) / len(cells_out)
        return (ox + cx * res, oy + cy * res, len(seen))

    def _finalize_target(self):
        """Completion-phase target: largest unknown region inside the office
        that still has a reachable known-free approach cell."""
        if not self.finalize_mode:
            return None
        flood = self.flood()
        if flood is None:
            return None
        for (wx, wy, area) in self._office_unknown_targets():
            if self._blacklisted(wx, wy):
                continue
            if self.approach_cell(flood, wx, wy) is None:
                continue
            yaw = math.atan2(wy - self.robot_y, wx - self.robot_x)
            self.get_logger().info(
                f'FINALIZE target: unknown region (area={area}) around '
                f'({wx:.2f}, {wy:.2f})')
            return (wx, wy, yaw, area, 0)
        return None

    # ------------------------------------------------------------- nav goals
    def send_nav_goal(self, gx, gy, yaw, frontier_xy=None):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        send_future = self.nav_action.send_goal_async(goal)
        send_future.add_done_callback(self._goal_accepted_cb)
        self._pending_goal = (gx, gy)
        self._pending_frontier = frontier_xy
        self.nav_goal_target = (gx, gy)
        self.nav_goal_start = self.get_clock().now().nanoseconds / 1e9
        self._goal_travel = 0.0
        self._prev_odom_pos = (self.robot_x, self.robot_y)
        self.get_logger().info(
            f'NavigateToPose sent: ({gx:.2f}, {gy:.2f}) yaw={yaw:.2f} '
            f'[size={self._cur_size}, dist={self._cur_bd}]')

    def _goal_accepted_cb(self, future):
        """Blacklist a goal ONLY once the action server actually accepted it.
        If the server is not up yet (goal rejected), leave the frontier
        unblacklisted and schedule a retry so it is not lost forever."""
        try:
            handle = future.result()
        except Exception:
            handle = None
        if handle is None or not getattr(handle, 'accepted', True):
            self.get_logger().warn(
                'NavigateToPose goal rejected (Nav2 not ready?) — will retry '
                f'{self._pending_goal} shortly.')
            # Do NOT blacklist a rejected goal.
            self.nav_goal_target = None
            self._pending_goal = None
            self._pending_frontier = None
            self._reject_until = self.get_clock().now().nanoseconds / 1e9 + 3.0
            return
        # Server accepted: now it is safe to mark the frontier as visited.
        if self._pending_frontier is not None:
            self.record_visited(self._pending_frontier[0], self._pending_frontier[1])
        elif self._pending_goal is not None:
            self.record_visited(self._pending_goal[0], self._pending_goal[1])
        self._pending_goal = None
        self._pending_frontier = None
        self.nav_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        target = self.nav_goal_target
        self.nav_handle = None
        self.nav_goal_target = None
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('NavigateToPose SUCCEEDED.')
            if target is not None:
                self.record_visited(target[0], target[1])
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('NavigateToPose ABORTED.')
            if target is not None:
                self.record_failed(target[0], target[1])
            self._reject_until = self.get_clock().now().nanoseconds / 1e9 + 2.0
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('NavigateToPose CANCELED.')
        else:
            self.get_logger().warn(f'NavigateToPose result status: {status}')

    def _too_early_to_finish(self, elapsed_sec):
        """Refuse to conclude 'no frontiers / unreachable' while the map is
        still building. Requires at least 60s AND some real office coverage
        before a 'nothing left to explore' verdict counts."""
        if elapsed_sec < 60.0:
            return True
        opct = self._office_known_pct()
        if opct < 40.0:
            self.get_logger().info(
                f'Office only {opct:.1f}% known — refusing to finish yet.')
            return True
        return False

    def nav_busy(self):
        return self.nav_handle is not None or self.nav_goal_target is not None

    # ------------------------------------------------------------ diagnostics
    def _emit_diag(self):
        kinds = {}
        if self.current_map:
            for v in self.current_map.data:
                kinds[v] = kinds.get(v, 0) + 1
            free = kinds.get(0, 0)
            unk = kinds.get(-1, 0)
            occ = kinds.get(100, 0)
        else:
            free = unk = occ = 0
        ow = {'free': 0, 'unk': 0, 'occ': 0}
        ow_total = 0
        if self.current_map:
            g = self.current_map
            for iy in range(g.info.height):
                wy = g.info.origin.position.y + iy * g.info.resolution
                if wy < self.office_loy or wy > self.office_hiy:
                    continue
                row = iy * g.info.width
                for ix in range(g.info.width):
                    wx = g.info.origin.position.x + ix * g.info.resolution
                    if wx < self.office_lox or wx > self.office_hix:
                        continue
                    v = g.data[row + ix]
                    ow_total += 1
                    if v == 0:
                        ow['free'] += 1
                    elif v == 100:
                        ow['occ'] += 1
                    else:
                        ow['unk'] += 1
        ow_pct = 0.0 if not ow_total else \
            100.0 * (ow['free'] + ow['occ']) / ow_total
        try:
            nf = len(self.find_frontiers())
        except Exception:
            nf = -1
        nav = 'busy' if self.nav_busy() else 'idle'
        self.get_logger().info(
            f'DIAG pose=({self.robot_x:.2f},{self.robot_y:.2f}) '
            f'yaw={self.robot_yaw:.2f} free={free} unk={unk} occ={occ} '
            f'frontiers={nf} nav={nav} '
            f'office={ow["free"]}/{ow["unk"]}/{ow["occ"]} office_pct={ow_pct:.1f}')

    # ------------------------------------------------------------ control loop
    def control_loop(self):
        if not self.is_exploring:
            return
        if not (self.odom_received and self.map_received):
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        elapsed = now_sec - (self.start_time.nanoseconds / 1e9)

        if now_sec >= self.next_diag_time:
            self.next_diag_time = now_sec + 10.0
            self._emit_diag()

        if elapsed > self.max_exploration_time:
            self.get_logger().info(
                f'Max exploration time reached ({elapsed:.1f}s). Completing exploration.')
            self.finish_exploration(reason='max_time')
            return

        # Office-completion goal met?
        if self._office_known_pct() >= self.finalize_goal_pct * 100.0:
            self.get_logger().info(f'Office known {self._office_known_pct():.1f}% — goal met.')
            self.finish_exploration(reason='office_complete')
            return

        # Nav2 not up yet: wait.
        if not self.nav_action.server_is_ready():
            self.get_logger().info('Waiting for Nav2 navigate_to_pose server...', once=True)
            return

        # A goal was just rejected (Nav2 starting up); hold off a moment before
        # resending so we do not hammer the server.
        if now_sec < self._reject_until:
            return

        # Growth watchdog arms the completion phase once the office is mostly
        # covered and the map has stopped growing rapidly.
        if now_sec - self._last_growth_check >= 5.0:
            self._last_growth_check = now_sec
            if self.current_map is not None:
                free_now = sum(1 for v in self.current_map.data if v == 0)
                if self._last_free_count == 0:
                    self._last_free_count = free_now
                else:
                    self.last_growth_delta = free_now - self._last_free_count
                    self._last_free_count = free_now
                if (elapsed > self.warmup_seconds
                        and self._office_known_pct() >= self.finalize_trigger_pct * 100.0
                        and self.last_growth_delta < 20):
                    self.finalize_mode = True

        # Abandon a goal that has been running well past a reasonable budget.
        if self.nav_busy() and now_sec - self.nav_goal_start > self.progress_timeout:
            self.get_logger().warn(f'Nav goal timed out ({self.progress_timeout:.0f}s). '
                                   f'Aborting {self.nav_goal_target}.')
            if self.nav_handle is not None:
                self.nav_handle.cancel_goal_async()
            if self.nav_goal_target is not None:
                self.record_failed(self.nav_goal_target[0], self.nav_goal_target[1], now_sec)
            self.nav_goal_target = None

        # Stuck watchdog: cancel a goal that has barely moved in the last
        # `stuck_timeout` seconds even though the controller is still "busy"
        # (this is Nav2 spinning/backing up in place for far too long). Uses
        # cumulative odometric travel since the goal was sent, so a slow but
        # steadily-moving robot is never flagged — only stagnant rotation is.
        if (self.nav_busy() and now_sec - self._last_stuck_check >= 5.0
                and now_sec - self.nav_goal_start > self.stuck_timeout):
            self._last_stuck_check = now_sec
            if self._goal_travel < self.stuck_min_move:
                self.get_logger().warn(
                    f'Goal barely moved ({self._goal_travel:.2f}m in '
                    f'{now_sec - self.nav_goal_start:.0f}s). Aborting stuck '
                    f'{self.nav_goal_target}.')
                if self.nav_handle is not None:
                    self.nav_handle.cancel_goal_async()
                if self.nav_goal_target is not None:
                    self.record_failed(self.nav_goal_target[0], self.nav_goal_target[1], now_sec)
                self.nav_goal_target = None

        if self.nav_busy():
            return  # let Nav2 finish the current goal

        # Select the next goal: finalize-mode unknown regions take priority;
        # otherwise standard frontier scoring.
        self._cur_size = 0
        self._cur_bd = 0
        target = self._finalize_target()
        if target is None:
            clusters = self.find_frontiers()
            if not clusters:
                if self._too_early_to_finish(elapsed):
                    self.get_logger().info(
                        f'No frontiers yet at {elapsed:.0f}s (office {self._office_known_pct():.1f}%)'
                        f' — waiting for maps to build.')
                    return
                self.get_logger().info('No more frontiers detected! Environment exploration complete.')
                self.finish_exploration(reason='no_frontiers')
                return
            t = self.select_best_frontier(clusters)
            if t is None:
                if self._too_early_to_finish(elapsed):
                    self.get_logger().info(
                        f'Frontiers exist but none reachable yet at {elapsed:.0f}s — waiting.')
                    return
                # Every frontier is blacklisted or unreachable — complete.
                self.get_logger().info('No reachable unblacklisted frontiers. Exploration complete.')
                self.finish_exploration(reason='no_reachable_frontier')
                return
            self._cur_size = t[3]
            self._cur_bd = t[4]
            target = t

        # Frontier goals: 7-tuple (gx,gy,yaw,size,bd,wx,wy); finalize 5-tuple.
        if len(target) >= 7:
            gx, gy, yaw, size, bd, fx, fy = target
            self.send_nav_goal(gx, gy, yaw, frontier_xy=(fx, fy))
        else:
            gx, gy, yaw, size, bd = target
            self.send_nav_goal(gx, gy, yaw)

    # ------------------------------------------------------------- completion
    def finish_exploration(self, reason='no frontier route'):
        self.is_exploring = False

        if self.nav_handle is not None:
            self.nav_handle.cancel_goal_async()

        elapsed = 0.0
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.start_time:
            elapsed = now_sec - (self.start_time.nanoseconds / 1e9)
        kinds = {}
        if self.current_map:
            for v in self.current_map.data:
                kinds[v] = kinds.get(v, 0) + 1
        ow_counts = {'free': 0, 'unk': 0, 'occ': 0}
        ow_total = 0
        if self.current_map:
            g = self.current_map
            for iy in range(g.info.height):
                wy = g.info.origin.position.y + iy * g.info.resolution
                if wy < self.office_loy or wy > self.office_hiy:
                    continue
                row = iy * g.info.width
                for ix in range(g.info.width):
                    wx = g.info.origin.position.x + ix * g.info.resolution
                    if wx < self.office_lox or wx > self.office_hix:
                        continue
                    v = g.data[row + ix]
                    ow_total += 1
                    if v == 0:
                        ow_counts['free'] += 1
                    elif v == 100:
                        ow_counts['occ'] += 1
                    else:
                        ow_counts['unk'] += 1
        ow_known = 0.0
        if ow_total:
            ow_known = 100.0 * (ow_counts['free'] + ow_counts['occ']) / ow_total
        self.get_logger().warn(
            f'RESULT reason={reason} elapsed={elapsed:.1f}s '
            f'free={kinds.get(0, 0)} unk={kinds.get(-1, 0)} occ={kinds.get(100, 0)} '
            f'office_wfree={ow_counts["free"]} office_wunk={ow_counts["unk"]} '
            f'office_wocc={ow_counts["occ"]} office_known_pct={ow_known:.1f}')

        if self.auto_save and not self.map_saved:
            self.save_map()

    def save_map(self):
        if not self.map_save_path:
            self.get_logger().warn('No map_save_path provided. Skipping automatic map saving.')
            return
        self.get_logger().info(f'Saving generated SLAM map to: {self.map_save_path} ...')
        os.makedirs(os.path.dirname(os.path.abspath(self.map_save_path)), exist_ok=True)

        if not self.save_map_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service /slam_toolbox/save_map not available. Falling back to nav2 map_saver_cli...')
            os.system(f'ros2 run nav2_map_server map_saver_cli -f "{self.map_save_path}" --ros-args -p use_sim_time:=true &')
            self.map_saved = True
            threading.Timer(8.0, self.snapshot_map).start()
            return

        req = SaveMap.Request()
        req.name.data = self.map_save_path + '_slam'
        future = self.save_map_client.call_async(req)
        future.add_done_callback(self.save_map_response_callback)

    def save_map_response_callback(self, future):
        try:
            res = future.result()
            if res.result != SaveMap.Response.RESULT_SUCCESS:
                self.get_logger().error(f'SaveMap service failed with code: {res.result}.')
            self.get_logger().info(
                f'Saving canonical FULL /map to {self.map_save_path} via nav2 map_saver_cli...')
            os.system(f'ros2 run nav2_map_server map_saver_cli -f "{self.map_save_path}" '
                      f'--ros-args -p use_sim_time:=true &')
            self.map_saved = True
            threading.Timer(8.0, self.snapshot_map).start()
        except Exception as e:
            self.get_logger().error(f'Exception while calling SaveMap: {e}')

    def run_config_text(self):
        return ('max_exploration_time=' + format(self.max_exploration_time, '.1f') + '\n'
                'min_frontier_size=' + str(self.min_frontier_size) + '\n'
                'gain_scale=' + str(self.gain_scale) + '\n'
                'potential_scale=' + str(self.potential_scale) + '\n'
                'orientation_scale=' + str(self.orientation_scale) + '\n'
                'progress_timeout=' + str(self.progress_timeout) + '\n'
                'nav_controller=Nav2 DWB (costmap + inflation + recovery)\n'
                'planner=NavfnPlanner allow_unknown\n'
                'finalize_trigger_pct=' + str(self.finalize_trigger_pct) + '\n'
                'finalize_goal_pct=' + str(self.finalize_goal_pct) + '\n'
                'lidar_range=12.0m\n'
                'blacklist=permanent (explore-lite style)\n')

    def snapshot_map(self):
        if not self.map_save_path:
            return
        try:
            base = os.path.basename(self.map_save_path)
            ts = time.strftime('%Y%m%d_%H%M%S')
            d = os.path.dirname(os.path.abspath(self.map_save_path))
            for suffix in ('.pgm', '.yaml'):
                src = self.map_save_path + suffix
                dst = os.path.join(d, f'{os.path.splitext(base)[0]}_{ts}{suffix}')
                if os.path.exists(src):
                    import shutil
                    shutil.copy2(src, dst)
            cfg_dst = os.path.join(d, f'{os.path.splitext(base)[0]}_{ts}.cfg')
            with open(cfg_dst, 'w') as f:
                f.write(f'# snapshot: {ts}\n')
                f.write(self.run_config_text())
            self.get_logger().info(f'Map snapshot + config saved: {os.path.splitext(base)[0]}_{ts}.*')
        except Exception as e:
            self.get_logger().warn(f'Could not snapshot map: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = AutoSlamExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('AutoSlamExplorer interrupted by user.')
        node.finish_exploration()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
