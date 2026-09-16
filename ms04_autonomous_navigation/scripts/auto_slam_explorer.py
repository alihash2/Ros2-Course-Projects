#!/usr/bin/env python3
"""
Autonomous SLAM Frontier Explorer & Automated Map Saver.
Explores unknown areas using occupancy grid frontiers and automatically
saves the map via slam_toolbox when exploration completes.
"""

import math
import os
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from slam_toolbox.srv import SaveMap


class AutoSlamExplorer(Node):
    def __init__(self):
        super().__init__('auto_slam_explorer')

        # Parameters
        self.declare_parameter('map_save_path', '')
        self.declare_parameter('max_exploration_time', 300.0)  # seconds
        self.declare_parameter('linear_speed', 0.35)          # m/s cruise
        self.declare_parameter('max_linear_speed', 0.6)       # m/s straight-line sprint
        self.declare_parameter('angular_speed', 0.8)          # rad/s
        self.declare_parameter('min_frontier_size', 6)        # cells
        self.declare_parameter('obstacle_distance', 0.35)     # meters
        self.declare_parameter('auto_save', True)

        self.map_save_path = self.get_parameter('map_save_path').get_parameter_value().string_value
        self.max_exploration_time = self.get_parameter('max_exploration_time').get_parameter_value().double_value
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        self.min_frontier_size = self.get_parameter('min_frontier_size').get_parameter_value().integer_value
        self.obstacle_distance = self.get_parameter('obstacle_distance').get_parameter_value().double_value
        self.auto_save = self.get_parameter('auto_save').get_parameter_value().bool_value

        # Frontier memory (visited centroids, anti-oscillation)
        self.visited_frontiers = []  # (x, y, timestamp_sec)
        self.visited_penalty_radius = 1.0   # meters — frontiers within this are 'seen'
        self.visited_penalty_window = 60.0  # seconds — how long a visit penalizes
        self.visited_penalty_factor = 0.25  # score multiplier for seen frontiers

        # Frontier selection bias: prefer outward (distant) frontiers so the
        # robot pushes to the map boundary before re-checking explored space
        self.outward_bias_distance = 1.5   # meters — frontiers beyond this are 'outward'
        self.outward_bias_factor = 0.35    # score multiplier for inward (near) frontiers

        # Obstacle skirting state machine (Bug-0 style): once triggered, the
        # robot commits to going around the obstacle instead of oscillating
        # between 'turn away' and 'realign to goal'
        self.avoid_mode = False
        self.avoid_phase = 0   # 0 = rotating to clear, 1 = skirting along the obstacle
        self.avoid_dir = 1.0   # +1 turns left, -1 turns right
        self.avoid_start_yaw = 0.0
        self.avoid_cum = 0.0   # cumulative rotation in avoid_dir (rad)
        self.avoid_start_time = 0.0

        # Skirting is triggered a little earlier than the hard obstacle
        # distance so the robot commits to going around a wall/door instead of
        # grinding itself into a wall first.
        self.skirt_trigger_dist = 0.55  # meters

        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False

        # Map state
        self.current_map = None
        self.map_received = False

        # LIDAR state
        self.front_dist = 10.0
        self.left_dist = 10.0
        self.right_dist = 10.0
        self.scan_received = False
        self.ranges = None
        self.range_min = 0.0
        self.range_max = 0.0
        self.angle_min = 0.0
        self.angle_increment = 0.0

        # Odometry velocity (used for odom-based stuck detection)
        self.odom_lin_vel = 0.0
        self.last_cmd_linear = 0.0
        self.slow_ticks = 0
        self.slow_stuck_threshold = 15  # 1.5s of commanded motion without travel

        # Navigation state
        self.current_target = None
        self.target_start_time = 0.0
        self.target_origin_x = 0.0
        self.target_origin_y = 0.0
        self.min_travel_time = 2.0       # seconds — must elapse before arrival can be declared
        self.min_travel_distance = 0.3   # meters — robot must have traveled this far from target origin
        self.start_time = self.get_clock().now()
        self.is_exploring = True
        self.map_saved = False
        self.stuck_counter = 0
        self.recovery_mode = False
        self.recovery_phase = 0          # 0 = backing up straight, 1 = rotating in place
        self.recovery_phase_end = 0.0

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

        # Publishers & Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, sensor_qos)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)

        # Service client for saving map
        self.save_map_client = self.create_client(SaveMap, '/slam_toolbox/save_map')

        # Control loop timer (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'AutoSlamExplorer initialized. Target map save path: {self.map_save_path or "(None specified)"}'
        )

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Quaternion to Yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_lin_vel = msg.twist.twist.linear.x
        self.odom_received = True

    def scan_callback(self, msg: LaserScan):
        ranges = msg.ranges
        num_readings = len(ranges)
        if num_readings == 0:
            return

        # Keep the raw beam data (plus sensor geometry) so the motion logic can
        # query how far the wall is in an ARBITRARY direction, e.g. the current
        # bearing to the goal.
        self.ranges = ranges
        self.range_min = msg.range_min
        self.range_max = msg.range_max
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

        def safe_range(start_idx, end_idx):
            valid = [r for r in ranges[start_idx:end_idx] if msg.range_min < r < msg.range_max]
            return min(valid) if valid else 10.0

        # LIDAR ranges around 360 deg
        # 0 deg is front, 90 left, 180 back, 270 right
        idx_front_left = int(num_readings * 30 / 360)
        idx_front_right = int(num_readings * 330 / 360)
        idx_left_start = int(num_readings * 30 / 360)
        idx_left_end = int(num_readings * 90 / 360)
        idx_right_start = int(num_readings * 270 / 360)
        idx_right_end = int(num_readings * 330 / 360)

        front_vals = [r for r in (ranges[:idx_front_left] + ranges[idx_front_right:])
                      if msg.range_min < r < msg.range_max]
        self.front_dist = min(front_vals) if front_vals else 10.0
        self.left_dist = safe_range(idx_left_start, idx_left_end)
        self.right_dist = safe_range(idx_right_start, idx_right_end)
        self.scan_received = True

    def map_callback(self, msg: OccupancyGrid):
        self.current_map = msg
        self.map_received = True

    def range_in_sector(self, center_rad, half_width_rad=0.26):
        """Nearest lidar range within a sector centered on an angle relative
        to the robot's heading (0 = straight ahead, >0 = toward the left)."""
        if not self.scan_received or self.ranges is None:
            return 10.0
        n = len(self.ranges)
        idx = int(round((center_rad - self.angle_min) / self.angle_increment))
        hw = max(1, int(round(half_width_rad / self.angle_increment)))
        best = 10.0
        for k in range(idx - hw, idx + hw + 1):
            r = self.ranges[k % n]
            if self.range_min < r < best:   # r[:range_max] handled by best<10.0 cap
                best = r
        return best

    def find_frontiers(self):
        """Extract frontier cells (free cells adjacent to unknown cells) and cluster them."""
        if not self.current_map:
            return []

        grid = self.current_map.data
        width = self.current_map.info.width
        height = self.current_map.info.height
        res = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y

        visited = bytearray(width * height)
        frontier_cells = []

        # Find frontier cells
        for y in range(1, height - 1):
            row_idx = y * width
            for x in range(1, width - 1):
                idx = row_idx + x
                # Only check known free cells
                if grid[idx] != 0:
                    continue

                # Check 8-neighborhood for unknown space (-1)
                is_frontier = False
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        neighbor_val = grid[(y + dy) * width + (x + dx)]
                        if neighbor_val == -1:
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
                # Compute centroid in world coordinates
                avg_x = sum(c[0] for c in cluster) / len(cluster)
                avg_y = sum(c[1] for c in cluster) / len(cluster)
                world_x = origin_x + avg_x * res
                world_y = origin_y + avg_y * res
                clusters.append((world_x, world_y, len(cluster)))

        return clusters

    def select_best_frontier(self, clusters):
        """Pick frontier with best utility (balance between proximity, cluster size, and exploration memory)."""
        if not clusters:
            return None

        best_score = -1.0
        best_candidate = None

        for (wx, wy, size) in clusters:
            dist = math.hypot(wx - self.robot_x, wy - self.robot_y)
            if dist < 0.4:
                # Too close, likely already mapped
                continue

            # Information gain score: size / distance
            score = float(size) / (dist + 0.8)

            # Outward bias: prefer distant frontiers so the robot expands the
            # map boundary first, only returning to nearby frontiers once the
            # outer area is explored/blocked
            if dist < self.outward_bias_distance:
                score *= self.outward_bias_factor

            # Penalize frontiers near recently-visited centroids (anti-oscillation)
            for (vx, vy, _) in self.visited_frontiers:
                if math.hypot(wx - vx, wy - vy) < self.visited_penalty_radius:
                    score *= self.visited_penalty_factor
                    break

            if score > best_score:
                best_score = score
                best_candidate = (wx, wy)

        return best_candidate

    def prune_visited_frontiers(self, now_sec):
        """Drop visited-frontier entries older than the penalty window."""
        cutoff = now_sec - self.visited_penalty_window
        self.visited_frontiers = [v for v in self.visited_frontiers if v[2] > cutoff]

    def record_visited_frontier(self, target, now_sec):
        """Remember that the robot was sent toward this frontier."""
        self.visited_frontiers.append((target[0], target[1], now_sec))

    def control_loop(self):
        if not self.is_exploring:
            return

        if not (self.odom_received and self.map_received and self.scan_received):
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        elapsed = now_sec - (self.start_time.nanoseconds / 1e9)

        # Check timeout
        if elapsed > self.max_exploration_time:
            self.get_logger().info(f'Max exploration time reached ({elapsed:.1f}s). Completing exploration.')
            self.finish_exploration()
            return

        # Handle Recovery Mode (two-phase: back up straight, then rotate in place)
        if self.recovery_mode:
            twist = Twist()
            if self.recovery_phase == 0:
                twist.linear.x = -0.18
            else:
                twist.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(twist)

            if now_sec >= self.recovery_phase_end:
                if self.recovery_phase == 0:
                    self.recovery_phase = 1
                    self.recovery_phase_end = now_sec + 1.0
                else:
                    self.recovery_mode = False
                    self.current_target = None
            return

        # Check if we need a new target
        need_target = False
        if self.current_target is None:
            need_target = True
        else:
            # Check if target reached or timed out (timeout scales with distance)
            dist_to_target = math.hypot(self.current_target[0] - self.robot_x,
                                        self.current_target[1] - self.robot_y)
            dist_from_start = math.hypot(self.robot_x - self.target_origin_x,
                                         self.robot_y - self.target_origin_y)
            if (dist_to_target < 0.35 and
                    dist_from_start > self.min_travel_distance and
                    (now_sec - self.target_start_time) > self.min_travel_time):
                self.get_logger().info(f'Reached frontier waypoint {self.current_target}')
                self.record_visited_frontier(self.current_target, now_sec)
                need_target = True
            elif (now_sec - self.target_start_time) > max(25.0, dist_to_target / 0.3):
                self.get_logger().info(f'Frontier target {self.current_target} timed out. Selecting new frontier.')
                self.record_visited_frontier(self.current_target, now_sec)
                need_target = True

        if need_target:
            self.prune_visited_frontiers(now_sec)
            clusters = self.find_frontiers()
            if not clusters:
                self.get_logger().info('No more frontiers detected! Environment exploration complete.')
                self.finish_exploration()
                return

            candidate = self.select_best_frontier(clusters)
            if candidate is None:
                self.get_logger().info('No reachable frontiers remaining. Exploration complete.')
                self.finish_exploration()
                return

            self.current_target = candidate
            self.target_start_time = now_sec
            self.target_origin_x = self.robot_x
            self.target_origin_y = self.robot_y
            self.get_logger().info(f'New frontier target selected: ({candidate[0]:.2f}, {candidate[1]:.2f})')

        # Reactive Motion Control toward target
        self.drive_towards_target()

        # Odometry-based stuck detection: commanded to move forward but the
        # robot is not actually traveling (handles wedging/grinding that the
        # front-lidar shield cannot see).
        if self.last_cmd_linear > 0.15 and abs(self.odom_lin_vel) < 0.02:
            self.slow_ticks += 1
        else:
            self.slow_ticks = max(0, self.slow_ticks - 1)

        if self.slow_ticks > self.slow_stuck_threshold and not self.recovery_mode:
            self._begin_recovery(now_sec, reason='commanded motion without travel (odom)')

    def drive_towards_target(self):
        if not self.current_target:
            return

        dx = self.current_target[0] - self.robot_x
        dy = self.current_target[1] - self.robot_y
        target_heading = math.atan2(dy, dx)
        heading_error = target_heading - self.robot_yaw

        # Normalize heading error to [-pi, pi]
        while heading_error > math.pi:
            heading_error -= 2.0 * math.pi
        while heading_error < -math.pi:
            heading_error += 2.0 * math.pi

        twist = Twist()
        now_sec = self.get_clock().now().nanoseconds / 1e9

        # ----- Obstacle skirting state machine (Bug-0 style) -----
        if self.avoid_mode:
            if self._avoid_step(twist):
                self.last_cmd_linear = twist.linear.x
                self.cmd_vel_pub.publish(twist)
                return
            # _avoid_step cleared avoid_mode: fall through to normal seeking

        # Trigger skirting when the path ahead is blocked
        if self.front_dist < self.skirt_trigger_dist:
            self.avoid_dir = self._choose_avoid_dir(heading_error)
            self.avoid_mode = True
            self.avoid_phase = 0
            self.avoid_start_yaw = self.robot_yaw
            self.avoid_start_time = now_sec
            self.avoid_cum = 0.0
            self.stuck_counter += 1
            if self.stuck_counter > 25:  # Stuck for 2.5s
                self._begin_recovery(now_sec, reason='robot trapped by obstacle (skirting)')
                return

            twist.linear.x = 0.0
            twist.angular.z = self.avoid_dir * self.angular_speed
            self.last_cmd_linear = 0.0
            self.cmd_vel_pub.publish(twist)
            return

        self.stuck_counter = max(0, self.stuck_counter - 1)

        # Speed reduction ramp: clear path ahead -> sprint; near obstacle -> slow down
        if self.front_dist > 1.0:
            speed_factor = 1.0
        else:
            speed_factor = max(0.0,
                               (self.front_dist - self.obstacle_distance) /
                               (1.0 - self.obstacle_distance))

        # Heading-based control (proportional, angular velocity clamped to
        # angular_speed to avoid overshoot oscillation)
        if abs(heading_error) > 1.4:
            # Very sharp turn: stop and rotate in place
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed if heading_error > 0 else -self.angular_speed
        elif abs(heading_error) > 0.15:
            # Moderate heading error: proportional steering while easing forward
            twist.linear.x = self.linear_speed * math.cos(heading_error) * speed_factor
            ang = 1.5 * heading_error
            twist.angular.z = math.copysign(min(abs(ang), self.angular_speed), ang)
        else:
            # Nearly straight: sprint at max speed with light corrective steering
            twist.linear.x = self.max_linear_speed * speed_factor
            ang = 1.0 * heading_error
            twist.angular.z = math.copysign(min(abs(ang), self.angular_speed), ang)

        self.last_cmd_linear = twist.linear.x
        self.cmd_vel_pub.publish(twist)

    def _choose_avoid_dir(self, heading_error):
        """Pick the skirting direction: turn toward the goal side when it is
        open enough, otherwise toward the more open side."""
        if heading_error >= 0.0 and self.left_dist > self.obstacle_distance * 2.0:
            return 1.0
        if heading_error < 0.0 and self.right_dist > self.obstacle_distance * 2.0:
            return -1.0
        return 1.0 if self.left_dist >= self.right_dist else -1.0

    def _avoid_step(self, twist):
        """Advance one tick of the skirting state machine.

        Returns True if a motor command was set (still skirting), or False
        after clearing avoid_mode because the way is genuinely clear.
        """
        now_sec = self.get_clock().now().nanoseconds / 1e9

        if self.avoid_phase == 0:
            # Rotate in place until we have cleared the obstacle: a minimum
            # rotation AND an open front ahead. Commits past the 0.4s dwell.
            self.avoid_cum += self.avoid_dir * self.angular_speed * 0.1
            if self.front_dist > 0.6 and abs(self.avoid_cum) >= 0.7:
                self.avoid_phase = 1
                self.avoid_cum = 0.0
                twist.linear.x = self.linear_speed * 0.5
                twist.angular.z = self.avoid_dir * self.angular_speed * 0.6
                return True

            if abs(self.avoid_cum) > math.pi:
                # Rotated a full turn without clearing anything: escalate
                self.avoid_mode = False
                self._begin_recovery(now_sec, reason='no clearance found while skirting')
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                return True

            twist.linear.x = 0.0
            twist.angular.z = self.avoid_dir * self.angular_speed
            return True

        # Phase 1: skirting — creep forward along the obstacle with a gentle
        # persistent turn in the avoid direction. The robot resumes goal
        # seeking ONLY when the direction toward the goal is actually clear:
        # if we exit just because the FRONT happens to be open, the seek
        # controller swings us straight back into the obstacle we just skirted
        # (wall beside the robot, goal still behind it) and the flip-flop
        # returns. So gate the exit on the lidar range along the goal bearing.
        if self.front_dist < self.obstacle_distance:
            # Obstacle ahead again (turning around a corner): back to phase 0
            self.avoid_phase = 0
            self.avoid_cum = 0.0
            twist.linear.x = 0.0
            twist.angular.z = self.avoid_dir * self.angular_speed
            return True

        dx = self.current_target[0] - self.robot_x
        dy = self.current_target[1] - self.robot_y
        goal_heading = math.atan2(dy, dx)
        hl = goal_heading - self.robot_yaw
        goal_error = (hl + math.pi) % (2.0 * math.pi) - math.pi

        if self.range_in_sector(goal_error) >= 1.2 and self.front_dist >= 0.8:
            # Goal bearing is genuinely open again: stop skirting, resume seek
            self.avoid_mode = False
            return False

        # Guard against pathological cases (e.g. the goal lies on the far side
        # of an enormous wall): give up on this target after skirting long
        # enough and let global frontier selection pick something reachable.
        if now_sec - self.avoid_start_time > 60.0:
            self.avoid_mode = False
            self._begin_recovery(now_sec, reason='skirting made no progress toward target')
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            return True

        twist.linear.x = self.linear_speed * 0.55
        nudge = min(1.0, max(0.0, (1.0 - self.front_dist) / 1.0))
        twist.angular.z = self.avoid_dir * (self.angular_speed * 0.5) * nudge
        return True

    def _begin_recovery(self, now_sec, reason='robot stuck'):
        """Bypass the current target and start the two-phase recovery maneuver."""
        self.get_logger().warn(f'Initiating recovery: {reason}.')
        self.recovery_mode = True
        self.recovery_phase = 0
        self.recovery_phase_end = now_sec + 1.0
        self.stuck_counter = 0
        self.slow_ticks = 0
        if self.current_target is not None:
            self.record_visited_frontier(self.current_target, now_sec)
            self.current_target = None

    def finish_exploration(self):
        self.is_exploring = False

        # Stop robot
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)

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
            return

        req = SaveMap.Request()
        req.name.data = self.map_save_path
        future = self.save_map_client.call_async(req)
        future.add_done_callback(self.save_map_response_callback)

    def save_map_response_callback(self, future):
        try:
            res = future.result()
            if res.result == SaveMap.Response.RESULT_SUCCESS:
                self.get_logger().info(f'Map successfully saved to {self.map_save_path}.yaml / .pgm!')
                self.map_saved = True
            else:
                self.get_logger().error(f'SaveMap service failed with code: {res.result}. Trying map_saver_cli...')
                os.system(f'ros2 run nav2_map_server map_saver_cli -f "{self.map_save_path}" --ros-args -p use_sim_time:=true &')
                self.map_saved = True
        except Exception as e:
            self.get_logger().error(f'Exception while calling SaveMap: {e}')


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
