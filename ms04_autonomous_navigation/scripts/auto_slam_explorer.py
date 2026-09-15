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

        # Navigation state
        self.current_target = None
        self.target_start_time = 0.0
        self.start_time = self.get_clock().now()
        self.is_exploring = True
        self.map_saved = False
        self.stuck_counter = 0
        self.recovery_mode = False
        self.recovery_end_time = 0.0

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
        self.odom_received = True

    def scan_callback(self, msg: LaserScan):
        ranges = msg.ranges
        num_readings = len(ranges)
        if num_readings == 0:
            return

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

        # Handle Recovery Mode
        if self.recovery_mode:
            if now_sec < self.recovery_end_time:
                # Back up and turn
                twist = Twist()
                twist.linear.x = -0.10
                twist.angular.z = self.angular_speed
                self.cmd_vel_pub.publish(twist)
                return
            else:
                self.recovery_mode = False
                self.current_target = None

        # Check if we need a new target
        need_target = False
        if self.current_target is None:
            need_target = True
        else:
            # Check if target reached or timed out (timeout scales with distance)
            dist_to_target = math.hypot(self.current_target[0] - self.robot_x,
                                        self.current_target[1] - self.robot_y)
            if dist_to_target < 0.5:
                self.get_logger().info(f'Reached frontier waypoint {self.current_target}')
                self.record_visited_frontier(self.current_target, now_sec)
                need_target = True
            elif (now_sec - self.target_start_time) > max(25.0, dist_to_target / 0.3):
                self.get_logger().info(f'Frontier target {self.current_target} timed out. Selecting new frontier.')
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
            self.get_logger().info(f'New frontier target selected: ({candidate[0]:.2f}, {candidate[1]:.2f})')

        # Reactive Motion Control toward target
        self.drive_towards_target()

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

        # LIDAR Obstacle Avoidance Shield
        if self.front_dist < self.obstacle_distance:
            # Obstacle directly ahead, steer towards the more open side
            twist.linear.x = 0.0
            if self.left_dist > self.right_dist:
                twist.angular.z = self.angular_speed
            else:
                twist.angular.z = -self.angular_speed

            self.stuck_counter += 1
            if self.stuck_counter > 25:  # Stuck for 2.5s
                self.get_logger().warn('Robot trapped by obstacle! Initiating recovery maneuver.')
                self.recovery_mode = True
                self.recovery_end_time = (self.get_clock().now().nanoseconds / 1e9) + 2.5
                self.stuck_counter = 0
                return
        else:
            self.stuck_counter = max(0, self.stuck_counter - 1)

            # Speed reduction ramp: clear path ahead -> sprint; near obstacle -> slow down
            if self.front_dist > 1.5:
                speed_factor = 1.0
            else:
                speed_factor = max(0.0,
                                   (self.front_dist - self.obstacle_distance) /
                                   (1.5 - self.obstacle_distance))

            # Heading-based control (proportional; only stop-and-turn for sharp angles)
            if abs(heading_error) > 1.4:
                # Very sharp turn: stop and rotate in place
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed if heading_error > 0 else -self.angular_speed
            elif abs(heading_error) > 0.15:
                # Moderate heading error: proportional steering while easing forward
                twist.linear.x = self.linear_speed * math.cos(heading_error) * speed_factor
                twist.angular.z = 1.5 * heading_error
            else:
                # Nearly straight: sprint at max speed with light corrective steering
                twist.linear.x = self.max_linear_speed * speed_factor
                twist.angular.z = 1.0 * heading_error

        self.cmd_vel_pub.publish(twist)

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
