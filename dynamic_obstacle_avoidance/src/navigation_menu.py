#!/usr/bin/env python3
"""Interactive CLI mission-control menu for dynamic_obstacle_avoidance.

- Auto-localizes the robot at the default spawn pose (-2.0, -0.5).
- Validates every goal against the global costmap BEFORE sending it:
    * outside the map bounds        -> rejected with reason
    * inside a lethal obstacle      -> rejected with reason
    * unknown / unexplored cell     -> rejected with reason
- Shows live robot position and current goal.
- On exit, shuts down Gazebo, RViz and the launch terminal automatically.
"""
import math
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Conservative usable range around the origin (irregular walls) — the real,
# exact validity of a point is still checked against the costmap.
SUGGESTED_LIMIT = 2.0
DEFAULT_SPAWN = (-2.0, -0.5)
GOAL_TOLERANCE = 0.25

LETHAL_COST = 90       # occupancy probability >= 90 -> lethal obstacle
UNKNOWN_COST = -1      # unexplored cell


class CostmapMonitor(Node):
    """Subscribes to the global costmap + odom for goal validation & display."""

    def __init__(self):
        super().__init__('costmap_monitor')
        self.map_msg = None
        self.current_pos = None

        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self._map_cb, 10)
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)

    def _map_cb(self, msg):
        self.map_msg = msg

    def _odom_cb(self, msg):
        self.current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def validate_goal(self, x, y):
        """Return (ok, reason). Checks map bounds and cell occupancy."""
        m = self.map_msg
        if m is None:
            return False, "costmap not received yet (is Nav2 fully up?)"

        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        w = m.info.width
        h = m.info.height

        # 1. Inside the actual map (the enclosed world walls)?
        if x < ox or y < oy or x >= ox + w * res or y >= oy + h * res:
            return False, (f"outside the map! Map covers X: {ox:.1f}..{ox + w * res:.1f}, "
                           f"Y: {oy:.1f}..{oy + h * res:.1f}")

        # 2. Cell occupancy check
        col = int((x - ox) / res)
        row = int((y - oy) / res)
        # Check the target cell plus its immediate neighbours so goals right
        # against an obstacle/wall are rejected too.
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                rr, cc = row + dr, col + dc
                if 0 <= rr < h and 0 <= cc < w:
                    val = m.data[rr * w + cc]
                    if val == UNKNOWN_COST:
                        return False, "that spot is in unknown/unexplored space"
                    if val >= LETHAL_COST:
                        return False, "an obstacle (or wall) occupies that spot"
        return True, ""


def prompt_float(name, lo, hi):
    while True:
        raw = input(f"Enter {name} ({lo} to {hi}): ").strip()
        try:
            value = float(raw)
        except ValueError:
            print(f"  [!] '{raw}' is not a number. Try again.")
            continue
        if value < lo or value > hi:
            print(f"  [!] {name} must be between {lo} and {hi}. Try again.")
            continue
        return value


def shutdown_simulation():
    """Kill Gazebo, RViz, Nav2 and any ros2 launch processes."""
    patterns = ['gz sim', 'gzserver', 'gzclient', 'ruby.*gz', 'rviz2',
                'navigation.launch.py', 'ros2 launch',
                'nav2_container', 'amcl', 'controller_server',
                'planner_server', 'behavior_server', 'bt_navigator',
                'smoother_server', 'waypoint_follower', 'velocity_smoother',
                'lifecycle_manager', 'map_server', 'cmd_vel_relay']
    killed = set()
    for pat in patterns:
        try:
            out = subprocess.run(
                ['pgrep', '-f', pat], capture_output=True, text=True)
            pids = [p for p in out.stdout.split() if p.isdigit()]
        except FileNotFoundError:
            print("[WARN] pgrep not available; cannot auto-shutdown sim.")
            return
        for pid in pids:
            if pid not in killed:
                killed.add(pid)
                try:
                    os.kill(int(pid), signal.SIGTERM)
                except (ProcessLookupError, PermissionError):
                    pass
    if killed:
        print(f"[INFO] Sent shutdown to {len(killed)} sim/Nav2 process(es).")
    else:
        print("[INFO] No running sim processes found.")


def main():
    rclpy.init()

    monitor = CostmapMonitor()
    # Use a dedicated executor for the monitor so it never touches the global
    # default executor — BasicNavigator.goToPose() needs that one internally.
    monitor_executor = rclpy.executors.SingleThreadedExecutor()
    monitor_executor.add_node(monitor)
    spin_thread = threading.Thread(target=monitor_executor.spin, daemon=True)
    spin_thread.start()

    nav = BasicNavigator()

    print("\n[SYSTEM] Initializing Navigation Menu...")

    # 1. Set Initial Pose (TurtleBot3 default spawn pose: x=-2.0, y=-0.5)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = DEFAULT_SPAWN[0]
    initial_pose.pose.position.y = DEFAULT_SPAWN[1]
    initial_pose.pose.orientation.w = 1.0

    print("[SYSTEM] Setting Initial Pose to spawn location (-2.0, -0.5)...")
    nav.setInitialPose(initial_pose)

    # 2. Wait for Nav2 to activate
    print("[SYSTEM] Waiting for Nav2 components to wake up...")
    try:
        nav.waitUntilNav2Active(localizer='amcl')
        print("\n[SUCCESS] Nav2 is fully Active!")
    except Exception as e:
        print(f"\n[WARN] Activation check timeout: {e}")
        print("[INFO] You can set 2D Pose Estimate in RViz or proceed directly...")

    current_goal = None
    goal_announced = True

    try:
        while rclpy.ok():
            pos_str = f"({monitor.current_pos[0]:.2f}, {monitor.current_pos[1]:.2f})" \
                if monitor.current_pos else "waiting for /odom..."
            goal_str = f"({current_goal[0]:.2f}, {current_goal[1]:.2f})" \
                if current_goal else "none"
            print("\n====================================")
            print("   ROBOT NAVIGATION MISSION CONTROL")
            print("====================================")
            print(f" Current position : {pos_str}")
            print(f" Current goal     : {goal_str}")
            if current_goal is not None and monitor.current_pos is not None:
                d = math.hypot(current_goal[0] - monitor.current_pos[0],
                               current_goal[1] - monitor.current_pos[1])
                if d < GOAL_TOLERANCE and not goal_announced:
                    print(" >>> GOAL REACHED! <<<")
                    goal_announced = True
            print("------------------------------------")
            print(" 1. Send Goal (Compute Path & Move)")
            print(" 2. Emergency Stop (Cancel Task)")
            print(" 3. Exit Mission Control (shuts down sim too)")
            print("------------------------------------")

            choice = input("Enter Choice [1-3]: ").strip()

            if choice == '1':
                print(f"\n--- Enter Target Coordinates ---")
                print(f"Suggested safe range from origin: about "
                      f"-{SUGGESTED_LIMIT}..+{SUGGESTED_LIMIT} on X and Y "
                      f"(walls are irregular; every entry is verified against "
                      f"the live costmap before the robot moves).")
                x = prompt_float("X", -10.0, 10.0)
                y = prompt_float("Y", -10.0, 10.0)

                ok, reason = monitor.validate_goal(x, y)
                if not ok:
                    print(f"\n[REJECTED] Goal ({x:.2f}, {y:.2f}) is invalid: {reason}.")
                    print("[INFO] Enter another coordinate or pick option 3 to exit.")
                    continue

                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.header.stamp = nav.get_clock().now().to_msg()
                goal.pose.position.x = x
                goal.pose.position.y = y
                goal.pose.orientation.w = 1.0

                print(f"\n[MISSION] Pathfinding to ({x}, {y})...")
                nav.goToPose(goal)

                # Monitor progress (single-line update, no log spam)
                while not nav.isTaskComplete():
                    feedback = nav.getFeedback()
                    if feedback:
                        print(f"Distance to goal: {feedback.distance_remaining:.2f} m",
                              end="\r")
                        sys.stdout.flush()
                    time.sleep(0.5)

                result = nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    print("\n[SUCCESS] Goal Reached!")
                    current_goal = (x, y)
                    goal_announced = True
                elif result == TaskResult.CANCELED:
                    print("\n[INFO] Mission Canceled.")
                elif result == TaskResult.FAILED:
                    print("\n[ERROR] Mission Failed (environment may have changed; "
                          "try again).")

            elif choice == '2':
                nav.cancelTask()
                print("\n[STOP] Emergency Stop Command Sent.")

            elif choice == '3':
                print("\n[INFO] Exiting Mission Control and shutting down simulation...")
                shutdown_simulation()
                break
            else:
                print("[!] Invalid selection. Enter 1, 2 or 3.")
    except KeyboardInterrupt:
        print("\n[INFO] User interrupted.")

    rclpy.shutdown()
    monitor_executor.shutdown()
    spin_thread.join(timeout=1.0)
    sys.exit(0)


if __name__ == '__main__':
    main()
