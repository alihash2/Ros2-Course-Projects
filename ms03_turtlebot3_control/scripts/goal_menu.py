#!/usr/bin/env python3
"""Interactive CLI menu for sending goals to the ms03 TurtleBot3 behavior tree.

Publishes PoseStamped messages (in WORLD coordinates) on /user_goal, shows
live robot position and current goal, and shuts down Gazebo + the launch
terminal when you exit. Run it in a separate terminal while the sim is up.
"""
import math
import os
import signal
import signal
import subprocess
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# TurtleBot3 world limits (with respect to world origin 0,0)
X_MIN, X_MAX = -4.0, 4.0
Y_MIN, Y_MAX = -4.0, 4.0
DEFAULT_SPAWN = (-2.0, -0.5)   # where the robot spawns in Gazebo/world coords
GOAL_TOLERANCE = 0.15          # matches GoalCheck in bt_executor_node


class GoalMenuNode(Node):
    def __init__(self):
        super().__init__('goal_menu_node')
        self.goal_pub = self.create_publisher(PoseStamped, '/user_goal', 10)

        # Raw odom reading (odom origin == spawn point) and derived world pos
        self.odom_pos = None
        self.current_pos = None
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, qos_profile_sensor_data)

        self.goal_x = DEFAULT_SPAWN[0]
        self.goal_y = DEFAULT_SPAWN[1]
        self.goal_reached_announced = True  # nothing to announce at start

    def _odom_cb(self, msg):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        self.odom_pos = (ox, oy)
        # Convert odom -> world so the menu speaks the same coordinates
        # the user types in (bt_executor_node does the inverse for goals).
        self.current_pos = (ox + DEFAULT_SPAWN[0], oy + DEFAULT_SPAWN[1])


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
    """Kill Gazebo (gz sim / gzserver / gzclient) and any ros2 launch processes."""
    patterns = ['gz sim', 'gzserver', 'gzclient', 'ruby.*gz',
                'turtlebot3_obstacle_avoidance.launch.py', 'ros2 launch']
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
        print(f"[INFO] Sent shutdown to {len(killed)} sim/launch process(es).")
    else:
        print("[INFO] No running sim processes found.")


def main():
    rclpy.init()
    node = GoalMenuNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("\n=============================================")
    print("   TURTLEBOT3 GOAL SENDER — MS03 CONTROL")
    print("=============================================")
    print(f"World limits -> X: {X_MIN}..{X_MAX} | Y: {Y_MIN}..{Y_MAX}")
    print(f"(coordinates are in WORLD frame, origin 0,0 at map center)")
    print(f"Robot default spawn pose: ({DEFAULT_SPAWN[0]}, {DEFAULT_SPAWN[1]})")

    try:
        while rclpy.ok():
            # --- Live status header ---
            pos_str = f"({node.current_pos[0]:.2f}, {node.current_pos[1]:.2f})" \
                if node.current_pos else "waiting for /odom..."
            print("\n---------------------------------------------")
            print(f" Current position : {pos_str}")
            print(f" Current goal     : ({node.goal_x:.2f}, {node.goal_y:.2f})")

            # --- Goal reached detection ---
            if node.current_pos is not None:
                dist = math.hypot(node.goal_x - node.current_pos[0],
                                  node.goal_y - node.current_pos[1])
                if dist < GOAL_TOLERANCE and not node.goal_reached_announced:
                    print(" >>> GOAL REACHED! Robot is at the goal pose. <<<")
                    node.goal_reached_announced = True

            print("---------------------------------------------")
            print(" 1. Send a new goal pose")
            print(" 2. Exit (also shuts down Gazebo & launch terminal)")
            choice = input("Select option [1-2]: ").strip()

            if choice == '1':
                print(f"\n--- Enter Target Coordinates "
                      f"(X: {X_MIN}..{X_MAX}, Y: {Y_MIN}..{Y_MAX}) ---")
                x = prompt_float("X", X_MIN, X_MAX)
                y = prompt_float("Y", Y_MIN, Y_MAX)

                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.header.stamp = node.get_clock().now().to_msg()
                goal.pose.position.x = x
                goal.pose.position.y = y
                goal.pose.orientation.w = 1.0

                node.goal_pub.publish(goal)
                node.goal_x, node.goal_y = x, y
                node.goal_reached_announced = False
                print(f"[SENT] Goal published on /user_goal: ({x:.2f}, {y:.2f})")

            elif choice == '2':
                print("\n[INFO] Exiting and shutting down simulation...")
                shutdown_simulation()
                break
            else:
                print("[!] Invalid selection. Enter 1 or 2.")
    except KeyboardInterrupt:
        print("\n[INFO] User interrupted.")

    rclpy.shutdown()
    spin_thread.join(timeout=1.0)
    sys.exit(0)


if __name__ == '__main__':
    main()
