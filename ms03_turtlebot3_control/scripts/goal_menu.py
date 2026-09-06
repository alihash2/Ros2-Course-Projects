#!/usr/bin/env python3
"""Interactive CLI menu for sending goals to the ms03 TurtleBot3 behavior tree.

Publishes PoseStamped messages (in WORLD coordinates) on /user_goal, shows
live robot position, current goal and remaining distance, validates every
goal against the turtlebot3_world map (bounds + pillar obstacles), supports
emergency stop, and shuts down Gazebo + the launch terminal on exit.

Run it in a separate terminal while the sim is up.
"""
import math
import os
import signal
import subprocess
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry

# TurtleBot3 world limits (with respect to world origin 0,0)
X_MIN, X_MAX = -4.0, 4.0
Y_MIN, Y_MAX = -4.0, 4.0
DEFAULT_SPAWN = (-2.0, -0.5)   # where the robot spawns in Gazebo/world coords
GOAL_TOLERANCE = 0.15          # matches GoalCheck in bt_executor_node

# Known pillar positions in turtlebot3_world (world frame). Goals on or near
# a pillar are rejected — the robot cannot occupy them.
PILLARS = [
    (0.0, 0.0),
    (1.1, 0.0),
    (-1.1, 0.0),
    (0.0, 1.1),
    (0.0, -1.1),
]
PILLAR_RADIUS = 0.35   # pillar radius + robot radius safety margin


class GoalMenuNode(Node):
    def __init__(self):
        super().__init__('goal_menu_node')
        self.goal_pub = self.create_publisher(PoseStamped, '/user_goal', 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Raw odom reading (odom origin == spawn point) and derived world pos
        self.odom_pos = None
        self.current_pos = None
        self.current_yaw = 0.0
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
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)


def validate_goal(x, y):
    """Return (ok, reason). Checks world bounds and pillar obstacles."""
    if x < X_MIN or x > X_MAX or y < Y_MIN or y > Y_MAX:
        return False, (f"outside the world bounds! Valid range is "
                       f"X: {X_MIN}..{X_MAX}, Y: {Y_MIN}..{Y_MAX}")
    for px, py in PILLARS:
        if math.hypot(x - px, y - py) < PILLAR_RADIUS:
            return False, (f"an immovable pillar occupies that spot "
                           f"(pillar at ~({px:.1f}, {py:.1f}))")
    return True, ""


def prompt_float(name):
    while True:
        raw = input(f"  {name}: ").strip()
        try:
            return float(raw)
        except ValueError:
            print(f"  [!] '{raw}' is not a number")


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

    print("\n===========================================")
    print("   TURTLEBOT3 GOAL SENDER — MS03 CONTROL")
    print("===========================================")
    print(f"Spawn: ({DEFAULT_SPAWN[0]}, {DEFAULT_SPAWN[1]})  |  Frame: WORLD (map center = 0,0)")

    try:
        while rclpy.ok():
            pos_str = f"({node.current_pos[0]:.2f}, {node.current_pos[1]:.2f})" \
                if node.current_pos else "waiting for /odom..."
            heading_str = f"{math.degrees(node.current_yaw):.1f}°"
            print(f"\n  Position: {pos_str}")
            print(f"  Heading : {heading_str}")
            print(f"  Goal    : ({node.goal_x:.2f}, {node.goal_y:.2f})")
            if node.current_pos is not None:
                dist = math.hypot(node.goal_x - node.current_pos[0],
                                  node.goal_y - node.current_pos[1])
                print(f"  Distance: {dist:.2f} m")
                if dist < GOAL_TOLERANCE and not node.goal_reached_announced:
                    print("  >>> GOAL REACHED <<<")
                    node.goal_reached_announced = True

            print("\n  1. Send Goal")
            print("  2. Emergency Stop")
            print("  3. Exit")
            choice = input("\n  Select [1-3]: ").strip()

            if choice == '1':
                print()
                x = prompt_float("X")
                y = prompt_float("Y")

                ok, reason = validate_goal(x, y)
                if not ok:
                    print(f"\n  [REJECTED] ({x:.2f}, {y:.2f}) - {reason}")
                    continue

                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.header.stamp = node.get_clock().now().to_msg()
                goal.pose.position.x = x
                goal.pose.position.y = y
                goal.pose.orientation.w = 1.0

                node.goal_pub.publish(goal)
                node.goal_x, node.goal_y = x, y
                node.goal_reached_announced = False
                print(f"  [SENT] Goal: ({x:.2f}, {y:.2f})")

            elif choice == '2':
                if node.current_pos is not None:
                    # Send goal at current pose with current heading so BT sees goal reached immediately
                    stop = PoseStamped()
                    stop.header.frame_id = 'map'
                    stop.header.stamp = node.get_clock().now().to_msg()
                    stop.pose.position.x = node.current_pos[0]
                    stop.pose.position.y = node.current_pos[1]
                    # Use current heading so no rotation occurs
                    stop.pose.orientation.z = math.sin(node.current_yaw / 2.0)
                    stop.pose.orientation.w = math.cos(node.current_yaw / 2.0)
                    node.goal_pub.publish(stop)
                    # Also publish zero velocity directly for immediate stop
                    zero = TwistStamped()
                    zero.header.stamp = node.get_clock().now().to_msg()
                    zero.header.frame_id = 'base_link'
                    node.cmd_pub.publish(zero)
                    node.goal_x, node.goal_y = node.current_pos
                    node.goal_reached_announced = True
                    print(f"\n  [STOP] Emergency stop at ({node.goal_x:.2f}, {node.goal_y:.2f}), heading {heading_str}")
                else:
                    print("\n  [STOP] No odometry yet")

            elif choice == '3':
                print("\n  Exiting...")
                shutdown_simulation()
                break
            else:
                print("  Invalid option")
    except KeyboardInterrupt:
        print("\n  Interrupted")
        shutdown_simulation()

    rclpy.shutdown()
    spin_thread.join(timeout=1.0)
    sys.exit(0)


if __name__ == '__main__':
    main()
