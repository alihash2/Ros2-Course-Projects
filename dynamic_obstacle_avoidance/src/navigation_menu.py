#!/usr/bin/env python3
"""Interactive CLI mission-control menu for dynamic_obstacle_avoidance.

- Auto-localizes the robot at the default spawn pose (-2.0, -0.5).
- Validates every goal against the global costmap BEFORE sending it:
    * outside the map bounds        -> rejected with reason
    * inside a lethal obstacle      -> rejected with reason
    * unknown / unexplored cell     -> rejected with reason
- Shows live robot position, heading, and current goal.
- Runtime planner switching: global (A*/RRT) and local (MPPI/DWB).
- On exit, shuts down Gazebo, RViz and the launch terminal automatically.
"""
import math
import os
import select
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


# Conservative usable range around the origin (irregular walls) — the real,
# exact validity of a point is still checked against the costmap.
SUGGESTED_LIMIT = 2.0
DEFAULT_SPAWN = (-2.0, -0.5)
GOAL_TOLERANCE = 0.25

LETHAL_COST = 90       # occupancy probability >= 90 -> lethal obstacle
UNKNOWN_COST = -1      # unexplored cell

# Available planners (must be configured in param files to appear).
# Value = (plugin-id used by Nav2 selector topics, short display name)
GLOBAL_PLANNERS = {
    '1': ('GridBased', 'A*'),
    '2': ('RRTBased', 'RRT'),
}
LOCAL_PLANNERS = {
    '1': ('FollowPath', 'MPPI'),
    '2': ('DWBPath', 'DWB'),
    '3': ('PidLqrPath', 'PID+LQR'),
}


class CostmapMonitor(Node):
    """Subscribes to the global costmap + odom for goal validation & display."""

    def __init__(self):
        super().__init__('costmap_monitor')
        self.map_msg = None
        self.static_map_msg = None
        self.current_pos = None
        self.current_yaw = 0.0

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self._map_cb, latched_qos)
        self.create_subscription(
            OccupancyGrid, '/map', self._static_map_cb, latched_qos)
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)

    def _map_cb(self, msg):
        self.map_msg = msg

    def _static_map_cb(self, msg):
        self.static_map_msg = msg

    def _odom_cb(self, msg):
        self.current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def validate_goal(self, x, y):
        """Return (ok, reason). Checks map bounds and cell occupancy."""
        m = self.map_msg if self.map_msg is not None else self.static_map_msg
        if m is None:
            return False, "neither global costmap nor static map received yet (is Nav2 fully up?)"

        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        w = m.info.width
        h = m.info.height

        if x < ox or y < oy or x >= ox + w * res or y >= oy + h * res:
            return False, (f"outside the map! Map covers X: {ox:.1f}..{ox + w * res:.1f}, "
                           f"Y: {oy:.1f}..{oy + h * res:.1f}")

        col = int((x - ox) / res)
        row = int((y - oy) / res)
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


def prompt_float(name):
    while True:
        raw = input(f"  {name}: ").strip()
        try:
            return float(raw)
        except ValueError:
            print(f"  [!] '{raw}' is not a number")


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


class PlannerSwitcher(Node):
    """Switches the active global/local planner at runtime.

    Nav2's default behavior tree uses two selector nodes ("ControllerSelector" and
    "PlannerSelector") that subscribe to the "controller_selector" and
    "planner_selector" topics (std_msgs/String) with LATCHED (transient-local)
    QoS. Publishing the plugin ID of a loaded plugin switches which
    controller/planner the next navigation goal uses. Because the subscription
    is latched, the selection is retained across navigation goals even though
    the BT selector nodes are recreated per goal.
    """

    def __init__(self):
        super().__init__('planner_switcher')
        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.planner_selector_pub = self.create_publisher(String, '/planner_selector', latched)
        self.controller_selector_pub = self.create_publisher(String, '/controller_selector', latched)

    def switch_global_planner(self, plugin_id):
        """Switch global planner by publishing its plugin ID."""
        msg = String()
        msg.data = plugin_id
        self.planner_selector_pub.publish(msg)
        return True, f"Switched global planner to {plugin_id}"

    def switch_local_planner(self, plugin_id):
        """Switch local controller by publishing its plugin ID."""
        msg = String()
        msg.data = plugin_id
        self.controller_selector_pub.publish(msg)
        return True, f"Switched local planner to {plugin_id}"


def do_switch_global(switcher, current_global):
    """Interactively switch the global planner. Returns the new name (unchanged on invalid input)."""
    print("\n  Global Planners:")
    for k, (_, name) in GLOBAL_PLANNERS.items():
        mark = " (current)" if name == current_global else ""
        print(f"   {k}. {name}{mark}")
    glen = len(GLOBAL_PLANNERS)
    sel = input(f"  Select [1-{glen}]: ").strip()
    if sel in GLOBAL_PLANNERS:
        plugin_id, name = GLOBAL_PLANNERS[sel]
        print(f"  Switching global planner to {name}...")
        ok, msg = switcher.switch_global_planner(plugin_id)
        if ok:
            print(f"  [OK] {msg}")
            return name
        print(f"  [FAIL] {msg}")
    else:
        print("  Invalid selection")
    return current_global


def do_switch_local(switcher, current_local):
    """Interactively switch the local planner. Returns the new name (unchanged on invalid input)."""
    print("\n  Local Planners:")
    for k, (_, name) in LOCAL_PLANNERS.items():
        mark = " (current)" if name == current_local else ""
        print(f"   {k}. {name}{mark}")
    llen = len(LOCAL_PLANNERS)
    sel = input(f"  Select [1-{llen}]: ").strip()
    if sel in LOCAL_PLANNERS:
        plugin_id, name = LOCAL_PLANNERS[sel]
        print(f"  Switching local planner to {name}...")
        ok, msg = switcher.switch_local_planner(plugin_id)
        if ok:
            print(f"  [OK] {msg}")
            return name
        print(f"  [FAIL] {msg}")
    else:
        print("  Invalid selection")
    return current_local


def main():
    init_args = list(sys.argv)
    if not any('use_sim_time' in arg for arg in init_args):
        init_args.extend(['--ros-args', '-p', 'use_sim_time:=true'])
    rclpy.init(args=init_args)

    monitor = CostmapMonitor()
    monitor_executor = SingleThreadedExecutor()
    monitor_executor.add_node(monitor)
    spin_thread = threading.Thread(target=monitor_executor.spin, daemon=True)
    spin_thread.start()

    # Separate node for emergency stop publisher (fixes getNode() error)
    cmd_node = Node('menu_cmd_pub')
    cmd_pub = cmd_node.create_publisher(TwistStamped, '/cmd_vel', 10)
    cmd_executor = SingleThreadedExecutor()
    cmd_executor.add_node(cmd_node)
    cmd_thread = threading.Thread(target=cmd_executor.spin, daemon=True)
    cmd_thread.start()

    # Planner switcher node (background spin for DDS discovery/persistence)
    switcher = PlannerSwitcher()
    switcher_executor = SingleThreadedExecutor()
    switcher_executor.add_node(switcher)
    switcher_thread = threading.Thread(target=switcher_executor.spin, daemon=True)
    switcher_thread.start()

    nav = BasicNavigator()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    rclpy.spin_once(nav, timeout_sec=0.1)
    now_msg = monitor.get_clock().now().to_msg()
    if now_msg.sec == 0 and now_msg.nanosec == 0:
        time.sleep(0.5)
        now_msg = monitor.get_clock().now().to_msg()
    initial_pose.header.stamp = now_msg
    initial_pose.pose.position.x = DEFAULT_SPAWN[0]
    initial_pose.pose.position.y = DEFAULT_SPAWN[1]
    initial_pose.pose.orientation.w = 1.0

    print("\n  Initializing Navigation Menu...")
    print(f"  Setting Initial Pose: ({DEFAULT_SPAWN[0]}, {DEFAULT_SPAWN[1]})")
    nav.setInitialPose(initial_pose)

    print("  Waiting for Nav2...")
    try:
        nav.waitUntilNav2Active(localizer='amcl')
        print("  Nav2 Active\n")
    except Exception as e:
        print(f"  [WARN] {e}")
        print("  Proceeding...\n")

    current_goal = None
    goal_announced = True
    current_global = 'A*'
    current_local = 'MPPI'

    try:
        while rclpy.ok():
            pos_str = f"({monitor.current_pos[0]:.2f}, {monitor.current_pos[1]:.2f})" \
                if monitor.current_pos else "waiting for /odom..."
            heading_str = f"{math.degrees(monitor.current_yaw):.1f}°"
            goal_str = f"({current_goal[0]:.2f}, {current_goal[1]:.2f})" \
                if current_goal else "none"
            map_status = "global costmap" if monitor.map_msg else ("static map (fallback)" if monitor.static_map_msg else "waiting...")

            print(f"\n  Position: {pos_str}")
            print(f"  Heading : {heading_str}")
            print(f"  Goal    : {goal_str}")
            print(f"  Map     : {map_status}")
            print(f"  Planners: Global={current_global} | Local={current_local}")

            if current_goal is not None and monitor.current_pos is not None:
                d = math.hypot(current_goal[0] - monitor.current_pos[0],
                               current_goal[1] - monitor.current_pos[1])
                if d < GOAL_TOLERANCE and not goal_announced:
                    print("  >>> GOAL REACHED <<<")
                    goal_announced = True

            print("\n  1. Send Goal")
            print("  2. Emergency Stop")
            print("  3. Switch Global Planner")
            print("  4. Switch Local Planner")
            print("  5. Exit")
            choice = input("\n  Select [1-5]: ").strip()

            if choice == '1':
                print()
                x = prompt_float("X")
                y = prompt_float("Y")

                ok, reason = monitor.validate_goal(x, y)
                if not ok:
                    print(f"\n  [REJECTED] ({x:.2f}, {y:.2f}) - {reason}")
                    continue

                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.header.stamp = monitor.get_clock().now().to_msg()
                goal.pose.position.x = x
                goal.pose.position.y = y
                goal.pose.orientation.w = 1.0

                print(f"\n  [MISSION] Pathfinding to ({x}, {y}) via {current_global}...")
                nav.goToPose(goal)
                current_goal = (x, y)
                goal_announced = False
                stopped = False
                exit_requested = False

                # Mission loop: keep accepting keyboard input so that 2 (and 3/4/5)
                # work while the robot is moving, not only when idle.
                while not nav.isTaskComplete():
                    fb = nav.getFeedback()
                    if fb is not None:
                        sys.stdout.write(
                            f"\r  Distance: {fb.distance_remaining:6.2f} m    "
                            "[2]=STOP [3]/[4]=switch [5]=exit   ")
                        sys.stdout.flush()

                    if select.select([sys.stdin], [], [], 0.0)[0]:
                        key = sys.stdin.readline().strip()
                        if key == '2':
                            print("\n  [STOP] Emergency Stop...")
                            nav.cancelTask()
                            zero = TwistStamped()
                            zero.header.stamp = monitor.get_clock().now().to_msg()
                            zero.header.frame_id = 'base_link'
                            cmd_pub.publish(zero)
                            stopped = True
                            current_goal = None
                            goal_announced = True
                        elif key == '3':
                            current_global = do_switch_global(switcher, current_global)
                        elif key == '4':
                            current_local = do_switch_local(switcher, current_local)
                        elif key == '5':
                            print("\n  Exiting...")
                            nav.cancelTask()
                            exit_requested = True
                            break
                        else:
                            print(f"\n  [INFO] Unknown key during mission: '{key}' (press 2 to stop)")
                    time.sleep(0.25)

                if exit_requested:
                    break

                result = nav.getResult()
                if stopped:
                    print("\n  [STOP] Robot stopped, mission canceled")
                elif result == TaskResult.SUCCEEDED:
                    print("\n  [SUCCESS] Goal Reached")
                    current_goal = (x, y)
                    goal_announced = True
                elif result == TaskResult.CANCELED:
                    print("\n  [INFO] Mission Canceled")
                elif result == TaskResult.FAILED:
                    print("\n  [ERROR] Mission Failed")


            elif choice == '2':
                nav.cancelTask()
                zero = TwistStamped()
                zero.header.stamp = monitor.get_clock().now().to_msg()
                zero.header.frame_id = 'base_link'
                cmd_pub.publish(zero)
                current_goal = None
                goal_announced = True
                print(f"\n  [STOP] Emergency Stop - Goal cleared, heading {heading_str}")

            elif choice == '3':
                current_global = do_switch_global(switcher, current_global)

            elif choice == '4':
                current_local = do_switch_local(switcher, current_local)

            elif choice == '5':
                print("\n  Exiting...")
                shutdown_simulation()
                break
            else:
                print("  Invalid option")
    except KeyboardInterrupt:
        print("\n  Interrupted")
        shutdown_simulation()

    rclpy.shutdown()
    monitor_executor.shutdown()
    switcher_executor.shutdown()
    cmd_executor.shutdown()
    spin_thread.join(timeout=1.0)
    sys.exit(0)


if __name__ == '__main__':
    main()