# Dynamic Obstacle Avoidance (`dynamic_obstacle_avoidance`)

A ROS 2 (Jazzy) package providing **custom Nav2 global planner plugins** (A\* and RRT) for the TurtleBot3 Waffle in the `turtlebot3_world` Gazebo environment, with full Nav2 stack integration, RViz visualization, and an interactive CLI navigation menu.

> This package is part of the [Ros2-Course-Projects](../README.md) repo — see the top-level README for workspace setup and cloning instructions.

---

## 📦 What's Inside

| Component | Description |
|---|---|
| `src/astar_planner.cpp` / `astar_planner_plugin.cpp` | A\* global planner as a `nav2_core::GlobalPlanner` plugin |
| `src/rrt_planner.cpp` / `rrt_planner_plugin.cpp` | RRT global planner plugin |
| `src/cmd_vel_relay.cpp` | Relays Nav2's `TwistStamped` to the `/cmd_vel` topic Gazebo expects |
| `src/path_follower_node.cpp` | Optional custom PID/LQR path follower (`enable_custom_follower:=true`) |
| `src/navigation_menu.py` | Interactive CLI mission-control menu (Nav2 Simple Commander) |
| `params/` | Nav2 parameter files (`nav2_params_astar.yaml` default, plus RRT variant) |
| `launch/navigation.launch.py` | One-command launch of Gazebo + Nav2 + RViz |

---

## ✅ Prerequisites

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-turtlebot3-gazebo \
                 ros-$ROS_DISTRO-nav2-bringup \
                 ros-$ROS_DISTRO-nav2-simple-commander -y
```

## 🔨 Build

```bash
cd ~/ros2_ws
colcon build --packages-select dynamic_obstacle_avoidance
source install/setup.bash
```

---

## 🚀 Running

### Terminal 1 — Launch the simulation + Nav2 + RViz

```bash
ros2 launch dynamic_obstacle_avoidance navigation.launch.py
```

This starts:
- **Gazebo** with the TurtleBot3 Waffle in `turtlebot3_world` (default spawn pose: **x = -2.0, y = -0.5**)
- **Nav2** (AMCL, planners, controller, behaviors) using this package's params
- **RViz** with the standard Nav2 view
- The `cmd_vel_relay` node

### Automatic localization at spawn (no manual 2D Pose Estimate needed)

AMCL is configured with `set_initial_pose: true` and the initial pose hardcoded to the robot's default spawn location `(-2.0, -0.5)` in all param files. When RViz opens, the robot is already localized on the map — **you do not need to press "2D Pose Estimate"**.

### Terminal 2 — Launch the interactive navigation menu

```bash
ros2 run dynamic_obstacle_avoidance navigation_menu.py
```

The menu:

1. Sets the initial pose to the default spawn location `(-2.0, -0.5)` via Nav2 Simple Commander.
2. Waits until Nav2 is fully active.
3. Presents options:
   - **1 — Send Goal**: enter target X and Y. **Every coordinate is validated against the live global costmap before the robot moves** (see below). If valid, the path is computed with the selected planner and the robot navigates there, showing live distance-to-goal feedback. The menu also shows current position and current goal at all times, and prints `>>> GOAL REACHED! <<<` once when the robot arrives.
   - **2 — Emergency Stop**: cancels the active navigation task.
   - **3 — Exit**: closes the menu **and shuts down Gazebo, RViz, Nav2 and the launch terminal automatically** — no Ctrl+C needed in Terminal 1.

### 🛡️ Goal validation & limits

The world walls are irregular, so instead of a hard-coded square limit the menu checks every entered goal against the **live `/global_costmap/costmap`** and rejects it with an explicit reason if:

- it lies **outside the map** (beyond the enclosed world walls) — the menu prints the actual map bounds from the costmap;
- the spot is occupied by an **obstacle or wall** (lethal cost cell, including its immediate neighbours so goals hugging obstacles are rejected too);
- the spot is in **unknown/unexplored space**.

As a rough guide while typing, the menu suggests the safe range of about **−2 to +2 on X and Y around the origin**, but the costmap check is the real authority — a goal like `(3.5, 0.1)` inside a wall will be rejected with `[REJECTED] ... outside the map!` before anything moves.

### Useful launch arguments

```bash
# Use the RRT planner instead of A*
ros2 launch dynamic_obstacle_avoidance navigation.launch.py params_file:=<path>/nav2_params_rrt.yaml

# Run the custom PID/LQR path follower instead of the Nav2 controller
ros2 launch dynamic_obstacle_avoidance navigation.launch.py enable_custom_follower:=true

# Use a different map
ros2 launch dynamic_obstacle_avoidance navigation.launch.py map:=/path/to/map.yaml
```

### Manual goal publishing (optional)

You can still send goals without the menu:

```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}}}" --once
```

---

## 🗺️ World & Spawn Reference

- **World:** `turtlebot3_world` — roughly bounded by **X: -4..+4**, **Y: -4..+4** around the origin; practically navigable range is about **±2 m** from the origin due to irregular walls and obstacles (enforced per-goal via the costmap check).
- **Default spawn pose:** `(-2.0, -0.5)`, yaw 0.
- AMCL initial pose and the menu's initial pose both match this spawn location automatically.

## 🧠 How It Works

- The custom planner plugins are loaded through `pluginlib` (see `plugins/planner_plugin.xml`) and selected in the Nav2 params under `planner_server`.
- Plans are computed on the global costmap inflated from the static map + live obstacles; the controller server tracks the path while avoiding dynamic obstacles.
- `cmd_vel_relay` bridges Nav2's `TwistStamped` output onto the plain `Twist` `/cmd_vel` topic consumed by the Gazebo diff-drive bridge.
- The menu's goal validation subscribes to `/global_costmap/costmap` (`OccupancyGrid`) and converts world coordinates → grid cells using the map's origin/resolution, rejecting lethal (>90 occupancy) and unknown (-1) cells plus their 8-neighbourhood.
