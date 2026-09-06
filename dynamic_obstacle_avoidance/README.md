# Dynamic Obstacle Avoidance (`dynamic_obstacle_avoidance`)

ROS 2 (Jazzy) package providing **custom Nav2 global planner plugins** (A* and RRT) for TurtleBot3 Waffle in `turtlebot3_world` Gazebo environment, with full Nav2 stack integration, RViz visualization, and an interactive CLI navigation menu.

> This package is part of the [Ros2-Course-Projects](../README.md) repo — see the top-level README for workspace setup and cloning instructions.

---

## Components

| Component | Description |
|-----------|-------------|
| `astar_planner.cpp` / `astar_planner_plugin.cpp` | A* global planner as `nav2_core::GlobalPlanner` plugin |
| `rrt_planner.cpp` / `rrt_planner_plugin.cpp` | RRT global planner plugin |
| `cmd_vel_relay.cpp` | Relays Nav2's `TwistStamped` to `/cmd_vel` for Gazebo |
| `path_follower_node.cpp` | Legacy standalone PID/LQR follower (replaced by the `PidLqrController` plugin; kept for reference) |
| `pid_lqr_controller.cpp` | PID heading + LQR velocity **Nav2 controller plugin** (`dynamic_obstacle_avoidance/PidLqrController`) - switchable at runtime |
| `navigation_menu.py` | Interactive CLI mission-control menu (Nav2 Simple Commander) |
| `params/` | Nav2 parameter files (`nav2_params_astar.yaml` default, plus RRT variant) |
| `launch/navigation.launch.py` | One-command launch of Gazebo + Nav2 + RViz |

---

## Prerequisites

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-turtlebot3-gazebo \
                 ros-$ROS_DISTRO-nav2-bringup \
                 ros-$ROS_DISTRO-nav2-simple-commander -y
```

---

## Workspace Setup & Build

```bash
# 1. Create workspace and clone
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/alihash2/Ros2-Course-Projects.git

# 2. Build the package
cd ~/ros2_ws
colcon build --packages-select dynamic_obstacle_avoidance
source install/setup.bash
```

> **Every new terminal must source the workspace:**
> ```bash
> source ~/ros2_ws/install/setup.bash
> ```

---

## Running

### Terminal 1 — Launch Simulation + Nav2 + RViz

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch dynamic_obstacle_avoidance navigation.launch.py
```

This starts:
- Gazebo with TurtleBot3 Waffle in `turtlebot3_world` (spawn: -2.0, -0.5)
- Nav2 (AMCL, planners, controller, behaviors) with this package's params
- RViz with standard Nav2 view
- `cmd_vel_relay` node

**Auto-localization**: AMCL is configured with `set_initial_pose: true` at the spawn location (-2.0, -0.5). Robot is localized on map when RViz opens — no manual "2D Pose Estimate" needed.

### Terminal 2 — Interactive Navigation Menu

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run dynamic_obstacle_avoidance navigation_menu.py
```

The menu:
1. Sets initial pose to spawn location (-2.0, -0.5)
2. Waits for Nav2 to activate
3. Presents options:
```
  1. Send Goal
  2. Emergency Stop
  3. Switch Global Planner
  4. Switch Local Planner
  5. Exit
```

**Coordinates**: Enter X and Y in WORLD frame (map center = 0,0). Every goal is validated against the live global costmap before the robot moves.

**Global Planner switching** (runtime):
- **A*** (`dynamic_obstacle_avoidance/AStarPlannerPlugin`) - optimal grid search
- **RRT** (`dynamic_obstacle_avoidance/RRTPlannerPlugin`) - random sampling

**Local Planner switching** (runtime):
- **MPPI** (`nav2_mppi_controller::MPPIController`) - sampling-based MPC, costmap-aware obstacle avoidance
- **DWB** (`dwb_core::DWBLocalPlanner`) - Dynamic Window Approach, sampling-based local planner
- **PID+LQR** (`dynamic_obstacle_avoidance/PidLqrController`) - pure-pursuit style PID heading + LQR velocity tracking (no costmap-aware avoidance; fast, simple path following)

The menu shows the currently active planners in the status header.

**How runtime switching works**: the menu publishes the selected plugin ID to the `/planner_selector` and `/controller_selector` topics (`std_msgs/String`, **latched / transient-local** QoS). The BT nodes `PlannerSelector` / `ControllerSelector` in Nav2's default behavior tree subscribe to these topics and switch the plugins used for each mission. All candidate plugins must be listed in `planner_plugins` / `controller_plugins` so they are loaded at startup and selectable at runtime. (Note: Nav2 Jazzy does **not** expose `nav2_msgs/srv/ChangePlugin`.)

> DWB plugin type is `dwb_core::DWBLocalPlanner`. A wrong type string made `controller_server` crash at startup, which previously broke RViz control and auto-localization — if you edit `params/*.yaml`, keep the plugin IDs above exactly.

**Validation checks** (rejected with reason):
- Outside map bounds (beyond enclosed world walls)
- On obstacle/wall (lethal cost cell or neighbors)
- In unknown/unexplored space

**Emergency Stop**: Press **2 at any time** — including *while a mission is running*. It cancels the active navigation goal and publishes zero velocity to `/cmd_vel` immediately. The mission loop keeps polling the keyboard, so you don't need to wait for the goal to finish to stop the robot. (3/4/5 also work mid-mission: they pre-select the planner for the next goal or exit.)

**Exit**: Shuts down Gazebo, RViz, Nav2, and launch terminal automatically.

---

## Launch Arguments

```bash
# Use RRT planner instead of A*
ros2 launch dynamic_obstacle_avoidance navigation.launch.py params_file:=<path>/nav2_params_rrt.yaml

# Use different map
ros2 launch dynamic_obstacle_avoidance navigation.launch.py map:=/path/to/map.yaml

# Legacy: run the standalone PID/LQR follower (NOT recommended - use the plugin instead)
ros2 launch dynamic_obstacle_avoidance navigation.launch.py enable_custom_follower:=true
```

Both `nav2_params_astar.yaml` and `nav2_params_rrt.yaml` now configure both global planners (A\* and RRT) and three local controllers (MPPI, DWB and PID+LQR), so they can be switched at runtime from the menu. The default behavior tree keeps **A\* (`GridBased`) + MPPI (`FollowPath`)** as the startup default — identical to the original working setup.

---

## Manual Goal Publishing (Alternative)

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}}}" --once
```

---

## Coordinate Frames

| Frame | Robot Spawn | World Origin (map center) |
|-------|-------------|---------------------------|
| **WORLD / what you type** | `(-2.0, -0.5)` | `(0, 0)` |
| **Odom (`/odom` raw)** | `(0, 0)` | `(2.0, 0.5)` |

The menu works in WORLD coordinates. Conversion: `world = odom + spawn`.

---

## Troubleshooting

### Costmap not received / DDS port errors

If menu shows `costmap not received yet` or DDS port errors:

1. **Sourcing** — Run `source ~/ros2_ws/install/setup.bash` in **every** terminal
2. **Folder structure** — Packages should be directly in `src/` (e.g., `~/ros2_ws/src/dynamic_obstacle_avoidance/`)
3. **ROS_DOMAIN_ID** — All terminals must share same domain ID (default 0). Check: `echo $ROS_DOMAIN_ID`
4. **ROS_LOCALHOST_ONLY** — Set `export ROS_LOCALHOST_ONLY=1` on VPNs/firewalls/multi-host setups
5. **Timing** — Wait 2-3 seconds after Nav2 starts for costmap to become available

**Quick check**: Run `ros2 node list` in both terminals. If menu terminal only shows `navigation_menu` but sim terminal shows Nav2 nodes, sourcing is missing in menu terminal.

---

## World & Spawn Reference

- **World**: `turtlebot3_world` — roughly bounded by X: -4..+4, Y: -4..+4; practically navigable ±2 m from origin
- **Default spawn**: `(-2.0, -0.5)`, yaw 0
- AMCL initial pose and menu initial pose both match spawn automatically