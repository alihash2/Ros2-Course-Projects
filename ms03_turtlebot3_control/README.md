# TurtleBot3 Behavior Tree Control Package (`ms03_turtlebot3_control`)

This package implements a **Reactive Behavior Tree Controller** for the TurtleBot3 in ROS 2. It integrates odometry tracking, LIDAR-based obstacle checking, and motion control into a unified navigation framework.

> This package is part of the [Ros2-Course-Projects](../README.md) repo — see the top-level README for workspace setup and cloning instructions.

---

## Features

- **Behavior Tree Logic**: Uses `behaviortree_cpp` to orchestrate navigation and recovery.
- **Reactive Obstacle Avoidance**: Continuously monitors LIDAR scans to interrupt movement when obstacles are detected.
- **In-Place Rotation Recovery**: Immediately pivots upon obstacle detection, followed by clearance verification.
- **Active Speed Damping**: Automatically slows down near obstacles for smooth maneuvering.

---

## Prerequisites

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-turtlebot3-gazebo ros-$ROS_DISTRO-behaviortree-cpp -y
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
colcon build --packages-select ms03_turtlebot3_control
source install/setup.bash
```

> **Every new terminal must source the workspace:**
> ```bash
> source ~/ros2_ws/install/setup.bash
> ```

---

## Running

### Terminal 1 — Launch Simulation + Behavior Tree

```bash
cd ~/ros2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch ms03_turtlebot3_control turtlebot3_obstacle_avoidance.launch.py
```

This starts Gazebo with TurtleBot3 Waffle in `turtlebot3_world`, plus odometry, LIDAR scan, and behavior tree nodes.

### Terminal 2 — Interactive Goal Menu

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run ms03_turtlebot3_control goal_menu.py
```

The menu displays:
- Current robot position (WORLD frame)
- Current goal
- Distance to goal

Options:
```
  1. Send Goal
  2. Emergency Stop
  3. Exit
```

**Coordinates**: Enter X and Y in WORLD frame (map center = 0,0). Robot spawns at (-2.0, -0.5). Every goal is validated — invalid coordinates (out of bounds or on pillars) are rejected with a reason.

**Emergency Stop**: Commands robot to hold current position and clears the goal.

**Exit**: Shuts down Gazebo and the launch terminal automatically.

---

## Manual Goal Publishing (Alternative)

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub /user_goal geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}}}" --once
```

---

## Coordinate Frames

| Frame | Robot Spawn | World Origin (map center) |
|-------|-------------|---------------------------|
| **WORLD / what you type** | `(-2.0, -0.5)` | `(0, 0)` |
| **Odom (`/odom` raw)** | `(0, 0)` | `(2.0, 0.5)` |

The menu works in WORLD coordinates. Conversion happens automatically:
- `bt_executor_node` converts goals: world → odom (`odom = world - spawn`)
- `goal_menu.py` converts odom → world for display (`world = odom + spawn`)

---

## Validation Rules

Goals are rejected if:
- Outside world bounds (X: -4..4, Y: -4..4)
- On/near one of 5 pillars in `turtlebot3_world` at (0,0), (±1.1,0), (0,±1.1) with radius 0.35 m

Rejected goals are never published — the menu explains why and prompts again.