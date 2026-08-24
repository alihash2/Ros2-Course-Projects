# TurtleBot3 Behavior Tree Control Package (`ms03_turtlebot3_control`)

This package implements a **Reactive Behavior Tree Controller** for the TurtleBot3 in ROS 2. It integrates odometry tracking, LIDAR-based obstacle checking, and motion control into a unified navigation framework.

> This package is part of the [Ros2-Course-Projects](../README.md) repo — see the top-level README for workspace setup and cloning instructions.

---

## 🎯 The Big Picture Goal

The goal of this package is to guide the robot to a target goal position (`/user_goal`) while dynamically avoiding obstacles. 

### Features
- **Behavior Tree Logic**: Uses `behaviortree_cpp` to orchestrate navigation and recovery.
- **Reactive Obstacle Avoidance**: Continuously monitors LIDAR scans (±45° Front, ±45°–125° Sides) to interrupt movement when obstacles are detected.
- **In-Place Rotation Recovery**: Immediately pivots in-place upon obstacle detection (no reversal required), followed by a clearance verification step to ensure the obstacle is fully bypassed.
- **Active Speed Damping**: Automatically slows down as it approaches objects to ensure smooth, contactless maneuvering.

If an obstacle is detected in the front sector (less than 30cm), the behavior tree **interrupts** the normal `GoToPose` movement and switches to a **Turn Recovery** state. 

Once the path is clear, the robot resumes its path toward the target.

---

## 🛠️ Step-by-Step Installation & Setup

Follow these foolproof instructions to clone, build, and run this package from scratch.

### 1. Prerequisites
Ensure you have ROS 2 and the TurtleBot3 Gazebo simulation packages installed on your system.
```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-turtlebot3-gazebo ros-$ROS_DISTRO-behaviortree-cpp -y
```

### 2. Create and Clone the Workspace
If you do not have a workspace, create one and clone this repository into your `src` folder.

```bash
# Create workspace directories
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the repository (replace with your repository link if different)
# git clone <your-repo-link>
```

### 3. Build the Package
Navigate to the workspace root and build the packages using `colcon`.

```bash
cd ~/ros2_ws
colcon build --packages-select ms03_turtlebot3_control
```

---

## 🚀 How to Launch and Test

We have created a single-command launch file that starts the **Gazebo Simulator**, the **Odometry Processor**, the **LIDAR Scanner**, and the **Behavior Tree Brain**.

### Step 1: Export TurtleBot3 Model
You must tell ROS 2 which model of the TurtleBot3 you are simulating.
```bash
export TURTLEBOT3_MODEL=waffle
```

### Step 2: Run the Launch File
Run the single command to launch everything in one window.
```bash
# Go to workspace
cd ~/ros2_ws
# Source the workspace setup script
source install/setup.bash

# Run the launch file
ros2 launch ms03_turtlebot3_control turtlebot3_obstacle_avoidance.launch.py
```
*Gazebo will open, and you will see the logs from all three nodes (`odom_node`, `scan_node`, `bt_executor_node`) appearing in your terminal window.*

---

## 🎯 How to Send Navigation Goals

### Option A: Interactive Goal Menu (Recommended)

Open a **new terminal** and run:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run ms03_turtlebot3_control goal_menu.py
```

The menu greets you and shows, at all times, the robot's **current position** (converted to world coordinates) and the **current goal pose**:

```
=============================================
   TURTLEBOT3 GOAL SENDER — MS03 CONTROL
=============================================
World limits -> X: -4.0..4.0 | Y: -4.0..4.0
(coordinates are in WORLD frame, origin 0,0 at map center)
Robot default spawn pose: (-2.0, -0.5)

---------------------------------------------
 Current position : (-2.00, -0.50)
 Current goal     : (-2.00, -0.50)
---------------------------------------------
 1. Send a new goal pose
 2. Exit (also shuts down Gazebo & launch terminal)
Select option [1-2]:
```

- Choose `1` and enter **X** then **Y**. Values are validated against the world limits (**X: -4 to +4**, **Y: -4 to +4**) — invalid input is rejected and re-prompted.
- The script publishes a `PoseStamped` on `/user_goal` exactly like the manual `ros2 topic pub` command did.
- When the robot reaches within 15 cm of the goal, a **GOAL REACHED!** message appears in the menu (once per goal).
- Afterwards the menu reappears so you can send another goal or exit.
- **Choosing `2` shuts everything down**: it terminates Gazebo (`gz sim` / `gzserver` / `gzclient`) and the `ros2 launch` process in your other terminal, so both terminals close cleanly.

### ⚠️ Coordinate frames: WORLD vs ODOM (read this!)

The Gazebo↔ROS bridge publishes `/odom` relative to the robot's **spawn point**, so:

| Frame | Robot spawn | World origin (map center) |
|---|---|---|
| **World / Gazebo / what you type** | `(-2.0, -0.5)` | `(0, 0)` |
| **Odom (`/odom` raw)** | `(0, 0)` | `(2.0, 0.5)` |

Everything user-facing speaks **WORLD coordinates**: you type goals in world coordinates, the menu displays position in world coordinates, and limits are `-4..+4` around the world origin. The conversion happens automatically:

- `bt_executor_node` converts incoming `/user_goal` goals from world → odom (`odom = world − spawn`) before giving them to the behavior tree.
- `goal_menu.py` converts raw `/odom` readings from odom → world (`world = odom + spawn`) for display.

So if you want the robot at Gazebo's `(1.5, 1.0)`, just enter `1.5` and `1.0` in the menu — no mental math needed.

### Option B: Manual Topic Publish (Still Supported)

```bash
ros2 topic pub /user_goal geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}}}" --once
```

> **Note:** There is **no preloaded destination anymore**. On startup the behavior tree holds the robot's current pose (odom `(0,0)` == spawn point), so the robot stays put until you send a goal.

Watch the terminal logs from `bt_executor_node` and `scan_node` to see the robot navigate and react to any obstacles you place in its path inside Gazebo!
