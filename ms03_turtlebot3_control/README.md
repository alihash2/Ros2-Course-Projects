# TurtleBot3 Behavior Tree Control Package (`ms03_turtlebot3_control`)

This package implements a **Reactive Behavior Tree Controller** for the TurtleBot3 in ROS 2. It integrates odometry tracking, LIDAR-based obstacle checking, and motion control into a unified navigation framework.

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

Open a **new terminal** and publish a goal position to the `/user_goal` topic:

```bash
# Source your environment
source /opt/ros/$ROS_DISTRO/setup.bash

# Send target coordinates (e.g., x = 2.0, y = 1.0)
ros2 topic pub /user_goal geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}}}" --once
```

Watch the terminal logs from `bt_executor_node` and `scan_node` to see the robot navigate and react to any obstacles you place in its path inside Gazebo!
