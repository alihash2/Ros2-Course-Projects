# ROS 2 Course Projects

A collection of ROS 2 (Jazzy) packages built as course projects, covering everything from C++ fundamentals to behavior-tree robot control and custom Nav2 planner plugins.

---

## 📦 Packages in this Repo

| Folder | Package | What it does |
|---|---|---|
| `ms01_cpp_foundations` | `ms01_cpp_foundations` | Basic C++ / ROS 2 foundation exercises (publishers, subscribers, services). |
| `ms02_mobile_robot_control` | `ms02_mobile_robot_control` | Closed-loop go-to-pose control for **turtlesim** with an interactive CLI menu. Send the turtle to any `(x, y, theta)` within the 10×10 grid. |
| `ms03_turtlebot3_control` | `ms03_turtlebot3_control` | Reactive **Behavior Tree** controller for a TurtleBot3 Waffle in Gazebo (`turtlebot3_world`) — LIDAR obstacle detection, turn-in-place recovery, and an interactive goal-sending menu. |
| `dynamic_obstacle_avoidance` | `dynamic_obstacle_avoidance` | Custom **Nav2 global planner plugins** (A\* and RRT) for the TurtleBot3 in Gazebo, full Nav2 + RViz integration, costmap-validated goals, and a mission-control CLI menu. |

> Each package folder contains its own detailed `README.md` with prerequisites, build steps, run instructions, and usage examples. **Read the package's README before running it.**

---

## 🗂️ Repository Structure

This repo is meant to be cloned **inside** your ROS 2 workspace's `src/` folder. The layout looks like:

```
~/ros2_ws/                          <- your workspace root
├── src/                            <- all source packages live here
│   └── Ros2-Course-Projects/       <- THIS repository (cloned here)
│       ├── README.md               <- you are reading it
│       ├── ms01_cpp_foundations/
│       │   ├── CMakeLists.txt
│       │   ├── package.xml
│       │   ├── include/
│       │   └── src/
│       ├── ms02_mobile_robot_control/
│       │   ├── CMakeLists.txt
│       │   ├── package.xml
│       │   ├── apps/               <- interactive menu app
│       │   ├── include/
│       │   └── src/
│       ├── ms03_turtlebot3_control/
│       │   ├── bt_xml/             <- behavior tree definitions
│       │   ├── launch/             <- one-command sim launch files
│       │   ├── scripts/            <- Python CLI menus
│       │   ├── include/
│       │   └── src/
│       └── dynamic_obstacle_avoidance/
│           ├── launch/             <- Gazebo + Nav2 + RViz launch
│           ├── params/             <- Nav2 parameter files (A*/RRT)
│           ├── plugins/            <- pluginlib plugin descriptor
│           ├── src/                <- planner plugins + CLI menu
│           └── include/
├── install/                        <- created by colcon build (not in repo)
├── build/                          <- created by colcon build (not in repo)
└── log/                            <- created by colcon build (not in repo)
```

---

## 🚀 Getting Started (from scratch)

### 1. Install ROS 2

Follow the official instructions for your Ubuntu version: https://docs.ros.org/en/jazzy/Installation.html

These projects were developed on **ROS 2 Jazzy** on Ubuntu 24.04. Newer/older distros should work but are untested.

### 2. Create the workspace folder structure

If you don't already have a workspace, create it:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 3. Clone this repository into `src`

```bash
cd ~/ros2_ws/src
git clone https://github.com/alihash2/Ros2-Course-Projects.git
```

> Because the clone lands inside `src/`, colcon will discover every package automatically — no extra setup needed.

### 4. Source ROS 2 and install common dependencies

```bash
source /opt/ros/jazzy/setup.bash   # add to ~/.bashrc to make permanent
sudo apt update
sudo apt install -y \
    ros-$ROS_DISTRO-turtlesim \
    ros-$ROS_DISTRO-turtlebot3-gazebo \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-nav2-simple-commander \
    ros-$ROS_DISTRO-behaviortree-cpp \
    python3-numpy
export TURTLEBOT3_MODEL=waffle     # needed by ms03 & dynamic packages; add to ~/.bashrc
```

### 5. Build

Build everything at once:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Or build just one package (see that package's README for what it needs):

```bash
cd ~/ros2_ws
colcon build --packages-select <package_name>
source install/setup.bash
```

### 6. Run a package

Each package is independent — pick the one you want and follow its README:

- **Turtlesim go-to-pose menu** → [`ms02_mobile_robot_control/README.md`](ms02_mobile_robot_control/README.md)
- **Behavior tree TurtleBot3 control** → [`ms03_turtlebot3_control/README.md`](ms03_turtlebot3_control/README.md)
- **Custom A\*/RRT Nav2 planners** → [`dynamic_obstacle_avoidance/README.md`](dynamic_obstacle_avoidance/README.md)

Quick example (ms02):

```bash
# Terminal 1
ros2 run turtlesim turtlesim_node
# Terminal 2
ros2 run ms02_mobile_robot_control goto_pose_app
```

---

## 💡 Tips

- Always `source ~/ros2_ws/install/setup.bash` in every new terminal before running anything.
- If builds behave strangely after pulling changes: `rm -rf build install log && colcon build`.
- The simulation packages (ms03, dynamic) expect `TURTLEBOT3_MODEL=waffle` to be exported.
