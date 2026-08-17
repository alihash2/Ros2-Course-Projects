# MS02 Mobile Robot Control (`ms02_mobile_robot_control`)

A ROS 2 C++ package implementing closed-loop proportional position and heading control for a mobile robot operating inside the `turtlesim` simulation environment.

---

## 1. Prerequisites & Dependencies

Ensure your environment meets the following baseline dependencies before building:

* **Operating System:** Ubuntu 22.04 LTS (or compatible Linux distribution)
* **ROS 2 Distribution:** Humble Hawksbill (or newer, e.g., Jazzy / Rolling)
* **Build System:** `colcon` with `cmake` and `ament_cmake`
* **Compiler:** C++17 capable compiler (`g++` or `clang`)

### Required ROS 2 Packages
Install standard dependencies via `apt` if not already installed:

```bash
sudo apt update
sudo apt install -y ros-$ROS_DISTRO-turtlesim ros-$ROS_DISTRO-geometry-msgs ros-$ROS_DISTRO-rclcpp
 
```

## 2. Directory Structure Setup
This workspace assumes your ROS2 workspace is locatioed at ~/ros2_ws
File structure is:
```bash
~/ros2_ws/src/ms02_mobile_robot_control/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/
│   └── ms02_mobile_robot_control/
│       └── goto_pose_node.hpp
└── src/
    ├── goto_pose_node.cpp
    └── goto_pose_node_app.cpp
```


## 3. Building the Package

1. Naviagte to Workspace root
   ```bash
    cd ~/ros2_ws
   ```
3. Build package with colcon
   ```bash
    colcon build --packages-select ms02_mobile_robot_control
   ```
5. Source the workspace overlay
   ```bash
    source install/setup.bash
   ```

## 4. Running the Control App
1. On a separate terminal (terminal1), launch turtlesim:
   ```bash
    ros2 run turtlesim turtlesim_node
   ```
3. On a separate terminal (terminal2), luanch program with commands direclty passed into main:
   ```bash
    ros2 run ms02_mobile_robot_control goto_pose_app <target_x> <target_y> <target_theta>
   ```

(Replace <> placeholders with desired X and Y coordindate and Theta orientation in radians respectively. Default values are 7.0, 7.0, 0.0)
A sample input command will look like this: ```
    ros2 run ms02_mobile_robot_control goto_pose_app 9.0 9.0 0.78  
    ```

Press Cntrl+ C to exit subscription and run again to issue a different target heading.
