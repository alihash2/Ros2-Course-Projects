# MS02 Mobile Robot Control (`ms02_mobile_robot_control`)

A ROS 2 C++ package implementing closed-loop proportional position and heading control for a mobile robot operating inside the `turtlesim` simulation environment.

---

## 1. Prerequisites & Dependencies

* **Operating System:** Ubuntu 22.04 LTS (or compatible Linux distribution)
* **ROS 2 Distribution:** Humble Hawksbill (or newer, e.g., Jazzy / Rolling)
* **Build System:** `colcon` with `ament_cmake`
* **Compiler:** C++17 capable compiler

### Required ROS 2 Packages

```bash
sudo apt update
sudo apt install -y ros-$ROS_DISTRO-turtlesim ros-$ROS_DISTRO-geometry-msgs ros-$ROS_DISTRO-rclcpp
```

## 2. Directory Structure

```bash
~/ros2_ws/src/ms02_mobile_robot_control/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/
│   └── ms02_mobile_robot_control/
│       └── goto_pose_node.hpp
├── src/
│   ├── goto_pose_node.cpp
│   └── (control node implementation)
└── apps/
    └── goto_pose_node_app.cpp
```

## 3. Building the Package

```bash
cd ~/ros2_ws
colcon build --packages-select ms02_mobile_robot_control
source install/setup.bash
```

## 4. Running the Interactive Control App (Recommended)

1. **Terminal 1** — launch turtlesim:
   ```bash
   ros2 run turtlesim turtlesim_node
   ```
2. **Terminal 2** — launch the interactive menu:
   ```bash
   ros2 run ms02_mobile_robot_control goto_pose_app
   ```

The app presents a CLI menu:

```
=============================================
   TURTLESIM GO-TO-POSE INTERACTIVE MENU
=============================================
Valid pose limits -> X: 0..10 | Y: 0..10
Theta is any angle in radians (it will be normalized).

---------------------------------------------
 1. Send turtle to a pose
 2. Exit
---------------------------------------------
Select option [1-2]:
```

- Choose `1`, then enter **X**, **Y** and **Theta** one at a time.
- X and Y are validated to be within **0 to 10 inclusive** — out-of-range or non-numeric input is rejected and re-prompted.
- Theta is accepted in the range -2π to +2π radians.
- The turtle navigates to the pose; once it arrives you get a success message and the menu reappears immediately so you can send another pose.
- Choose `2` (or press Ctrl+C / Ctrl+D) to exit.

### Behavior notes (important)

- **No movement at launch.** The turtle stays exactly where turtlesim spawns it (~5, 5) until you send your first pose — the app does not drive it to (0, 0).
- **"Goal reached" prints exactly once per pose command**, both in the menu (`[SUCCESS] Turtle reached the target pose!`) and in the node log (`Target pose successfully reached!`). No infinite log scrolling while the turtle sits at the goal.

### Example session

```
Select option [1-2]: 1
Enter X (range 0 to 10): 9.0
Enter Y (range 0 to 10): 9.0
Enter Theta (radians) (-6.28... to 6.28...): 0.78
Navigating to (9.0, 9.0, theta=0.78)...
[SUCCESS] Turtle reached the target pose!
```

## 5. Legacy Command-Line Mode (Still Supported)

You can still pass a target directly as arguments; the turtle will go there and the program exits when done:

```bash
ros2 run ms02_mobile_robot_control goto_pose_app 9.0 9.0 0.78
```

> Note: argument values are not range-checked in this mode — prefer the interactive menu.

---

## 6. How It Works

- `GoToPoseNode` subscribes to `/turtle1/pose`, publishes `Twist` commands on `/turtle1/cmd_vel`, and runs a ~30 Hz proportional control loop (linear gain 1.5, angular gain 4.0).
- Stage A drives toward (x, y); Stage B rotates in place to align with theta.
- Tolerances: 0.05 units distance, 0.02 rad heading.
- The node has an internal `has_target_` flag: until the app calls `setTarget()`, the control loop idles — this prevents both auto-movement at startup and spurious "goal reached" logs.
- The "goal reached" log is edge-triggered (fires once per target), so the terminal never floods with repeated messages.
- The interactive app spins the node on a background thread and uses `setTarget()` to re-task the same node without restarting, plus `isGoalReached()` to detect arrival.
