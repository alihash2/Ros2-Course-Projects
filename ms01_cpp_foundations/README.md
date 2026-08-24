A C++ mobile robot library providing simulated sensor outputs (Wheel Speeds, Odometry, and LiDAR) using modern C++17 and CMake.

> This package is part of the [Ros2-Course-Projects](../README.md) repo — see the top-level README for workspace setup and cloning instructions.

Directory Structure:
* `include/` - Header declarations (`.hpp`) and data structures.
* `src/` - Implementation of robot logic and sensor simulation.
* `apps/` - Example driver application (`main.cpp`).

Instructions:
1. Download the repo or git clone on your terminal using the link given on the repo page.
2. Open the root directory or type "cd LearnGit"
3. On your terminal, type these commands from the root directory.

	mkdir -p build && cd build
	cmake ..
	make
	./mobile_robot_app
