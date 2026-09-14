# ms04_autonomous_navigation

ROS 2 package for autonomous navigation with GUI interfaces across custom Gazebo environments (Office and Warehouse).

## Features
- Custom Gazebo simulation worlds for Office and Warehouse layouts.
- Autonomous SLAM mapping and automated map saving.
- Unified Nav2 coordinator supporting single Go-To-Pose and Multi-Waypoint routes from a single command.
- Action-server event logging for all lifecycle transitions.
- Controls for starting, cancelling, pausing, resuming, and replacing goals.
- PyQt5 Mission Control GUI with real-time navigation logging.

## Custom Interfaces

### Messages
- `NavigationMission.msg`: Unified mission command message supporting `MODE_GO_TO_POSE` (0) and `MODE_WAYPOINTS` (1), along with execution control commands (`COMMAND_START`, `COMMAND_CANCEL`, `COMMAND_PAUSE`, `COMMAND_RESUME`, `COMMAND_REPLACE`).
- `NavigationEvent.msg`: Telemetry and lifecycle event message (`EVENT_GOAL_SUBMITTED`, `EVENT_GOAL_ACCEPTED`, `EVENT_GOAL_REJECTED`, `EVENT_FEEDBACK`, `EVENT_GOAL_COMPLETED`, `EVENT_GOAL_CANCELED`, `EVENT_GOAL_ABORTED`, `EVENT_GOAL_PAUSED`, `EVENT_GOAL_RESUMED`, `EVENT_GOAL_REPLACED`).

### Actions
- `ExecuteMission.action`: ROS 2 Action interface for mission execution with real-time feedback (current pose, distance remaining, ETA, recovery count, waypoint indices).
