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

## Autonomous SLAM Exploration (primary mapping workflow)

Drives a TurtleBot3 Waffle through the world on a frontier-based exploration
strategy, with Nav2 owning all motion (planning, avoidance, recovery). On
completion the map is saved automatically.

- **Frontier detection & scoring** — classic three-term cost
  (`gain_scale*size + potential_scale*distance + orientation_scale*angle`,
  tuned `1.0 / 0.001 / 0.0`). Frontiers are clustered (BFS), scored, and the
  best is sent to Nav2 `NavigateToPose`.
- **Deep-room aiming** — goals are aimed at the centroid of the contiguous
  *unknown* region behind a frontier (BFS over unknown cells, radius 6 m) so a
  single visit with the 12 m lidar opens a whole room; falls back to a fixed
  `push_into_unknown` nudge.
- **Permanent blacklist** — reached and aborted goals are blacklisted
  (radius 1 m) so the robot never oscillates on a jammed pocket.
- **Completion phase** — once coverage ≥ `finalize_trigger_pct` (0.90) and map
  growth stalls (< 20 free cells / 5 s past a 30 s warmup), the explorer stops
  frontier chasing and targets the largest remaining *unknown* blobs until
  coverage ≥ `finalize_goal_pct` (0.95) or no reachable frontier remains.
- **Watchdogs** — `progress_timeout` (300 s) aborts a goal that never
  succeeds; a stuck-dog (45 s, < 1.2 m odometric travel) cancels goals where
  Nav2 is only spinning/backing in place.
- **Stale-map hygiene** — the target map files are removed at startup so every
  run starts clean; on finish the map is snapshotted with a timestamp plus a
  `.cfg` of the run config.

### Launch
```bash
# Office (16 x 14 m, spawn 0,0; census window x[-8,8] y[-7,7])
ros2 launch ms04_autonomous_navigation office_mapping.launch.py

# Warehouse (20 x 18 m, spawn -6,0; census window x[-10,10] y[-9,9])
ros2 launch ms04_autonomous_navigation warehouse_mapping.launch.py
```

Each mapping launch starts Gazebo + bridge + Nav2 (with SLAM) + RVis; the
explorer terminates itself on completion and saves to
- `maps/office_map.{pgm,yaml}`
- `maps/warehouse_map.{pgm,yaml}`

### Key tuning (params/)
`nav2_exploration_params.yaml`:
- Collision configs kept deliberately *snug* (BaseObstacle `scale: 0.02`,
  `robot_radius: 0.22`, local inflation `0.45`), because in this simulator a
  slight contact triggers long-lived jitter that visibly twists/overlaps the
  map. The **collision_monitor `FootprintApproach`** (`time_before_collision:
  0.6`) is the essential contact-prevention layer; wider margins hurt door
  reachability more than they help map quality.
- `initial_transform_timeout: 120.0` on both costmaps — Nav2 waits for
  SLAM's `map -> odom` instead of aborting its whole lifecycle bringup during
  slow starts (otherwise the map never loads).

`slam_toolbox_params.yaml`:
- `do_loop_closing: true` — required to remove the late-run global rotation /
  interior-wall overlap seen when loop closing was disabled. Fit in
  `loop_search_space_*` are the defaults.
- `max_laser_range: 12.0` (custom lidar model), mode `mapping`,
  `use_map_saver: true`.

`auto_slam_explorer.py` (scripts/) also exposes per-run params
(`progress_timeout`, `finalize_goal_pct`, office census bounds, etc.) via
launch.

### Verified results (final configuration)
| Map | reason | elapsed | office_known_pct | notes |
|-----|--------|---------|------------------|-------|
| Office run27 | no_reachable_frontier | 675 s | 92.8% | loop closing on, crisp walls |
| Warehouse run04 | no_reachable_frontier | 506 s | 92.8% | loop closing on, crisp walls |

Residual ~2% below the 95% ceiling is unreachable slivers, not a timeout
artifact.

## Simulation Environments

### Office Environment
Features a multi-room office layout with reception/lounge, conference room, open office cubicles, executive office, and central corridor.
```bash
ros2 launch ms04_autonomous_navigation office_simulation.launch.py
```

### Warehouse Environment
Features an industrial storage layout with 3 dual-sided high-bay shelving racks, staging areas, pallet stacks, cargo crates, and loading zones.
```bash
ros2 launch ms04_autonomous_navigation warehouse_simulation.launch.py
```