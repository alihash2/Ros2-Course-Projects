# ms04_autonomous_navigation

ROS 2 package for autonomous navigation with GUI interfaces across custom Gazebo environments (Office and Warehouse).

## Features
- Custom Gazebo simulation worlds for Office and Warehouse layouts.
- Autonomous SLAM mapping and automated map saving.
- Environment-specific AMCL localization and Nav2 tuning (office vs warehouse).
- Modular launch files: a shared Nav2 server stack + per-phase/environment bringups.
- Unified Nav2 coordinator supporting single Go-To-Pose and Multi-Waypoint routes from a single command.
- Action-server event logging for all lifecycle transitions.
- Controls for starting, cancelling, pausing, resuming, and replacing goals.
- PyQt5 Mission Control GUI with real-time navigation logging.

## Custom Interfaces

### Messages
- `NavigationMission.msg`: Unified mission command message supporting `MODE_GO_TO_POSE` (0) and `MODE_WAYPOINTS` (1), along with execution control commands (`COMMAND_START`, `COMMAND_CANCEL`, `COMMAND_PAUSE`, `COMMAND_RESUME`, `COMMAND_REPLACE`).
- `NavigationEvent.msg`: Telemetry and lifecycle event message (`EVENT_GOAL_SUBMITTED`, `EVENT_GOAL_ACCEPTED`, `EVENT_GOAL_REJECTED`, `EVENT_FEEDBACK`, `EVENT_GOAL_COMPLETED`, `EVENT_GOAL_CANCELED`, `EVENT_GOAL_ABORTED`, `EVENT_GOAL_PAUSED`, `EVENT_GOAL_RESUMED`, `EVENT_GOAL_REPLACED`).
- `NavigationStatus.msg`: Live status snapshot of the coordinator (`STATE_IDLE`, `STATE_NAVIGATING`, `STATE_PAUSED`, `STATE_COMPLETED`, `STATE_CANCELED`, `STATE_ABORTED`) with current pose, remaining distance, ETA and waypoint indices.

### Actions
- `ExecuteMission.action`: ROS 2 Action interface for mission execution with real-time feedback (current pose, distance remaining, ETA, recovery count, waypoint indices).

## Navigation Coordinator Node

`navigation_coordinator_node` is the unified dispatcher: it receives mission
commands, dispatches them to the matching Nav2 navigator (single pose →
`bt_navigator`/`NavigateToPose`, multi-waypoint → `waypoint_follower`/
`FollowWaypoints`), implements the full control set, and logs every lifecycle
transition and telemetry update.

### Interface
| Direction | Topic / Action | Interface |
|-----------|----------------|-----------|
| in | `/navigation/mission` (user commands) | `NavigationMission` |
| out | `/navigation/events` (lifecycle + telemetry log) | `NavigationEvent` |
| out | `/navigation/status` (state snapshot) | `NavigationStatus` |
| in | `/execute_mission` (action server) | `ExecuteMission` |
| out (action client) | `/navigate_to_pose` | nav2 `NavigateToPose` |
| out (action client) | `/follow_waypoints` | nav2 `FollowWaypoints` |

### Control commands
- **Start** — dispatch a `Go-To-Pose` or `Waypoints` mission; rejected with an
  event if a mission is already active (use Replace to preempt).
- **Cancel** — terminate the active goal; the mission ends in `CANCELED`.
  Works on navigating missions and on missions paused mid-route.
- **Pause** — cancels the Nav2 goal and preserves the *remaining* waypoints
  (or the held target pose); the mission holds in `PAUSED`.
- **Resume** — re-dispatches the preserved goal from the pause point.
- **Replace** — preempts the active goal and dispatches the new mission
  (`EVENT_GOAL_REPLACED`).

### Invocation
```bash
# terminal echo of all events and state
ros2 topic echo /navigation/events ms04_autonomous_navigation/msg/NavigationEvent
ros2 topic echo /navigation/status ms04_autonomous_navigation/msg/NavigationStatus

# publish a go-to-pose mission
ros2 topic pub --once /navigation/mission ms04_autonomous_navigation/msg/NavigationMission \
  "{header: {stamp: {sec: 0}, frame_id: 'map'}, mission_id: 'demo', mode: 0, \
    command: 0, target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 2.0}, orientation: {w: 1.0}}}}"

# pause / resume / cancel while it runs (command enum: 0 start, 1 cancel, 2 pause, 3 resume, 4 replace)
ros2 topic pub --once /navigation/mission ms04_autonomous_navigation/msg/NavigationMission "{header: {stamp: {sec: 0}, frame_id: 'map'}, command: 2}"

# or use the action server with full feedback
ros2 action send_goal /execute_mission ms04_autonomous_navigation/action/ExecuteMission \
  "{mission_id: 'demo', mode: 0, target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 2.0}, orientation: {w: 1.0}}}}"
```

### Nav2 launch notes
`launch/nav2_servers.launch.py` is the shared, localization-agnostic Nav2
server stack used by **both** the mapping and the navigation launches. It
starts `bt_navigator` (`NavigateToPose`) and `waypoint_follower`
(`FollowWaypoints`), plus the controller/planner/smoother/behavior servers,
`velocity_smoother` and `collision_monitor`, all managed by the auto-started
`lifecycle_manager_navigation`. Its `params_file` argument selects the phase
and environment profile; `/map` and `map->odom` come from slam_toolbox during
mapping and from `map_server` + AMCL during navigation.

## Environment Navigation (AMCL)

Once a map exists (`maps/office_map.yaml` / `maps/warehouse_map.yaml`), the
navigation phase localizes with **AMCL** against that static map instead of
SLAM. Each environment has its own tuned profile and bringup launch:

| Environment | Params profile | Launch |
|-------------|----------------|--------|
| Office (narrow corridors, doorways, tight turns) | `params/office_nav2_params.yaml` | `launch/office_navigation.launch.py` |
| Warehouse (long straights, pallet clearance, higher speed) | `params/warehouse_nav2_params.yaml` | `launch/warehouse_navigation.launch.py` |

Each launch starts Gazebo + `nav2_servers` (env params) + `map_server` +
`amcl` (localization lifecycle) + the issue-4 `navigation_coordinator_node` +
RViz (`rviz/nav2_gui_view.rviz`).

```bash
# Office navigation (add gui:=false to run headless)
ros2 launch ms04_autonomous_navigation office_navigation.launch.py

# Warehouse navigation
ros2 launch ms04_autonomous_navigation warehouse_navigation.launch.py
```

### Tuning philosophy
- **Office = complex tune** — `max_vel_x 0.5`, `max_vel_theta 1.2`, larger
  global inflation (`0.8`) and `xy_goal_tolerance 0.25` for doorway-level
  precision; extra recovery headroom (`movement_time_allowance 10 s`).
- **Warehouse = fast tune** — `max_vel_x 0.9`, `max_vel_theta 1.8`, higher
  accel/decel, longer `sim_time 2.2`, larger local inflation (`0.6`) to sweep
  wide of pallet racks.
- Both set AMCL `set_initial_pose: true` at `(0,0,0)`; the saved maps are
  anchored at the robot spawn, so no manual 2D Pose Estimate is required.
- The RViz profile shows robot model, global/local costmaps, global/local
  planned paths, waypoint markers (published by the coordinator on
  `/navigation/waypoints_marker`) and the 2D Pose Estimate / 2D Goal tools.

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