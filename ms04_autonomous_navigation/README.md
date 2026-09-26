# Autonomous Navigation with Mission Control GUI (`ms04_autonomous_navigation`)

ROS 2 (Jazzy) package implementing **autonomous navigation for a TurtleBot3 Waffle** in two custom Gazebo worlds (**Office** and **Warehouse**), driven by a **PyQt5 Mission Control GUI**. The GUI launches the simulation, loads the map and Nav2 stack, sets the initial pose, and dispatches single-goal or waypoint missions — **everything from buttons**. No ROS 2 CLI knowledge is required to use or test it.

> This package is part of the [Ros2-Course-Projects](../README.md) repo — see the top-level README for workspace setup and cloning instructions.

---

## 1. Features

- **Button-driven Mission Control GUI** — one window runs the whole pipeline: simulation, map + Nav2 bringup, initial pose, and goal dispatch.
- **Custom Gazebo worlds** — purpose-built Office (multi-room) and Warehouse (high-bay racks) layouts.
- **Environment-specific Nav2 tuning & AMCL** — Office profile for tight corridors, Warehouse profile for fast straight runs.
- **Unified coordinator node** — single Go-To-Pose *and* multi-waypoint `FollowWaypoints` routes from one command interface.
- **Full execution control set** — Start, **Pause, Resume, Cancel, Replace**, all with event logging into the GUI log view.
- **Live mission status** — pose, distance remaining, ETA and waypoint index update in real time while a mission runs.
- **Environment switching** — changing env and pressing Launch cleanly stops the other environment's whole stack first (no Gazebo pile-ups).
- **Autonomous SLAM exploration** — frontier-based mapping with automatic map saving (maps already committed).

---

## 2. Prerequisites

- **OS:** Ubuntu 24.04 LTS
- **ROS 2:** Jazzy
- **Simulation/navigation deps + PyQt5:**

```bash
sudo apt update
sudo apt install -y \
    python3-pyqt5 \
    ros-$ROS_DISTRO-turtlebot3-gazebo \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-nav2-bringup
```

- **Environment variable** (must be set in every terminal and added to `~/.bashrc`):

```bash
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
```

> The saved maps (`maps/office_map.yaml`, `maps/warehouse_map.yaml`) ship inside
> the package, so no SLAM run is required before testing navigation.

---

## 3. Workspace Setup & Build

```bash
# 1. Create workspace and clone
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/alihash2/Ros2-Course-Projects.git

# 2. Build the package
cd ~/ros2_ws
colcon build --packages-select ms04_autonomous_navigation
source install/setup.bash
```

> **Every new terminal must source the workspace:**
> ```bash
> source ~/ros2_ws/install/setup.bash
> ```

---

## 4. Quick Start — the whole pipeline in one GUI

**Copy-paste these commands in order:**

```bash
cd ~/ros2_ws
colcon build --packages-select ms04_autonomous_navigation
source install/setup.bash

export TURTLEBOT3_MODEL=waffle
ros2 run ms04_autonomous_navigation mission_control_gui
```

Inside the GUI, the button flow is the **only** flow you need:

| Step | Button | What happens |
|------|--------|--------------|
| 1 | **Launch Simulation** | Gazebo opens with the selected world (Office by default) and RViz |
| 2 | **Load Map + Activate Nav2** | Nav2 servers + AMCL localization + map + coordinator start |
| 3 | **Set Initial Pose** | Robot is placed at the environment's configured spawn on `/initialpose` |
| 4 | Pick a goal → **Start Mission** | Robot navigates; every event appears in the live log |

No `ros2 topic` commands are ever required during testing — the buttons
publish them for you.

---

## 5. Step-by-Step Guided Session — Office

### Terminal 1 — Mission Control GUI

```bash
cd ~/ros2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 run ms04_autonomous_navigation mission_control_gui
```

### Steps

1. **Select `Office`** in the Environment dropdown (default).
2. Press **Launch Simulation**. Wait for the log to show `Launched simulation: Office (office_simulation.launch.py)` and for Gazebo + RViz to open with the Waffle in the office world.
3. Press **Load Map + Activate Nav2**. Wait for `Activated Nav2 + map load: Office (office_navigation.launch.py) with launch_sim:=false`.
4. Press **Set Initial Pose**. The robot appears localized in RViz (green scan lines on the map).
5. Set Mode = **Waypoints**. From the **POI** dropdown add these three stops with **Add as Waypoint**:
   - `NW Conference`
   - `SE Breakroom`
   - `Corridor Center`
6. Press **Start Mission**. Expect:
   - the **Progress** lamp lights;
   - the log prints compact progress lines as the robot moves (waypoint index, remaining distance, ETA);
   - on each arrival the log reports the reached waypoint and the **Progress** lamp walks to the next stage.
7. When the last stop is reached:
   - the log shows a **green "mission completed"** line;
   - Mission Status shows `0.00 m` remaining / `~0s` ETA;
   - the **Reached** lamp lights.

---

## 6. Step-by-Step Guided Session — Warehouse

Repeat section 5 with Env = **Warehouse**. Differences:

- **Launch Simulation** → `Launched simulation: Warehouse (warehouse_simulation.launch.py)`, world has high-bay racks, spawn at `(-6.0, 0.0)`.
- Suggested waypoint route: `Loading Dock (West)` → `Rack B (South)` → `East Storage`.
- The **Warehouse** Nav2 profile is tuned faster (`max_vel_x 0.9`) — expect longer, quicker straights and wider turns around pallet racks.

---

## 7. Pause / Resume / Cancel / Replace (step by step)

Start any mission, then exercise the controls:

| Action | Button | Expected log + state |
|--------|--------|----------------------|
| While the robot drives, halt it | **Pause** | `State: PAUSED`; robot stops; lamp turns **Paused** |
| Continue again | **Resume** | `State: NAVIGATING`; robot re-dispatches from the pause point |
| Stop the mission outright | **Cancel Goal** | `EVENT_GOAL_CANCELED`, `State: CANCELED`; lamps go dark; distance/ETA reset to `0.00 m / ~0s` |
| Change target mid-route | pick a new goal, then **Replace Goal** | `EVENT_GOAL_REPLACED`; robot abandons the old route and heads to the new goal |

> **Single-goal missions never light waypoint lamps** — by design the lamps
> report waypoint-group status only.

---

## 8. Switching Environments Mid-Session

With the **Office** stack fully running:

1. Set Env = **Warehouse** in the dropdown (only the POI list swaps immediately).
2. Press **Launch Simulation**.
3. Expect a log warning `Stopping Office stack ... before switching environment`, then the Office simulation, Nav2, RViz **and coordinator processes are stopped cleanly** before the Warehouse simulation starts — no second Gazebo, no port conflict.
4. Press **Load Map + Activate Nav2** then **Set Initial Pose** to bring up Warehouse navigation.

---

## 9. Mission Control GUI Reference

| Panel | Controls |
|-------|----------|
| Environment & Navigation | Env selector (Office / Warehouse), **Launch Simulation**, **Load Map + Activate Nav2**, **Set Initial Pose** |
| Goal & Waypoint Dispatcher | Mode (Single Goal / Waypoints), X / Y / Theta inputs, POI dropdown per environment, Add/Remove/Clear waypoint list |
| Execution Control | Start, Pause, Resume, Cancel Goal, Replace Goal (color-coded buttons) |
| Mission Status | Live state, mission id, current pose, distance remaining, ETA, current waypoint, target |
| Waypoint Lamps | Loaded → Progress → Paused → Reached (one lit at a time for waypoint missions; all dark for single goals) |
| Real-Time Navigation Log | Color-coded `NavigationEvent` + GUI command feed; auto-scroll; green mission-completed lines |

The GUI log also surfaces failures automatically (`start rejected: mission
already active`, `cancel ignored: no active mission`, `mission aborted: Failed
to create plan…`), so problems are visible in the window instead of only in the
terminal. The window backdrop tints with mission state (idle / navigating /
paused / completed / failed) as a visual cue.

---

## 10. World & Coordinate Reference

| | Office | Warehouse |
|---|---|---|
| World size | 16 × 14 m | 20 × 18 m |
| Robot spawn (world) | `(0.0, 0.0, 0.0)` | `(-6.0, 0.0, 0.0)` |
| GUI "Set Initial Pose" | `(0.0, 0.0, 0.0)` | `(0.0, 0.0, 0.0)` |
| Default map | `maps/office_map.yaml` | `maps/warehouse_map.yaml` |

**Office POIs** (from the GUI dropdown):

| POI | (x, y, θ) |
|-----|-----------|
| Spawn | (0.0, 0.0, 0.0) |
| Corridor Center | (0.0, -4.0, 0.0) |
| NW Conference | (-2.5, 5.5, π) |
| SW Reception | (-3.2, -2.2, π) |
| NE Cubicles | (3.0, 3.2, 0.0) |
| SE Breakroom | (3.0, -1.8, 0.0) |

**Warehouse POIs**:

| POI | (x, y, θ) |
|-----|-----------|
| Spawn | (0.0, 0.0, 0.0) |
| Aisle Center | (6.5, 0.0, 0.0) |
| Loading Dock (West) | (-3.0, 1.5, π) |
| East Storage | (9.5, 0.0, 0.0) |
| Rack A (North) | (5.0, 6.0, 0.0) |
| Rack B (South) | (5.0, -6.0, 0.0) |

You can also type raw X / Y / Theta — any typed pose within `POI_MATCH_TOLERANCE` (0.5 m) of a predefined POI is auto-labelled with the POI name in the waypoint list.

---

## 11. Launch Files (alternative to the GUI)

The GUI runs these launches in the background; you can also run them directly.

| Launch | What it starts |
|--------|----------------|
| `office_simulation.launch.py` | Gazebo + bridge + RViz, Office world only |
| `office_navigation.launch.py` | Sim (unless `launch_sim:=false`) + Nav2 servers + AMCL + coordinator + RViz |
| `warehouse_simulation.launch.py` | Gazebo + bridge + RViz, Warehouse world only |
| `warehouse_navigation.launch.py` | Sim (unless `launch_sim:=false`) + Nav2 servers + AMCL + coordinator + RViz |
| `nav2_servers.launch.py` | Shared Nav2 stack (planner/controller/smoother/behavior/waypoint navigation) |
| `office_mapping.launch.py` / `warehouse_mapping.launch.py` | Autonomous SLAM exploration + auto map save |

```bash
# Full navigation standalone (equivalent of the two GUI buttons):
ros2 launch ms04_autonomous_navigation office_navigation.launch.py

# Attach Nav2 to an already-running sim (no second Gazebo):
ros2 launch ms04_autonomous_navigation office_navigation.launch.py launch_sim:=false

# Headless run (no RViz/Gazebo windows):
ros2 launch ms04_autonomous_navigation warehouse_navigation.launch.py gui:=false
```

Arguments: `gui` (default `true`), `launch_sim` (default `true`), `use_sim_time`
(`true`), `autostart` (`true`), `params_file` (env profile), `map_yaml`.

---

## 12. Manual Command-Line Publishing (developer option)

For scripting / CI only — the tester-facing path is fully button-driven.

```bash
source ~/ros2_ws/install/setup.bash

# Watch the live event + status streams
ros2 topic echo /navigation/events ms04_autonomous_navigation/msg/NavigationEvent
ros2 topic echo /navigation/status ms04_autonomous_navigation/msg/NavigationStatus

# Start a go-to-pose mission (same message the GUI's Start button publishes)
ros2 topic pub --once /navigation/mission ms04_autonomous_navigation/msg/NavigationMission \
  "{header: {stamp: {sec: 0}, frame_id: 'map'}, mission_id: 'demo', mode: 0, \
    command: 0, target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 2.0}, orientation: {w: 1.0}}}}"

# Pause / resume / cancel (command: 0 start, 1 cancel, 2 pause, 3 resume, 4 replace)
ros2 topic pub --once /navigation/mission ms04_autonomous_navigation/msg/NavigationMission \
  "{header: {stamp: {sec: 0}, frame_id: 'map'}, command: 2}"
```

---

## 13. Custom Interfaces

| Interface | Purpose |
|-----------|---------|
| `NavigationMission.msg` | Unified command: `MODE_GO_TO_POSE` / `MODE_WAYPOINTS` × `COMMAND_START` / `CANCEL` / `PAUSE` / `RESUME` / `REPLACE` |
| `NavigationEvent.msg` | Lifecycle + telemetry events (`GOAL_SUBMITTED`, `ACCEPTED`, `REJECTED`, `FEEDBACK`, `COMPLETED`, `CANCELED`, `ABORTED`, `PAUSED`, `RESUMED`, `REPLACED`) |
| `NavigationStatus.msg` | Live snapshot: state, pose, remaining distance, ETA, waypoint indices |
| `ExecuteMission.action` | Action server with real-time feedback (pose, distance, ETA, recovery count) |

---

## 14. Navigation Coordinator Node

Receives missions, dispatches to the matching Nav2 navigator (single pose →
`NavigateToPose`, waypoints → `FollowWaypoints`), implements the full control
set, and logs every transition on `/navigation/events`.

| Direction | Topic / Action | Interface |
|-----------|----------------|-----------|
| in | `/navigation/mission` (user commands) | `NavigationMission` |
| out | `/navigation/events` (lifecycle + telemetry) | `NavigationEvent` |
| out | `/navigation/status` (state snapshot) | `NavigationStatus` |
| in | `/execute_mission` (action server) | `ExecuteMission` |
| out | `/navigate_to_pose` | nav2 `NavigateToPose` |
| out | `/follow_waypoints` | nav2 `FollowWaypoints` |

Control semantics: **Start** (rejected if a mission is already active),
**Cancel** (ends `CANCELED`, works while paused too), **Pause** (preserves
remaining mission, holds `PAUSED`), **Resume** (re-dispatches from pause),
**Replace** (preempts and dispatches the new mission).

---

## 15. Autonomous SLAM Exploration (mapping workflow)

Frontier-based exploration with Nav2-owned motion, deep-room aiming, permanent
goal blacklist and watchdogs; saves the map automatically on completion.

```bash
# Developer/mapping workflow (the GUI does not include mapping)
ros2 launch ms04_autonomous_navigation office_mapping.launch.py      # Office
ros2 launch ms04_autonomous_navigation warehouse_mapping.launch.py   # Warehouse
```

Writes `maps/office_map.{pgm,yaml}` / `maps/warehouse_map.{pgm,yaml}`.
Verified: office 92.9 % and warehouse 92.8 % coverage; the remaining ~2 % are
unreachable slivers, not timeout artifacts.

---

## 16. Troubleshooting

### GUI launches nothing / buttons do nothing

1. **Sourcing** — `source ~/ros2_ws/install/setup.bash` must run in the terminal before starting the GUI (`ros2 run` fails with "launch file not found" otherwise).
2. **`TURTLEBOT3_MODEL`** — must be `waffle` or the robot model fails to spawn.
3. **Timing** — Gazebo takes a few seconds; wait for the log line matching the launch before pressing the next button. Launching again while the stack is still starting logs a duplicate warning.

### "No Office simulation detected" when Activating Nav2

The **Load Map + Activate Nav2** button attaches to an already-running sim —
press **Launch Simulation** first.

### Two Gazebo instances / port conflicts

This is the env-switch slip-up: launching a different environment while another
one's stack is running. The GUI now stops the other env's whole stack first
(section 8). If it ever happens anyway, close everything with:

```bash
pkill -f 'gazebo|ros2 launch ms04'
```

### DDS / discovery problems (multiple machines, VPN)

Set the same `ROS_DOMAIN_ID` in every terminal (or `export ROS_LOCALHOST_ONLY=1`)
and check with `ros2 node list`.

---

## 17. Simulation Environments

| World | Layout | Launch |
|-------|--------|--------|
| **Office** | reception/lounge, conference room, open cubicles, executive office, central corridor | `office_simulation.launch.py` |
| **Warehouse** | 3 dual-sided high-bay racks, staging areas, pallet stacks, crates, loading zones | `warehouse_simulation.launch.py` |