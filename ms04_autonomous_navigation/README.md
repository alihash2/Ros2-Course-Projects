# ms04_autonomous_navigation

Autonomous navigation (Nav2) package with a **PyQt5 Mission Control GUI** that
drives the entire pipeline — simulation, map + Nav2 bringup, initial pose, and
goal/waypoint missions — **entirely from its buttons**. Runs in two custom
Gazebo worlds: **Office** and **Warehouse**.

The GUI is the primary interface: do not hand-publish missions when testing.
The buttons publish the ROS 2 commands for you; the log view shows every event
live.

---

## Requirements

- Ubuntu 24.04 with **ROS 2 Jazzy**
- Python 3 + **PyQt5** (`python3-pyqt5`)
- TurtleBot3 sim bits for the custom worlds (`turtlebot3_gazebo`,
  `turtlebot3_navigation` for the Waffle model), Nav2 stack, SLAM toolbox
- `TURTLEBOT3_MODEL=waffle` exported in every terminal

Install the extra system pieces (most are colcon-resolvable too):

```bash
sudo apt install -y \
    python3-pyqt5 \
    ros-$ROS_DISTRO-turtlebot3-gazebo \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-nav2-bringup
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
```

---

## Quick Start — Test Drive (2 commands)

Build once, then drive everything from the GUI:

```bash
cd ~/ros2_ws
colcon build --packages-select ms04_autonomous_navigation
source install/setup.bash

ros2 run ms04_autonomous_navigation mission_control_gui     # the whole app
```

> Source `~/ros2_ws/install/setup.bash` in every terminal before running.

In the GUI, the **button flow is the only flow you need**:

1. **Launch Simulation** — starts Gazebo with the selected world (Office/Warehouse) + RViz.
2. **Load Map + Activate Nav2** — starts AMCL localization + Navigation2 against the saved map.
3. **Set Initial Pose** — drops the robot at the spawn point so Nav2 has a pose.
4. Pick a **goal** and press **Start Mission** — the log shows each event live.

No `ros2 topic` commands are required during testing.

---

## Testing Guide

Run these scenarios to exercise the whole system. The last column is the
expected, correct behaviour — anything that differs is a defect to report.

### T1 — Office waypoint route (main path)

| Step | What you do | What should happen |
|------|-------------|--------------------|
| 1 | Env = **Office**, press **Launch Simulation** | Gazebo + RViz open, log shows the sim command, robot spawns in the office world |
| 2 | Press **Load Map + Activate Nav2** | Log shows `Activated Nav2 + map load: Office` |
| 3 | Press **Set Initial Pose** | Robot appears localized in RViz; no "no map" warnings |
| 4 | Mode = **Waypoints**; from the POI list add 3 stops (e.g. NW Conference, SE Breakroom, Corridor Center) | 3 rows appear in the waypoint list |
| 5 | Press **Start Mission** | Robot navigates stop-to-stop; **Progress** lamp lights |
| 6 | — | On each arrival the log prints the waypoint; the **Progress** lamp walks across as stops are reached |
| 7 | Last waypoint reached | Log shows a **green "mission completed"** line; Mission Status shows `0.00 m · ~0s` remaining |

### T2 — Single goal + Pause/Resume/Cancel

| Step | What you do | What should happen |
|------|-------------|--------------------|
| 1 | Mode = **Single Goal**, target e.g. `0, -4, 0`; **Start Mission** | Robot drives there; state `NAVIGATING` |
| 2 | **Pause** mid-route | Robot stops; state `PAUSED`; lamp turns amber (**Paused**) |
| 3 | **Resume** | Robot continues from where it paused; state back to `NAVIGATING` |
| 4 | **Cancel Goal** | Mission ends `CANCELED`; all waypoint lamps dark; distance/ETA reset to `0.00 m / ~0s` |

> Note: with a **single goal** no waypoint lamps light at all — that is by design
> (the lamps track waypoint-group status only).

### T3 — Replace mid-route

| Step | What you do | What should happen |
|------|-------------|--------------------|
| 1 | Start a waypoint route | Navigator active |
| 2 | While moving, build a different goal and press **Replace Goal** | Log shows `EVENT_GOAL_REPLACED`; the robot abandons the old route and heads to the new target |

### T4 — Warehouse waypoint route

Repeat T1 with **Env = Warehouse** (POIs: Loading Dock (West), Rack B (South),
East Storage). Expect the same flow with the faster warehouse Nav2 tuning
(higher speed, wider turns).

### T5 — Switch environments mid-session (stack swap)

| Step | What you do | What should happen |
|------|-------------|--------------------|
| 1 | With the **Office** stack fully up, set Env = **Warehouse** | Env selector swaps POIs |
| 2 | Press **Launch Simulation** | The Office sim/Nav2 processes are **cleanly stopped first** (log warning) and the Warehouse sim starts — no second Gazebo pile-up |
| 3 | **Load Map + Activate Nav2**, **Set Initial Pose** | Warehouse navigation comes up normally |

### T6 — Failure visibility

| Step | What you do | What should happen |
|------|-------------|--------------------|
| 1 | Try to **Start** while a mission is already active | Log shows a diagnostic line (e.g. `start rejected: mission already active`) |
| 2 | Try **Pause** while idle | Log shows `pause ignored: no active goal` |

### Reporting a finding

The GUI log view is the test artifact — paste the relevant log lines into the
issue. Include: environment (Office/Warehouse), the buttons you pressed, the
expected vs. actual outcome. File it against the repo's issue tracker as a
normal bug/feature ticket.

---

## Mission Control GUI

| Panel | Controls |
|-------|----------|
| Environment & Navigation | Env selector (Office / Warehouse), **Launch Simulation**, **Load Map + Activate Nav2**, **Set Initial Pose** |
| Goal & Waypoint Dispatcher | Mode (Single Goal / Waypoints), X / Y / Theta inputs, POI dropdown per env, Add/Remove/Clear waypoint list |
| Execution Control | Start, Pause, Resume, Cancel Goal, Replace Goal (color-coded) |
| Mission Status | Live state, mission id, current pose, distance remaining, ETA, current waypoint, target |
| Waypoint Lamps | One lamp lights per waypoint status — **Loaded → Progress → Paused → Reached** |
| Real-Time Navigation Log | Color-coded `NavigationEvent` + GUI command feed, auto-scroll |

- **Lamps**: only meaningful for waypoint missions — each lamp shows the active
  stage (a steady single lamp, or the walking "Reached" lamp as stops complete).
- **Env switching**: pressing Launch / Load+Activate when another environment's
  stack is running **stops that whole stack first** and then starts the newly
  selected one.
- **Log**: events render as human-readable milestone lines (waypoint arrivals,
  recovery triggers, remaining distance / ETA, aborts with the Nav2 error).
  Diagnostic failures are surfaced too ("start rejected…", "cancel ignored…",
  "mission aborted: Failed to create plan…"), so a problem is visible in the
  GUI instead of only in the terminal.
- Background colour shifts with mission state (idle / navigating / paused /
  completed / failed) as a visual status cue.

Run:

```bash
ros2 run ms04_autonomous_navigation mission_control_gui
```

---

## Features

- Custom Gazebo **Office** and **Warehouse** simulation worlds
- Environment-specific **AMCL localization** and **Nav2 tuning**
- Single **Go-To-Pose** and **Multi-Waypoint** missions through one unified
  coordinator node
- **Pause / Resume / Cancel / Replace** control set with full event logging
- PyQt5 **Mission Control GUI** covering the whole pipeline
- Autonomous SLAM **exploration with automatic map saving** (mapping workflow)

---

## Custom Interfaces

| Interface | Purpose |
|-----------|---------|
| `NavigationMission.msg` | Unified mission command: `MODE_GO_TO_POSE` / `MODE_WAYPOINTS` × `COMMAND_START` / `CANCEL` / `PAUSE` / `RESUME` / `REPLACE` |
| `NavigationEvent.msg` | Telemetry + lifecycle events (`GOAL_SUBMITTED`, `ACCEPTED`, `REJECTED`, `FEEDBACK`, `COMPLETED`, `CANCELED`, `ABORTED`, `PAUSED`, `RESUMED`, `REPLACED`) |
| `NavigationStatus.msg` | Live snapshot: state, pose, remaining distance, ETA, waypoint indices |
| `ExecuteMission.action` | Action server with real-time feedback (pose, distance, ETA, recovery count) |

### Developer reference — CLI
For scripting/CI only; the tester-facing flow is 100 % button-driven.

```bash
ros2 topic echo /navigation/events ms04_autonomous_navigation/msg/NavigationEvent
ros2 topic echo /navigation/status ms04_autonomous_navigation/msg/NavigationStatus

# start a go-to-pose mission (same message the GUI's Start button publishes)
ros2 topic pub --once /navigation/mission ms04_autonomous_navigation/msg/NavigationMission \
  "{header: {stamp: {sec: 0}, frame_id: 'map'}, mission_id: 'demo', mode: 0, \
    command: 0, target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 2.0}, orientation: {w: 1.0}}}}"

# pause / resume / cancel (command: 0 start, 1 cancel, 2 pause, 3 resume, 4 replace)
ros2 topic pub --once /navigation/mission ms04_autonomous_navigation/msg/NavigationMission \
  "{header: {stamp: {sec: 0}, frame_id: 'map'}, command: 2}"
```

---

## Navigation Coordinator Node

Receives missions, dispatches to the right Nav2 navigator (single pose →
`NavigateToPose`, waypoints → `FollowWaypoints`), implements the full control
set, and logs every transition.

| Direction | Topic / Action | Interface |
|-----------|----------------|-----------|
| in | `/navigation/mission` (user commands) | `NavigationMission` |
| out | `/navigation/events` (lifecycle + telemetry) | `NavigationEvent` |
| out | `/navigation/status` (state snapshot) | `NavigationStatus` |
| in | `/execute_mission` (action server) | `ExecuteMission` |
| out | `/navigate_to_pose` | nav2 `NavigateToPose` |
| out | `/follow_waypoints` | nav2 `FollowWaypoints` |

Control semantics: **Start** (rejected if something is already active),
**Cancel** (ends `CANCELED`, works paused too), **Pause** (keeps remaining
mission, holds `PAUSED`), **Resume** (re-dispatches from the pause point),
**Replace** (preempts and dispatches the new mission).

---

## Environment Navigation (AMCL)

Each env has its own tuned profile and brings up Gazebo + Nav2 servers +
`map_server` + `amcl` + the coordinator + RViz:

| Environment | Params profile | Launch |
|-------------|----------------|--------|
| Office (corridors, doorways, tight turns) | `params/office_nav2_params.yaml` | `office_navigation.launch.py` |
| Warehouse (long straights, pallet clearance, faster) | `params/warehouse_nav2_params.yaml` | `warehouse_navigation.launch.py` |

```bash
# equivalent of the GUI's Launch + Load/Activate buttons (or just use the GUI)
ros2 launch ms04_autonomous_navigation office_navigation.launch.py
ros2 launch ms04_autonomous_navigation warehouse_navigation.launch.py
```

Tuning philosophy: **Office** = precise (`max_vel_x 0.5`, tight tolerances);
**Warehouse** = fast (`max_vel_x 0.9`, longer `sim_time`, wider inflation).
Both set AMCL `set_initial_pose: true` at spawn, so no manual 2D pose estimate
is required.

---

## Autonomous SLAM Exploration (mapping workflow)

Frontier-based exploration with Nav2-owned motion, deep-room aiming, permanent
goal blacklist, completion cues, and watchdogs. Saves maps automatically.

```bash
# mapping runs (developer workflow)
ros2 launch ms04_autonomous_navigation office_mapping.launch.py     # Office
ros2 launch ms04_autonomous_navigation warehouse_mapping.launch.py  # Warehouse
```

Outputs: `maps/office_map.{pgm,yaml}`, `maps/warehouse_map.{pgm,yaml}`.
Verified: office 92.9 %, warehouse 92.8 % coverage; the remaining ~2 % are
unreachable slivers, not timeouts.

---

## Simulation Environments

| World | Layout | Launch |
|-------|--------|--------|
| **Office** | multi-room: reception/lounge, conference room, open cubicles, executive office, central corridor | `office_simulation.launch.py` |
| **Warehouse** | industrial: 3 dual-sided high-bay racks, staging areas, pallet stacks, crates, loading zones | `warehouse_simulation.launch.py` |