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
ros2 run ms04_autonomous_navigation mission_control_gui.py
```

Inside the GUI, the button flow is the **only** flow you need:

| Step | Button | What happens |
|------|--------|--------------|
| 1 | **Launch Map** | Gazebo opens with the selected world (Office by default) and RViz |
| 2 | **Load Map + Nav** | Nav2 servers + AMCL localization + map + coordinator start |
| 3 | **Set Initial Pose** | Robot's live pose from Gazebo is published on `/initialpose` → AMCL re-anchors, robot appears localized in RViz |
| 4 | Pick a goal → **Start Mission** | Robot navigates; every event appears in the live log |

No `ros2 topic` commands are ever required during testing — the buttons
publish them for you.

### Workflow guardrails

Each stage unlocks only when the previous one is verified running (the GUI
watches the actual `ros2` processes every 2 s):

- **Greyed-out controls** are still clickable and, instead of silently doing
  nothing, show a **red error bar** naming the exact button you need to press.
- The required button briefly **pulses with a soft red/white glow** until the
  stage is completed.
- **Stage 2 counts as complete only when the Nav2 stack is fully online**:
  the environment's params file plus the shared `lifecycle_manager` and
  `navigation_coordinator_node` must all be running. If the nav launch dies
  before going fully online, the red bar reports the missing components and
  tells you to press **Load Map + Nav** again.
- **All five executor commands are gated the same way**: **Start**, **Pause**,
  **Resume**, **Cancel Goal** and **Replace Goal** first require the full
  workflow (Launch Map → Load Map + Nav fully online → Set Initial Pose) and an
  online executor; otherwise they show the red error bar and publish nothing,
  instead of sending a command the coordinator would ignore. Pause/Resume/Cancel
  additionally verify the mission state (`NAVIGATING`/`PAUSED`) and refuse an
  irrelevant command the same way.
- **Launch Map and Load Map + Nav clean up leftovers before starting.** Each
  press scans for any leftover ms04 stack processes (stale sims, orphaned
  bridges/coordinators, old rviz, nav2, lifecycle managers) and stops them first,
  then launches a fresh stack — an accidental duplicate click or a crashed stack
  restarts cleanly instead of piling orphans on top of each other. Load Map + Nav
  stops only the navigation tier and deliberately keeps a healthy running sim
  alive. The two launch buttons are also **debounced**: a second press inside
  6 s of the first is ignored, so a rapid double-click cannot spawn a colliding
  second stack while the first is still starting. If **Launch Map** is pressed
  but the world process never appears (the launch parent died mid-startup), the
  red bar reports the failed simulation and asks you to press **Launch Map**
  again — just like the Nav2 startup check. A second GUI instance is warned
  about on startup, since two GUIs would fight over the same stack.
- Pressing **Set Initial Pose** before it is allowed (Map not up, or Map+Nav
  not fully online) is blocked with the red bar. When it is allowed, the button
  publishes the robot's **true live pose** (queried straight from Gazebo every
  press) instead of a fixed constant, so a re-press is always safe and
  re-anchors AMCL exactly where the robot is — no drift, no shift. Re-seeding
  is **blocked while a mission is running or paused** (red bar, button greyed)
  because the map→odom transform must not jump under a live mission.
- **Starting a mission still requires the pose to have been set once** — goal
  dispatch stays locked until the robot is localized (`_pose_set`), even though
  AMCL auto-seeds on startup; the pressed pose is just re-anchored live.
- **Waypoint mode** refuses *consecutive duplicate* waypoints (same point
  within 0.5 m and same heading), and the coordinate boxes stay editable in
  waypoint mode so you can compose each waypoint before adding it.
- Switching the **Environment** clears all waypoints and resets the initial pose
  so the previous world's plan can never leak into the new one. It is **blocked
  while a mission is `NAVIGATING` or `PAUSED`** (red bar naming **Cancel Goal**),
  because the map frame differs per environment and a live mission cannot jump
  across it; finish or cancel the mission first.
- In **Single Goal** mode the coordinate boxes stay **greyed out while a preset
  POI is selected, but remain editable**: clicking the box or nudging the up/down
  arrows lets you type a new value, and the moment any coordinate changes the POI
  intent instantly switches to **Custom** (keeping your value, never snapping
  back). The Mission Status waypoint line reads `waypoint: 1/1` (a single goal
  has no waypoint list).
- **Add / Remove / Clear while in Single Goal mode show a popup instead of
  acting** on the waypoint list: the red bar explains that you are in **Single
  Goal** mode, tells you to select a POI or enter custom X/Y/Theta coordinates
  and press **Start Mission**, and pulses **Start Mission** — those buttons only
  build a waypoint route, so in Single Goal mode they are meaningless (and they
  are no-ops rather than silently editing a route you are not in).
- **POI and typed coordinates are one unified input**: picking a preset POI
  snaps the coordinate boxes to it. Editing any box never lets a preset snap
  the pose back — the intent immediately becomes **Custom** and your typed
  values are kept. The auto-select check runs **only once you have finished
  editing** (focus leaves a field): if the pose then matches a predefined POI
  **exactly** — X, Y *and* heading, to the 2-decimal display precision — that
  POI is auto-selected and the boxes snap to its canonical values; any near
  miss (e.g. a heading of 3.19 vs. a POI's π) stays Custom with your numbers
  untouched.
- **The coordinate fields accept any value** (no map bounds, e.g. the spin boxes
  range up to ±10000 in X/Y and heading), and the Custom entry reads
  **Custom (type coords above)** since the boxes sit above the POI list. A
  waypoint entered from Custom is always labelled `custom coords (x, y, θ)` in
  the tracker — the name of a predefined POI appears only when that POI is the
  one actually selected (never just "the last POI you picked").
- **Start Mission is blocked while a mission is active** — while
  `NAVIGATING`/`PAUSED`/… the button is greyed out; pressing it shows a red bar
  (and pulses **Replace Goal**) instead of publishing a command the coordinator
  would reject as "mission already active".
- **Replace Goal requires a paused mission with a changed plan**: it needs
  `PAUSED` **and** a goal or waypoint list different from the running mission
  (order matters). Pressing it while driving → "PAUSED first"; while paused with
  an unchanged plan → red bar telling you to change the goal/waypoints. The
  current mission is remembered automatically so an identical guess can't be
  re-sent.
- **The waypoint tracker is editable even while a waypoint mission drives**:
  Add / Remove / Clear stay **open** during `NAVIGATING` (and are fully
  unlocked while paused), so you can draft a replacement route while the robot
  keeps executing the *old* plan. The Loaded lamp is what flags the draft: it
  goes dark the moment the list diverges from the running plan and relights
  only when the list matches it again (see the lamps bullet below). Only
  **Replace Goal** (which still requires `PAUSED` first) actually swaps the
  new route in.
- **The status lamps each mean exactly one thing**: **Loaded = route-sync** —
  lit only while a waypoint mission is running/paused *and* the waypoint list
  in the tracker exactly matches the plan the robot is executing; **Progress**
  = driving it; **Paused** = stopped mid-plan; **Reached** = mission
  completed. Hovering over the **Loaded** dot or label pops a tooltip that (in
  the divergent case) tells you the new waypoints are **unapplied** and to
  **press Pause, then Replace Goal** to begin executing the new list.
- **Waypoint editing prompts for the initial pose just like dispatching** —
  Add / Remove / Clear (and **double-clicking the waypoint list** to add the
  current selection) run through the same staged readiness gate, so trying to
  plan a route before the pose is set raises the red bar naming **Set Initial
  Pose** instead of silently letting you build a plan you cannot run.
- **Clicking any goal-dispatcher input also walks you through startup**: the
  X / Y / Theta boxes and the POI selector (and the Add / Remove / Clear
  buttons) raise the same red bar + glowing button as the executor controls —
  **Launch Map → Load Map + Nav → Set Initial Pose** — so the first thing you
  touch on the goal form tells you exactly which button to press next, in
  order, to reach a dispatchable state.
- **The pending selection is signalled**: whenever the selected POI/coordinates
  differ from the latest waypoint in the tracker (i.e. Add would accept them),
  the **Add as Waypoint** button glows and the POI + X/Y/Theta text turns red;
  both indicators go neutral once the selection matches the last entry.
- **In Waypoint mode, picking a preset POI appends it immediately** — one click
  is one waypoint, so a pure-POI route needs zero button presses (re-picking a
  spot that already ends the list is a silent no-op for browsing). The moment
  you hand-edit any X / Y / θ field, that newest row turns into a **draft**
  (amber, *italic*, marked `▸ … (draft)`) that follows the boxes live — an
  edit-in-place heading tweak fixes the row instead of spawning a near-duplicate.
  The draft freezes into a real waypoint on any commit gesture: **Add as
  Waypoint**, **Enter** (or finishing the edit), **clicking another POI**, or
  **Start Mission**; the consecutive-duplicate rule still guards the row it
  resumes. Editing again *after* a commit starts composing the **next** entry
  (Add glows), not silently amending the row you already locked in.
- **Replacing / switching env resets the remembered plan**: after a successful
  Replace (or mission end, or env switch) the remembered running plan is cleared,
  so the next Replace press is gated on a genuinely new plan.
- **A waypoint mission that never reached some waypoints is reported as
  ABORTED, not COMPLETED**: Nav2's follow_waypoints keeps marching on even when
  a waypoint cannot be reached (it records it as missed). The coordinator now
  surfaces "N waypoint(s) never reached (robot stuck/blocked)" as an aborted
  partial mission instead of claiming every waypoint was achieved.
- **Plain-text run logs are retained for diagnosis**: every fresh launch of the
  GUI appends its full log to `~/.ros/ms04_mission_control/runs/run_*.log`, and
  only the newest **5 runs** are kept (a fresh launch prunes the oldest to make
  room). Set `MS04_MISSION_CONTROL_LOG_DIR` to redirect the directory. These
  files survive on disk so any user or agent can read what happened across the
  last sessions.

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
2. Press **Launch Map**. Wait for the log to show `Launched simulation: Office (office_simulation.launch.py)` and for Gazebo + RViz to open with the Waffle in the office world.
3. Press **Load Map + Nav** and wait until it is fully online (the log shows it as active; waypoint/goal stages unlock only then).
4. Press **Set Initial Pose**. The robot appears localized in RViz (green scan lines on the map).
5. Set Mode = **Waypoints**. Add these three stops by picking each POI from the
   dropdown — **one click appends one waypoint** (no button press needed), or
   use **Add as Waypoint** for typed coordinates:
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

- **Launch Map** → `Launched simulation: Warehouse (warehouse_simulation.launch.py)`, world has high-bay racks, spawn at `(-6.0, 0.0)`.
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
| Change target mid-route | press **Pause** first, then pick a new goal / waypoints, then **Replace Goal** | `EVENT_GOAL_REPLACED`; robot abandons the old route and heads to the new goal. **Replace** only works while `PAUSED` and with a plan different from the running mission — otherwise a red bar explains what is missing |

> **Single-goal missions never light waypoint lamps** — by design the lamps
> report waypoint-group status only.

---

## 8. Switching Environments Mid-Session

With the **Office** stack fully running:

1. Set Env = **Warehouse** in the dropdown (only the POI list swaps immediately).
2. Press **Launch Map**.
3. Expect a log warning `Stopping Office stack ... before switching environment`, then the Office simulation, Nav2, RViz **and coordinator processes are stopped cleanly** before the Warehouse simulation starts — no second Gazebo, no port conflict.
4. Press **Load Map + Nav** (wait for it to go fully online) then **Set Initial Pose** to bring up Warehouse navigation.

---

## 9. Mission Control GUI Reference

| Panel | Controls |
|-------|----------|
| Environment & Navigation | Env selector (Office / Warehouse), **Launch Map**, **Load Map + Nav**, **Set Initial Pose** |
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
| "Set Initial Pose" | publishes the robot's **live Gazebo pose** (world→map via the spawn offset) — never a fixed spawn | |
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

You can also type raw X / Y / Theta — any typed pose within `POI_MATCH_TOLERANCE` (0.5 m) of a predefined POI is auto-labelled with the POI name in the waypoint list, and the POI dropdown follows live as you type (from a Custom selection, matching → that POI selected with snapped values; editing a preset → always **Custom**). In Single Goal mode preset POI coordinates are greyed but editable — changing a value drops the preset to Custom.

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

### "No Office simulation detected" when activating Nav2

The **Load Map + Nav** button attaches to an already-running sim —
press **Launch Map** first.

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
