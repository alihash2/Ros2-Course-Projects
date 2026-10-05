# ms04_autonomous_navigation

**Milestone 4 — Autonomous Navigation with a Mission Control GUI** for two simulated
environments: **Office** and **Warehouse**.

Everything runs inside a Docker container, so **you do not need ROS 2 installed
on your computer**. Clone the repo, allow the container to use your display, and
start it.

---

## What this package does

- **Two simulated environments** — a custom Gazebo `office.world` and
  `warehouse.world`, each already populated with the map used for navigation.
- **Autonomous SLAM mapping** — a frontier explorer that drives the robot to map
  an unknown space on its own, then saves the finished map automatically.
- **Navigation on the saved map** — Nav2 plans and drives the robot to a goal,
  using AMCL to localize itself on the map.
- **Mission Control GUI** — one PyQt5 window that controls the whole pipeline:
  start the simulation, switch on navigation, send goals, and watch what happens.
- **Full mission control while it drives** — start, pause, resume, cancel, or
  replace a mission even while the robot is moving.
- **Live feedback** — a colour-coded log shows each mission event as it happens
  (goal accepted, moving, paused, reached, aborted…), plus live progress like
  distance remaining and current waypoint.

---

## Requirements

| | |
|---|---|
| **OS** | Linux |
| **Needed** | Git, Docker, Docker Compose |
| **Not needed on host** | ROS 2, Python, Colcon, Gazebo, RViz — all of it lives in the container |
| **Display** | A graphical desktop session (the GUI is a window on your screen) |

---

## Install

The repository contains the package folders at its top level, so clone it
**directly into the `src/` folder** of a ROS 2 workspace (note the `.` at the
end — it puts the packages straight into `src/`, where they are expected).

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/alihash2/Ros2-Course-Projects.git .
```

Your workspace now looks like this — note that the Docker files live **inside the
package folder**, because the container only builds this one package:

```
~/ros2_ws/src/
├── ms01_cpp_foundations/
├── ms02_mobile_robot_control/
├── ms03_turtlebot3_control/
├── dynamic_obstacle_avoidance/
└── ms04_autonomous_navigation/    ← this package
    ├── docker-compose.yml        ← Docker is run from here
    ├── docker/
    │   └── Dockerfile
    ├── action/  msg/
    ├── include/  src/
    ├── launch/
    ├── maps/
    ├── models/
    ├── params/
    ├── rviz/
    ├── scripts/
    └── worlds/
```

---

## Run

**Run this from inside the package folder** — that is the folder holding
`docker-compose.yml` (`~/ros2_ws/src/ms04_autonomous_navigation`):

```bash
cd ~/ros2_ws/src/ms04_autonomous_navigation
xhost +local:docker
sudo docker compose up
```

That single command builds the image (first time only — it takes a few minutes),
starts the container, and opens the **Mission Control GUI**.

> `xhost +local:docker` lets the container draw its window on your desktop.
> It is safe to revoke afterwards with `xhost -local:docker`.

---

## Using the GUI

The Mission Control window is the whole workflow:

1. **Choose an environment** — Office or Warehouse — and start its simulation.
2. **Load the map and switch on navigation** — AMCL localizes the robot and Nav2
   takes over.
3. **Send a goal** — type an X / Y / yaw, click a predefined point of interest,
   or build a multi-waypoint list.
4. **Drive it** — press **Start**.
5. **Control it** — **Pause**, **Resume**, **Cancel**, or **Replace** the mission
   while the robot is still moving.
6. **Watch the log** — every mission event appears live and colour-coded.

Both environments ship with a finished map, so you can start navigating right
away — running a SLAM mapping session first is optional.

---

## What is inside the package

| Folder / file | Purpose |
|---|---|
| `docker-compose.yml` | Container definition — run `docker compose` from this folder |
| `docker/Dockerfile` | Builds the image and this package inside it |
| `scripts/mission_control_gui.py` | The Mission Control GUI (this is what the container launches) |
| `scripts/auto_slam_explorer.py` | Autonomous frontier explorer + automatic map saving |
| `src/navigation_coordinator_node.cpp` | C++ mission coordinator — start / pause / resume / cancel / replace, waypoint progress, recovery handling |
| `src/cmd_vel_relay.cpp` | C++ node that relays velocity commands |
| `launch/` | 7 launch files: simulation, mapping and navigation, per environment |
| `worlds/` | The custom Office and Warehouse Gazebo worlds |
| `params/` | Nav2, AMCL and SLAM Toolbox configuration |
| `maps/` | The saved Office and Warehouse maps |
| `msg/`, `action/` | Custom ROS 2 interfaces (see below) |

### Launch files

| Launch file | What it starts |
|---|---|
| `office_simulation.launch.py` | Office world + robot |
| `warehouse_simulation.launch.py` | Warehouse world + robot |
| `office_mapping.launch.py` | Autonomous SLAM mapping in the Office |
| `warehouse_mapping.launch.py` | Autonomous SLAM mapping in the Warehouse |
| `office_navigation.launch.py` | AMCL + Nav2 navigation in the Office |
| `warehouse_navigation.launch.py` | AMCL + Nav2 navigation in the Warehouse |
| `nav2_servers.launch.py` | The shared Nav2 server stack, for either environment |

### Custom ROS 2 interfaces

| Interface | Purpose |
|---|---|
| `msg/NavigationMission.msg` | Send a mission and a command (start / cancel / pause / resume / replace) |
| `msg/NavigationStatus.msg` | Current mission state and live progress |
| `msg/NavigationEvent.msg` | One event in the mission lifecycle, for logging |
| `action/ExecuteMission.action` | Execute a goal or waypoint list, with feedback and result |

---

## Everyday commands

Run these from the package folder (`~/ros2_ws/src/ms04_autonomous_navigation`),
where `docker-compose.yml` lives:

| Command | What it does |
|---|---|
| `sudo docker compose up` | Build (first run) and start the GUI |
| `sudo docker compose down` | Stop and remove the container |
| `sudo docker compose build` | Rebuild the image after changing code |
| `xhost -local:docker` | Revoke display access when you are done |

---

## Troubleshooting

**`docker compose` cannot find the configuration file**
You are probably in the wrong folder. `docker-compose.yml` lives inside this
package, so `cd` into `~/ros2_ws/src/ms04_autonomous_navigation` and run the
command there.

**The GUI window does not appear**
Run `xhost +local:docker` first, and check that `echo $DISPLAY` is set (for
example `:0`). Without a display there is nothing to draw the window on.

**The first `up` takes a long time**
Expected. The image installs the ROS 2 desktop and builds the package before the
GUI starts. Later starts are quick.

**Changed the code but see no difference**
Rebuild the image: `sudo docker compose build && sudo docker compose up`.