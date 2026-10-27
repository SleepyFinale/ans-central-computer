# Central computer: multi-robot exploration

This workspace is the central-PC half of the TurtleBot3 fleet. Each robot (Blinky, Pinky, Inky, Clyde) runs bringup, SLAM Toolbox and Nav2 on its own Raspberry Pi; the robot workspace lives in `~/turtlebot3`. This PC merges the robots' maps, picks frontiers to explore, sends Nav2 goals, and shows everything in RViz. The two sides talk only through Zenoh over Tailscale.

How it fits together, and exactly what crosses the network: [docs/architecture.md](docs/architecture.md).

## Contents

1. [Run the fleet](#run-the-fleet)
2. [One-time setup](#one-time-setup)
3. [Robot access](#robot-access)
4. [What the central stack runs](#what-the-central-stack-runs)
5. [Troubleshooting](#troubleshooting)
6. [Diagnostics and logging](#diagnostics-and-logging)
7. [Repository layout](#repository-layout)

---



## Run the fleet

One command starts everything: central Zenoh, then bringup, SLAM + Nav2 and the Zenoh client on each selected robot over SSH, then the central stack and RViz.

```bash
cd ~/central-computer
./scripts/core/start_fleet.sh            # all four robots
./scripts/core/start_fleet.sh -ic        # inky + clyde (b=blinky p=pinky i=inky c=clyde)
./scripts/core/start_fleet.sh -i --no-rviz
./scripts/core/start_fleet.sh --dry-run -ic   # print the plan and SSH targets only
```

It stays in the foreground; Ctrl+C stops every process it started, here and on the robots. The terminal shows stage progress plus warnings and errors (`--verbose` prints everything). Full per-process logs go to `$XDG_RUNTIME_DIR/fleet-bringup/<timestamp>/` (override with `FLEET_LOG_DIR`). Robots log in with `ROBOT_SSH_PASSWORD` (default `ubuntu`); robots dial this PC by its short hostname (`--central-host <name>` to override).

Each robot does a short forward move when its Nav2 comes up (map seeding), then waits for goals. Exploration ends when no reachable frontiers remain; robots then drive back to where they started (their SLAM map origin). If an obstacle later disappears (a door opens), exploration resumes. See [Changing surroundings](docs/architecture.md#changing-surroundings).

### Running the pieces by hand

Central PC, terminal 1, then each robot (three SSH terminals), then central terminals 2 and 3:

```bash
# central 1: Zenoh router + fleet receiver
./scripts/comms/start_zenoh_central.sh -ic

# each robot (in ~/turtlebot3)
ros2 launch turtlebot3_bringup robot.launch.py
ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py
./scripts/comms/start_zenoh_robot.sh pcname     # this PC's Tailscale name

# central 2: map merge + explorer (waits up to 30 s for robot maps)
./scripts/core/start_central.sh -ic
# central 3: RViz
./scripts/core/start_rviz_central.sh             # merged map; one robot -> that robot's view
./scripts/core/start_rviz_central.sh -r inky     # one robot's map, scan, plan, local costmap
```

To poke at the fleet from another terminal, load the same DDS settings the stack uses:

```bash
source scripts/env/central_env.bash
ros2 topic echo /map --once
ros2 action list                                  # /<robot>/navigate_to_pose, ...
```

---



## One-time setup



### Central PC

Ubuntu 22.04 and [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) (`ros-humble-desktop`), then:

```bash
sudo apt install ros-humble-navigation2 ros-humble-rmw-cyclonedds-cpp \
  python3-colcon-common-extensions python3-yaml python3-numpy libopencv-dev sshpass
cd ~
git clone https://github.com/SleepyFinale/turtlebot3-workspace.git central-computer
cd central-computer
bash scripts/build/rebuild_common.sh          # builds multirobot_map_merge (~15 s)
./scripts/comms/install_zenoh_bridge.sh       # pinned zenoh-bridge-ros2dds 1.7.2
```

Ubuntu 22.04: do not `apt install zenoh-bridge-ros2dds` (it needs glibc 2.38); the install script fetches the standalone build into `third_party/zenoh/`. Robots must run the same bridge version.

Only `multirobot_map_merge` is compiled. The explorer and the other central nodes are Python scripts that run straight from `src/` and `scripts/`, so editing them needs no rebuild.

### Tailscale

This PC and every robot join the same tailnet. Set each machine's Tailscale hostname to its own name (`sudo tailscale set --hostname=<name>`): robots are `blinky`, `pinky`, `inky`, `clyde`; this PC is its short hostname. Leave `--accept-routes` off. Check with `tailscale status`.

### Each robot

See the robot workspace README (`~/turtlebot3/README.md`) for SBC setup, building, and `scripts/comms/ROBOT_ZENOH_SETUP.md` for Zenoh.

---



## Robot access


| Robot  | Tailscale (preferred) | Azure hotspot         |
| ------ | --------------------- | --------------------- |
| Blinky | `blinky@blinky`       | `blinky@172.20.10.13` |
| Pinky  | `pinky@pinky`         | `pinky@172.20.10.14`  |
| Inky   | `inky@inky`           | `inky@172.20.10.15`   |
| Clyde  | `clyde@clyde`         | `clyde@172.20.10.16`  |


```bash
source scripts/env/set_robot_env.sh inky     # sets ROBOT_SSH for the network you are on
ssh $ROBOT_SSH
# without Tailscale on TAMU_WiFi: source scripts/env/set_robot_env.sh inky <dhcp-ip>
```

Azure hotspot: SSID `Azure`, password `howdoyouwanttodothis`, gateway `172.20.10.1/28`. Robots try TAMU_WiFi first and fall back to Azure (`~/turtlebot3/scripts/network/`). Zenoh is needed on both networks: robot DDS is loopback-only.

---



## What the central stack runs


| Process                                                        | Started by                    | Does                                                                                   |
| -------------------------------------------------------------- | ----------------------------- | -------------------------------------------------------------------------------------- |
| `zenoh-bridge-ros2dds` (router, `tcp/0.0.0.0:7447`)            | `start_zenoh_central.sh`      | puts every robot's `/<robot>/...` topics and Nav2 actions into ROS domain 50           |
| `fleet_zenoh_receive.py`                                       | `start_zenoh_central.sh`      | `tf_zenoh` -> `/tf`, `tf_static_zenoh` -> `/tf_static`, `map_wire_z` -> `/<robot>/map` |
| `map_merge`                                                    | `start_central.sh`, 2+ robots | merged `/map` and `map -> <robot>/map` TF                                              |
| `map_merge_state_monitor.py`                                   | `start_central.sh`, 2+ robots | `map_merge/merge_state` (NO_OVERLAP / PARTIAL / MERGED)                                |
| `single_robot_world_tf_bridge.py`, `single_robot_map_relay.py` | `start_central.sh`, 1 robot   | identity `map -> <robot>/map`, `/<robot>/map` -> `/map`                                |
| `multi_robot_explorer.py`                                      | `start_central.sh`            | frontiers, goal assignment, path precheck, `NavigateToPose`                            |


Parameters:

- `src/m-explore-ros2/explore/config/multi_robot_explorer.yaml`: explorer (frontier size, cost weights, watchdogs, blacklists, return-to-start).
- `src/m-explore-ros2/map_merge/config/multirobot_params_unknown_poses.yaml`: map merge.
- `config/fleet_domain_map.yaml`: central ROS domain (50).
- `config/zenoh/central_router.json5.tmpl`: what the bridge lets through.

`start_central.sh` knobs: `CENTRAL_DISCOVERY_TIMEOUT_SEC` (default 30), `EXPLORER_FREQUENCY_HZ`, `EXPLORER_TERMINAL_EVENT_LOG_MODE=summary|verbose`, `EXPLORER_TERMINAL_SUMMARY_PERIOD_SEC`, `CENTRAL_MAP_MERGE_RAW_STDOUT=true`, `CENTRAL_AUTO_KILL=true` (replace a stale central stack without asking).

The explorer prints a summary line per robot every 10 s:

```text
Summary: inky[goal_active] reached=4(+1) failed=0(+0) status=executing cancel=- pos=(2.65,0.52)
```

---



## Troubleshooting

`start_central.sh`**: "no robot map arrived".** Work outward from the robot: SLAM running on the robot (`ros2 topic echo --once /<robot>/map` there), `start_zenoh_robot.sh` running and showing `TF zenoh publish`, `start_zenoh_central.sh` running here, then `source scripts/env/central_env.bash; ros2 topic echo --once /<robot>/map_wire_z --field layout`. `tailscale ping <robot>` checks the link.

`start_fleet.sh` **times out at a stage.** It prints the last 40 lines of that stage's log and the line it was waiting for. Stage logs are in the `Logs:` directory it printed at start.

**"A central stack is already running".** A previous run left processes. Answer `y` to kill them, or stop them yourself; non-interactive shells need `CENTRAL_AUTO_KILL=true`.

**RViz shows no map or TF.** Use `start_rviz_central.sh`, not plain `rviz2`: it selects Cyclone and the participant-cap profile. A FastDDS RViz is on a different graph.

**Robot stops making progress / goals fail repeatedly.** Some failures are normal: frontiers behind obstacles fail the path precheck and get blacklisted. Look at the robot's SLAM log (`<logs>/<robot>-slam.log`) for `Control loop missed its desired rate` (Pi out of CPU; check `top` on the robot) or `Starting point in lethal space` (robot too close to a wall; recoveries usually free it).

`map_merge` **logs "Grid pose estimation disabled permanently"** after an OpenCV/FLANN exception: restart `start_central.sh`.

**RViz GLSL warning** (`active samplers with a different type refer to the same texture image unit`): driver quirk, harmless if the map draws. If rendering is broken, try `LIBGL_ALWAYS_SOFTWARE=1`.

**Two** `/tf` **publishers disagree about** `map -> <robot>/map`**.** A robot was started with `fleet_mode:=false` (bench mode publishes its own `map -> <robot>/map`). Restart it with the default `fleet_mode:=true`.

---



## Diagnostics and logging

All of these expect `source scripts/env/central_env.bash` first.

- `python3 scripts/diagnostics/diagnose_multirobot_tf.py`: checks `/tf`, `/map`, each robot's link, and the `map -> <robot>/base_footprint` chain.
- `./scripts/diagnostics/start_central_debug_bag.sh [robot ...]`: rosbag of `/map`, `/tf`, frontiers, merge state, and each robot's map, plan and goal status.
- `python3 scripts/diagnostics/pose_jump_monitor.py`: logs jumps in `map -> base_footprint` (`POSE_JUMP_BASE_FRAME=inky/base_footprint`, `POSE_JUMP_MIN_DISTANCE_M`, `POSE_JUMP_RATE_HZ`).
- `scripts/validation/run_fleet_validation.sh <scenario> <log> [candidate_log]`: KPI counts from captured logs.
- `scripts/plotting/`: live XY trail and robot bottleneck-monitor plots.
- The explorer writes structured JSONL events through `src/m-explore-ros2/explore/scripts/central_explorer_event_logger.py` (`DEBUG_SESSION_ID` correlates with robot logs).

---



## Repository layout

```text
config/
  fleet_domain_map.yaml        central ROS domain
  zenoh/central_router.json5.tmpl
  cyclonedds/central_localhost.xml   RViz only
  rviz/                        global and per-robot RViz configs
docs/architecture.md
scripts/
  core/      start_fleet.sh, start_central.sh, start_rviz_central.sh
  comms/     start_zenoh_central.sh, fleet_zenoh_receive.py, install_zenoh_bridge.sh
  env/       central_env.bash, fleet_common.bash, set_robot_env.sh
  build/     rebuild_common.sh
  diagnostics/, validation/, plotting/
  security/  watermark experiments and their test data (see scripts/security/README.md)
src/m-explore-ros2/
  explore/   multi_robot_explorer.py and helpers (Python, run from source)
  map_merge/ multirobot_map_merge (C++, built)
third_party/zenoh/             standalone zenoh-bridge-ros2dds (installed by script)
```

