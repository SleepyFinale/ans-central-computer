# Autonomous Navigation Systems: Central Computer - Autonomous Exploration Setup

This workspace contains editable TurtleBot3 packages for ROS 2 Humble, configured for autonomous exploration and mapping with Nav2, SLAM Toolbox, and Explore Lite.

## Table of Contents

1. [Prerequisites](#prerequisites)
   - [Installing Ubuntu 22.04 LTS Desktop](#installing-ubuntu-2204-lts-desktop)
   - [Installing ROS 2 Humble](#installing-ros-2-humble)
2. [Workspace Setup](#workspace-setup)
   - [Cloning the Repository](#cloning-the-repository)
   - [Building the Workspace](#building-the-workspace)
3. [Robot Configuration and ROS Domain](#robot-configuration-and-ros-domain)
4. [Multi-Robot SLAM](#multi-robot-slam)
   - [Robot Terminal 1: Robot Bringup](#robot-terminal-1-robot-bringup)
   - [Robot Terminal 2: SLAM + Nav2](#robot-terminal-2-slam--nav2)
   - [Central Terminals: start_central.sh + RViz](#central-terminals-start_centralsh--rviz)
5. [Troubleshooting](#troubleshooting)
6. [Diagnostic Commands](#diagnostic-commands)
7. [Additional Resources](#additional-resources)
8. [Workspace Structure](#workspace-structure)

---

## Prerequisites

### Installing Ubuntu 22.04 LTS Desktop

Before setting up the workspace, you need Ubuntu 22.04 LTS Desktop installed on your Remote PC.

**Download the Ubuntu 22.04 LTS Desktop image:**

- Visit: <https://releases.ubuntu.com/22.04/>
- Download the **64-bit PC (AMD64) desktop image** (`ubuntu-22.04.5-desktop-amd64.iso`)

**Installation instructions:**

- Follow the official Ubuntu installation guide: <https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview>
- The guide covers:
  - Creating a bootable USB stick
  - Booting from USB
  - Installation setup and configuration
  - Completing the installation

**System requirements:**

- At least 25GB of storage space
- A flash drive (12GB or above recommended) for the installation media
- At least 1024MiB of RAM

---

### Installing ROS 2 Humble

After installing Ubuntu 22.04 LTS, install ROS 2 Humble on your Remote PC.

**Follow the official ROS 2 installation guide:**

- <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>

The installation process includes:

1. Setting up locale and sources
2. Installing ROS 2 packages
3. Setting up the environment
4. Installing additional tools (colcon, argcomplete, etc.)

**Quick summary:**

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo add-apt-repository "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Install Navigation2 and SLAM Toolbox packages (required - workspace uses system packages)
sudo apt install ros-humble-navigation2 ros-humble-slam-toolbox

# Install development tools
sudo apt install python3-colcon-common-extensions python3-argcomplete

# Source ROS 2 setup
source /opt/ros/humble/setup.bash
```

**Note:** This workspace uses system Navigation2 packages (installed via apt) for faster builds. The workspace contains:

- TurtleBot3 core packages
- Navigation2 configuration and launch files
- Explore Lite for autonomous exploration
- Custom launch files and scripts

The Navigation2 source code is included in the repository for reference, but the build scripts use the system-installed packages instead.

---

## Workspace Setup

### Cloning the Repository

Clone the repository to your workspace directory:

```bash
cd ~
git clone https://github.com/SleepyFinale/turtlebot3-workspace.git turtlebot3_ws
cd ~/turtlebot3_ws
```

This repository contains all the necessary packages for TurtleBot3 autonomous exploration:

- TurtleBot3 core packages (DynamixelSDK, turtlebot3_msgs, turtlebot3)
- Navigation2 configuration
- Explore Lite for autonomous exploration
- Custom launch files and scripts

You can now branch, push, and pull changes as needed for your development workflow.

---

### Building the Workspace

The workspace includes two build scripts in the `scripts/` folder to help you build and source everything:

#### `scripts/clean_rebuild.sh`

Performs a complete clean rebuild of the entire workspace:

- Removes all build artifacts (`build/`, `install/`, `log/` directories)
- Checks for system dependencies
- Builds all packages from scratch
- Sources the workspace automatically after build

**Usage:**

```bash
cd ~/turtlebot3_ws
./scripts/clean_rebuild.sh
```

**When to use:**

- First-time setup
- After major changes to multiple packages
- When experiencing build issues that require a clean slate
- After pulling significant changes from the repository

#### `scripts/minimal_rebuild.sh`

Performs a minimal rebuild of only essential packages:

- Removes build artifacts
- Builds only packages needed for:
  - `turtlebot3_bringup robot.launch.py`
  - `turtlebot3_navigation2 navigation2.launch.py`
  - `explore_lite` (for the explorer)

**Usage:**

```bash
cd ~/turtlebot3_ws
./scripts/minimal_rebuild.sh
```

**When to use:**

- After making small changes to specific packages
- Faster rebuild times during development
- When you only need to update navigation or exploration components

**Note:** Both scripts automatically source the workspace after building. All helper scripts (robot setup, build, SLAM, explorer) are located in the `scripts/` folder. To set the robot environment before connecting: `source scripts/set_robot_env.sh <robot> [ip]`. If you need to manually source the workspace:

```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## Robot Configuration and ROS Domain

To connect this central computer to the TurtleBot3 robots you need:

- The **correct SSH target (IP/hostname)** for each robot
- The **same WiFi network** between robot and central PC
- A shared **ROS domain** (`ROS_DOMAIN_ID=50`) on all machines

### Robot SSH targets

The table below lists the SSH targets for each robot on the supported WiFi networks.

| Robot  | Lab (SSID: SNS)       | RaspAP (rpi)        | Azure (SSID: Azure) |
| ------ | --------------------- | ------------------- | --------------------|
| Blinky | blinky@192.168.0.158  | blinky@10.3.141.220 | blinky@172.20.10.13 |
| Pinky  | pinky@192.168.0.194   | pinky@10.3.141.194  | pinky@172.20.10.14  |
| Inky   | inky@192.168.0.139    | inky@10.3.141.139   | inky@172.20.10.15   |
| Clyde  | `clyde@<IP>`          | `clyde@<IP>`        | `clyde@<IP>`        |

### Using `set_robot_env.sh` to SSH into a robot

`scripts/set_robot_env.sh` sets `ROBOT_SSH` for the selected robot. For **Blinky**, **Pinky**, and **Inky**, it **auto-detects** which WiFi your PC is on (SNS, RaspAP, or Azure) and picks the correct IP. Only **Clyde** requires you to pass the robot's IP (no fixed IPs per network).

From the workspace root on the **central PC**, source the script so variables apply to your current shell:

```bash
cd ~/turtlebot3_ws

# Blinky, Pinky, or Inky (fixed IPs – script auto-detects SNS vs RaspAP vs Azure)
source scripts/set_robot_env.sh blinky
# or
source scripts/set_robot_env.sh pinky
# or
source scripts/set_robot_env.sh inky

# Clyde (pass the robot's IP address for the network you're on)
source scripts/set_robot_env.sh clyde 192.168.0.xxx
```

Then SSH into the robot:

```bash
ssh $ROBOT_SSH
```

**Script output:** The script prints the detected network (`lab`, `rpi`, or `azure`) so you can confirm it picked the right one. Example: `Robot: Blinky  ROBOT_SSH=blinky@192.168.0.158  (network: lab)`.

When switching between robots, run `source scripts/set_robot_env.sh <robot> [ip]` again in each terminal (or open new terminals and source once).

### ROS domain (ROS_DOMAIN_ID)

All robots and the central PC share a **single ROS 2 domain**, `ROS_DOMAIN_ID=50`. Every machine that needs to talk to the robots should use this same value:

```bash
echo $ROS_DOMAIN_ID
```

If you see a different value on any terminal (robot or central PC), set:

```bash
export ROS_DOMAIN_ID=50
```

---

## Multi-Robot SLAM

This repository is the **central computer** side of a multi-robot SLAM system. Each TurtleBot3 robot runs **bringup + SLAM + Nav2 on the robot SBC**, and the central PC handles coordination (`start_central.sh`) and visualization (RViz).

You can connect to **Blinky**, **Pinky**, **Inky**, or **Clyde**—use the [robot table](#robot-configuration-and-ros-domain) and `scripts/set_robot_env.sh` so `ROBOT_SSH` matches the robot you want. For full SBC setup details, see the robot-side README in the `ans-turtlebot3` repo.

**Prerequisites:**

- Robot is powered on and connected to the network (SNS, RaspAP, or Azure WiFi)
- Remote PC is on the **same** WiFi network as the robot
- Remote PC has ROS 2 Humble installed
- Workspace is built (see [Building the Workspace](#building-the-workspace))
- Robot environment is set (see [Robot Configuration and ROS Domain](#robot-configuration-and-ros-domain)): `source scripts/set_robot_env.sh <robot>` — this sets `ROBOT_SSH` appropriately.

**Startup order is critical:** For each robot, use **two SSH terminals** to the robot SBC, then start the central stack on the remote PC.

---

### Robot Terminal 1: Robot Bringup

**Purpose:** On the **robot SBC** (after SSH from the central PC), launch the core TurtleBot3 bringup (sensors, base, TF).

**Commands (on the robot SBC):**

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_bringup robot.launch.py
```

**Expected output (if working correctly):**

```text
[INFO] [launch]: All log files can be found below /home/<robot_user>/.ros/log/<date-time>-<robot_namespace>-<pid>
[INFO] [launch]: Default logging verbosity is set to INFO
urdf_file_name : turtlebot3_burger.urdf
[INFO] [robot_state_publisher-1]: process started with pid [<pid>]
[INFO] [ld08_driver-2]: process started with pid [<pid>]
[INFO] [turtlebot3_ros-3]: process started with pid [<pid>]
[turtlebot3_ros-3] [INFO] [...] [<robot>.turtlebot3_node]: Init TurtleBot3 Node Main
[turtlebot3_ros-3] [INFO] [...] [<robot>.turtlebot3_node]: Init DynamixelSDKWrapper
[turtlebot3_ros-3] [INFO] [...] [DynamixelSDKWrapper]: Succeeded to open the port(/dev/ttyACM<idx>)!
[turtlebot3_ros-3] [INFO] [...] [DynamixelSDKWrapper]: Succeeded to change the baudrate!
[turtlebot3_ros-3] [INFO] [...] [<robot>.turtlebot3_node]: Start Calibration of Gyro
[robot_state_publisher-1] [INFO] [...] [<robot>.robot_state_publisher]: got segment <robot>/base_footprint
[robot_state_publisher-1] [INFO] [...] [<robot>.robot_state_publisher]: got segment <robot>/base_link
[robot_state_publisher-1] [INFO] [...] [<robot>.robot_state_publisher]: got segment <robot>/base_scan
[robot_state_publisher-1] [INFO] [...] [<robot>.robot_state_publisher]: got segment <robot>/caster_back_link
[robot_state_publisher-1] [INFO] [...] [<robot>.robot_state_publisher]: got segment <robot>/imu_link
[robot_state_publisher-1] [INFO] [...] [<robot>.robot_state_publisher]: got segment <robot>/wheel_left_link
[robot_state_publisher-1] [INFO] [...] [<robot>.robot_state_publisher]: got segment <robot>/wheel_right_link
[ld08_driver-2] /dev/ttyACM0    u-blox 7 - GPS/GNSS Receiver
[ld08_driver-2] /dev/ttyUSB0    CP2102 USB to UART Bridge Controller
[ld08_driver-2] /dev/ttyACM<idx>    OpenCR Virtual ComPort in FS Mode
[ld08_driver-2] FOUND LDS-02
[ld08_driver-2] LDS-02 started successfully
[turtlebot3_ros-3] [INFO] [...] [<robot>.turtlebot3_node]: Calibration End
[turtlebot3_ros-3] [INFO] [...] [<robot>.turtlebot3_node]: Add Motors
[turtlebot3_ros-3] [INFO] [...] [<robot>.turtlebot3_node]: Add Wheels
[turtlebot3_ros-3] [INFO] [...] [<robot>.turtlebot3_node]: Add Sensors
[turtlebot3_ros-3] [INFO] [...] [<robot>.turtlebot3_node]: Succeeded to create battery state publisher
[turtlebot3_ros-3] [INFO] [...] [<robot>.turtlebot3_node]: Succeeded to create imu publisher
[turtlebot3_ros-3] [INFO] [...] [<robot>.turtlebot3_node]: Succeeded to create ultrasonic publisher
[turtlebot3_ros-3] [INFO] [...] [<robot>.turtlebot3_node]: Succeeded to create sensor state publisher
[turtlebot3_ros-3] [INFO] [...] [<robot>.turtlebot3_node]: Succeeded to create joint state publisher
[turtlebot3_ros-3] [INFO] [...] [<robot>.turtlebot3_node]: Add Devices
[turtlebot3_ros-3] [INFO] [...] [<robot>.turtlebot3_node]: Succeeded to create motor power server
[turtlebot3_ros-3] [INFO] [...] [<robot>.turtlebot3_node]: Succeeded to create reset server
[turtlebot3_ros-3] [INFO] [...] [<robot>.turtlebot3_node]: Succeeded to create sound server
[turtlebot3_ros-3] [INFO] [...] [<robot>.turtlebot3_node]: Run!
[turtlebot3_ros-3] [INFO] [...] [<robot>.diff_drive_controller]: Init Odometry
[turtlebot3_ros-3] [INFO] [...] [<robot>.diff_drive_controller]: Run!
```

**What to look for:**

- No error messages about device connections
- Messages indicating successful initialization
- Namespaced topics publishing for your robot, e.g.:
  - `/robot/battery_state`
  - `/robot/cmd_vel`
  - `/robot/imu`
  - `/robot/joint_states`
  - `/robot/magnetic_field`
  - `/robot/odom`
  - `/robot/robot_description`
  - `/robot/scan`
  - `/robot/sensor_state`
  - `/robot/tf`, `/robot/tf_static`
  - `/robot/ultrasonic`
- Robot should respond to velocity commands

**Verification (from the central PC):**

```bash
cd ~/turtlebot3_ws
ros2 topic list | grep "/<robot>/" 
ros2 topic echo /<robot>/scan --once  # Should show laser scan data
```

---

### Robot Terminal 2: SLAM + Nav2

**Purpose:** On the **same robot SBC** (second SSH terminal), run SLAM and Nav2.

**Commands (on the robot SBC):**

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py \
  use_sim_time:=False \
  use_rviz:=False
```

This launch file (from the robot workspace, e.g. `ans-turtlebot3`) runs:

- SLAM Toolbox (live, namespaced `/map`)
- The laser scan normalizer
- Nav2 (planner, controller, BT navigator, costmaps)

all **namespaced per robot** (e.g. `/<robot>/...`) so multiple robots can share a single `ROS_DOMAIN_ID` with the central PC.

**Expected output (if working correctly):**

```text
[INFO] [launch]: All log files can be found below /home/<robot_user>/.ros/log/<date-time>-<robot_namespace>-<pid>
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [normalize_laser_scan.py-1]: process started with pid [<pid>]
[INFO] [async_slam_toolbox_node-2]: process started with pid [<pid>]
[INFO] [python3-3]: process started with pid [<pid>]
[async_slam_toolbox_node-2] [INFO] [...] [<robot>.slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
[normalize_laser_scan.py-1] [INFO] [...] [<robot>.laser_scan_normalizer]: Laser scan normalizer started: scan -> scan_normalized (normalizing to 228 readings, publishing every 1 scan(s))
[normalize_laser_scan.py-1] [INFO] [...] [<robot>.laser_scan_normalizer]: Scan 1: received <N_in> readings, normalizing to 228
[normalize_laser_scan.py-1] [INFO] [...] [<robot>.laser_scan_normalizer]: Scan 1: published 228 readings (target: 228)
[async_slam_toolbox_node-2] Registering sensor: [Custom Described Lidar]
[python3-3] [INFO] [...] [wait_for_tf]: Waiting for TF (odom only). Need <robot>/odom->(one of ['<robot>/base_footprint', '<robot>/base_link']). Timeout: 30.0s
[python3-3] [INFO] [...] [wait_for_tf]: TF ready: <robot>/odom -> <robot>/base_footprint
[python3-3] [INFO] [...] [wait_for_tf]: TF tree looks ready.
[INFO] [python3-3]: process has finished cleanly [pid <pid>]
[INFO] [controller_server-4]: process started with pid [<pid>]
[INFO] [smoother_server-5]: process started with pid [<pid>]
[INFO] [planner_server-6]: process started with pid [<pid>]
[INFO] [behavior_server-7]: process started with pid [<pid>]
[INFO] [bt_navigator-8]: process started with pid [<pid>]
[INFO] [waypoint_follower-9]: process started with pid [<pid>]
[INFO] [velocity_smoother-10]: process started with pid [<pid>]
[INFO] [lifecycle_manager-11]: process started with pid [<pid>]
[lifecycle_manager-11] [INFO] [...] [<robot>.lifecycle_manager_navigation]: Managed nodes are active
...
[lifecycle_manager-11] [INFO] [...] [<robot>.lifecycle_manager_navigation]: Managed nodes are active
[lifecycle_manager-11] [INFO] [...] [<robot>.lifecycle_manager_navigation]: Creating bond timer...

```

You will also see the individual Nav2 lifecycle nodes configuring and activating their costmaps and behavior tree, similar to the detailed output shown above.

**What to look for:**

- SLAM Toolbox and the laser scan normalizer both start without errors.
- The TF wait script (`wait_for_tf`) reports that `<robot>/odom -> <robot>/base_footprint` is ready.
- Nav2 lifecycle nodes reach the **Managed nodes are active** state.
- Namespaced SLAM + Nav2 topics publishing for your robot, for example:
  - `/robot/scan_normalized`
  - `/robot/map`, `/robot/map_metadata`, `/robot/map_updates`
  - `/robot/global_costmap/*`, `/robot/local_costmap/*`
  - `/robot/plan`, `/robot/plan_smoothed`, `/robot/local_plan`
  - `/robot/cmd_vel_nav`
  - `/robot/behavior_server/transition_event`, `/robot/bt_navigator/transition_event`, `/robot/waypoint_follower/transition_event`, `/robot/velocity_smoother/transition_event`, etc.

**Verification (from the central PC):**

```bash
cd ~/turtlebot3_ws
source scripts/set_robot_env.sh <robot>
ros2 topic list | grep "/<robot>/"
ros2 topic echo /<robot>/map --once           # Should show map data after SLAM initializes
ros2 action list | grep navigate_to_pose      # Should show /<robot>/navigate_to_pose
```

At this point, the robot is fully brought up with SLAM + Nav2 running **on the SBC**. Next, start the central coordination stack.

---

### Central Terminals: start_central.sh + RViz

After at least one robot is running bringup + SLAM + Nav2 as above (repeat Robot Terminal 1 and 2 for each robot you want to use), use two terminals on the **central PC**:

- **Central Terminal 1 – coordinator stack**

  ```bash
  cd ~/turtlebot3_ws
  ./scripts/start_central.sh
  ```

  This script:

  - Detects available robot namespaces (e.g. `/blinky`, `/pinky`, `/inky`)
  - Starts TF relay
  - In **single-robot mode**: skips map merge and treats `/<robot>/map` in frame `map` as the world map
  - In **multi-robot mode**: starts multi-robot map merge (unknown initial poses) and merges all per-robot maps into a global `/map`
  - Launches `multi_robot_explorer.py` to assign frontiers via `/<robot>/navigate_to_pose`

  **Expected output (if working correctly):**

  ```text
  ==========================================
    Central Computer — Multi-Robot Exploration
  ==========================================

    ROS_DOMAIN_ID = 50
    Robot filter    = (all detected robots)

    Explorer fallback = use_pose_goal_fallback=false
    Explorer frequency override = (using YAML default)

  Detecting robots from ROS topics...
  Detected robots: <robot1> [<robot2> ...]
  Using robots   : <robot1> [<robot2> ...]

    Mode          = single-robot (Nav2 on robot, no map_merge)
    # or, when multiple robots are detected:
    Mode          = multi-robot (Nav2 on robots, map_merge enabled)

  [1/3] Single-robot mode detected — starting TF relay without frame prefixing...
  # or: [1/3] Multi-robot mode detected — starting TF relay with frame prefixing...
  [INFO] [...] [tf_relay_multirobot]: TF relay started: merging ['<robot1>' ...] -> /tf (prefix_frames=<true|false>)

  [2/3] Single-robot setup detected (<robot1>) — skipping map merge.
  # or (multi-robot): [2/3] Starting map merge (unknown poses)...

  [3/3] Starting single-robot explorer (Nav2 offloaded to robot)...
  # or (multi-robot): [3/3] Starting multi-robot explorer...

  ==========================================
    All services running.  Press Ctrl+C to stop.
  ==========================================

    To visualise: rviz2  (add /map display, set frame to 'map')

  [INFO] [...] [multi_robot_explorer]: Multi-robot explorer started: robots=['<robot1>' ...], map_topic=/<robot_or_global_map>, world_frame=map, freq=<F> Hz, use_pose_goal_fallback=False, mode=<single_robot_offloaded_nav2|multi_robot>
  [INFO] [...] [multi_robot_explorer]: Waiting for merged map on configured topic...
  ```

  **What to look for:**

  - `ROS_DOMAIN_ID` printed as `50` and the expected robot filter (e.g. `-bpi` when you pass `./scripts/start_central.sh -bpi`).
  - The **list of detected robots** matches the robots you actually have running.
  - TF relay starts without errors (`tf_relay_multirobot` node up).
  - In multi-robot mode, map merge starts and no errors are reported about missing input maps.
  - `multi_robot_explorer` starts for the selected robots and logs that it is waiting for the (merged or single-robot) map.

  **Topics published from the central stack (examples):**

  - `/tf`, `/tf_static` (from TF relay)
  - `/explore/frontiers` (from `multi_robot_explorer`)

- **Central Terminal 2 – RViz visualization**

  ```bash
  cd ~/turtlebot3_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash

  rviz2 -d $(ros2 pkg prefix turtlebot3_navigation2)/share/turtlebot3_navigation2/rviz/tb3_navigation2.rviz
  ```

  Set the fixed frame to `map`. You should see the live map, robot poses, paths, and frontier goals as the explorer runs.

---

## Troubleshooting

### Common Issues and Solutions

#### 1. Build error: missing `nav2_msgs` / Navigation2 packages

**Symptoms:**

```text
CMake Error at CMakeLists.txt:40 (find_package):
  By not providing "Findnav2_msgs.cmake" in CMAKE_MODULE_PATH this project
  has asked CMake to find a package configuration file provided by
  "nav2_msgs", but CMake did not find one.
```

**Cause:** Navigation2 system packages are not installed. The workspace is configured to use system Navigation2 packages (installed via apt) rather than building them from source.

**Fix:**

```bash
sudo apt update
sudo apt install ros-humble-navigation2
```

This will install all Navigation2 packages including `nav2_msgs`, which is required by `explore_lite`.

**Verification:**

After installing, verify the package is available:

```bash
source /opt/ros/humble/setup.bash
ros2 pkg list | grep nav2_msgs
```

You should see `nav2_msgs` in the list. Then try building again:

```bash
./scripts/clean_rebuild.sh
```

**Note:** The build scripts now automatically check for Navigation2 packages and will provide a helpful error message if they're missing.

---

#### 2. `ROS_DOMAIN_ID` mismatch (topics not visible)

**Symptoms:**

- Topics from robot not visible on Remote PC (or vice versa)
- `ros2 topic list` shows different topics on robot vs Remote PC
- Nodes can't see each other

**Fix:**

- **Step 1**: Check `ROS_DOMAIN_ID` on robot (SSH terminal).

  ```bash
  echo $ROS_DOMAIN_ID
  ```

- **Step 2**: Check `ROS_DOMAIN_ID` on Remote PC (each terminal you launched nodes from).

  ```bash
  echo $ROS_DOMAIN_ID
  ```

- **Step 3**: Set the same value everywhere. Easiest: use the setup script for your robot (see [Robot Configuration and ROS Domain](#robot-configuration-and-ros-domain)).

  ```bash
  cd ~/turtlebot3_ws
  source scripts/set_robot_env.sh <robot>
  ```

- **Step 4**: Restart terminals (or `source ~/.bashrc`) so every process uses the same domain.

---

#### 3. SSH connection fails or times out (wrong WiFi network)

**Symptoms:**

- `ssh $ROBOT_SSH` hangs, times out, or "Connection refused"
- Robot is powered on but unreachable

**Cause:** Your Remote PC and the robot are on different WiFi networks. The robot uses different IPs on Lab (SNS), RaspAP (rpi), and Azure—if the robot is on one network but your PC is on another, you will connect to the wrong IP.

**Fix:**

- **Step 1**: Confirm which WiFi the robot is connected to (check the robot or its display, if available).
- **Step 2**: Connect your Remote PC to the **same** WiFi (SNS for Lab, RaspAP for rpi, Azure for Azure).
- **Step 3**: Run `source scripts/set_robot_env.sh <robot>` again. The script auto-detects your PC's WiFi and sets the correct IP. Check the output—it should show `(network: lab)`, `(network: rpi)`, or `(network: azure)`.
- **Step 4**: If you see "Unknown WiFi" or "defaulting to Lab", your PC's WiFi may not be SNS, RaspAP, or Azure. Connect to the correct network and source the script again.

---

#### 4. TF errors: `base_link` / `base_footprint` / `odom` frame does not exist

**Symptoms:**

```text
Timed out waiting for transform from base_footprint to odom to become available, tf error:
Invalid frame ID "base_footprint" ... frame does not exist
```

**Cause:** Nav2 is starting before it has received the robot TF (`odom -> base_*`), or the Remote PC is not receiving TF from the robot (often a `ROS_DOMAIN_ID` mismatch).

**Fix:**

- **Step 1**: Confirm your `ROS_DOMAIN_ID` is correct in the terminal running Nav2/SLAM and in the SSH robot terminal.

  ```bash
  echo $ROS_DOMAIN_ID
  ```

- **Step 2**: Confirm TF is actually arriving on the Remote PC.

  ```bash
  ros2 topic echo /tf --once
  ```

  You should see at least:
  - `map -> odom` (from SLAM Toolbox)
  - `odom -> base_*` (from robot bringup / odometry / robot_state_publisher)

- **Step 3**: Confirm the exact transforms Nav2 needs.

  ```bash
  ros2 run tf2_ros tf2_echo map odom
  ros2 run tf2_ros tf2_echo odom base_footprint
  ```

**Note:** `navigation2_slam.launch.py` includes a TF wait step (`wait_for_tf.py`) to reduce this startup race. If TF never appears, the issue is upstream (robot bringup or networking / DDS).

---

#### 5. RViz errors about Nav2 panels / GLSL

**Symptoms:**

- `nav2_rviz_plugins/Selector` or `nav2_rviz_plugins/Docking` failed to load
- GLSL error: `active samplers with a different type refer to the same texture image unit`

**Cause:** RViz plugin / GPU driver quirks. The workspace RViz config has been updated to remove the Selector and Docking panels (they are not provided by the installed `nav2_rviz_plugins` on some setups), so those plugin errors should no longer appear after a rebuild.

**Workarounds:**

- If the map still renders and Nav2 works, you can ignore any remaining messages.
- If RViz rendering is broken, try software rendering:

```bash
LIBGL_ALWAYS_SOFTWARE=1 rviz2
```

---

#### 6. RViz exit code -11 (SIGSEGV) or nav2_container slow to terminate on Ctrl+C

**Symptoms:**

- After pressing Ctrl+C to stop the launch: `[ERROR] [rviz2-6]: process has died [pid ..., exit code -11]`
- `component_container_isolated-2` fails to terminate within 5–10 seconds and is killed with SIGTERM then SIGKILL

**Cause:** Known quirks: RViz2 can segfault on shutdown on some GPU/driver combinations; the composed Nav2 container can take a long time to deactivate all lifecycle nodes.

**Workarounds:**

- These do not affect normal operation. You can ignore the exit-code messages after Ctrl+C.
- To avoid waiting, close RViz’s window first, then Ctrl+C the terminal; the rest of the processes usually stop more quickly.

---

#### 7. Costmap warning: “Sensor origin is out of map bounds”

**Cause:** The global costmap's static layer uses the merged map; if the map bounds start at (0, 0), the lidar (sensor) at about (-0.03, 0) in the robot frame can fall outside the map when the robot is near the origin, so Nav2 warns that it cannot raytrace for it.

**Symptoms:**

```text
[WARN] [blinky.global_costmap.global_costmap]: Sensor origin at (-0.03, -0.00) is out of map bounds (0.00, 0.00) to (4.98, 4.98)
[WARN] [pinky.global_costmap.global_costmap]: Sensor origin at (-0.03, -0.00) is out of map bounds (0.00, 0.00) to (4.98, 4.98)
```

**Fix (multi-robot / map_merge):**

- The workspace configures **map_merge** with `origin_margin: 0.05` (in `config/map_merge/multirobot_params.yaml`). That adds a small padding around the merged map so the map bounds extend beyond (0, 0), which removes this warning. Rebuild and restart multi-robot SLAM so map_merge uses the updated params:
  - `./scripts/minimal_rebuild.sh` (or `clean_rebuild.sh`), then
  - `./scripts/start_multirobot_slam.sh`, then
  - `./scripts/start_multirobot_nav2_explore.sh`
- You can increase the margin if needed (e.g. `origin_margin: 0.1`).

**Fix (single-robot):**

- Ensure SLAM is publishing a map (`ros2 topic echo /map --once`) and TF is valid (`tf2_echo map base_footprint`). If using static map + AMCL, set the initial pose in RViz.
- This warning can be normal until the initial pose is set; it may not prevent Nav2 from working once the robot is localized.

---

#### 8. No map appearing in SLAM (`/map` topic missing or not publishing)

**Cause:** SLAM Toolbox not receiving scan data, not initialized yet, or needs more time.

**Symptoms:**

- `/map` topic doesn't appear in `ros2 topic list`
- `/map` topic exists but `ros2 topic echo /map --once` shows "does not appear to be published yet"

**Fix:**

- **Step 1**: Check scan data is available.

  ```bash
  ros2 topic echo /scan --once
  ```

- **Step 2**: Check scan frequency (should be ~10 Hz, depending on the lidar).

  ```bash
  ros2 topic hz /scan
  ```

- **Step 3**: If scans are present, give SLAM time to initialize (20–40 seconds is normal). Moving the robot slightly can help.

  ```bash
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'
  ```

- **Step 4**: Verify SLAM node is running.

  ```bash
  ros2 node list | grep slam
  ```

- **Step 5**: Check the SLAM terminal for errors.

**Note:** It's normal for `/map` to not appear immediately. SLAM Toolbox needs to receive scan data, process several scans, build initial map, then start publishing `/map` topic. This typically takes 20-40 seconds from when SLAM starts.

---

#### 9. Explorer waiting for costmap

**Cause:** Nav2 costmap hasn't initialized yet (normal - takes 20-40 seconds).

**Symptoms:**

```text
[INFO] [explore_node]: Waiting for costmap to become available, topic: /global_costmap/costmap
```

**Fix:**

- **This is normal!** The explorer is designed to wait. Just be patient.
- The explorer will automatically connect once Nav2's costmap is ready.
- You'll see: `[INFO] [explore_node]: Exploration started` when ready.

**Total wait time:** Usually 20-40 seconds after Nav2 starts, but can take up to 60-90 seconds total from robot launch.

---

#### 10. Robot not moving / explorer not finding frontiers

**Cause:** System still initializing, or map too small.

**Fix:**

- Wait 60–90 seconds total from startup (robot + SLAM + Nav2 + explorer).
- Check explorer status:
  - Look for `[INFO] [explore_node]: Exploration started`
- Check goals:
  - `ros2 action list | grep navigate_to_pose`
- Check Nav2 is up:
  - `ros2 node list | grep nav2`
- Check TF is valid:
  - `ros2 run tf2_ros tf2_echo map odom`
  - `ros2 run tf2_ros tf2_echo odom base_footprint`
- Ensure the map has some free space and unknown space (explorer needs frontiers).

---

#### 11. "Starting point in lethal space" / "Collision Ahead - Exiting Spin" (robot stuck near walls/corners)

**Cause:** The planner thinks the robot is inside an obstacle (often due to costmap inflation when the robot is close to a wall or corner). During SLAM the map and costmap update continuously, so this can happen even in open space briefly. Recovery (spin/backup) may also see inflated obstacles and abort.

**Symptoms:**

- `GridBased: failed to create plan, invalid use: Starting point in lethal space!`
- `spin failed` / `Collision Ahead - Exiting Spin` / `backup failed`
- Robot gets close to a corner or doorframe and then stays there, not moving.

**Fix:** The Nav2 params in `turtlebot3_navigation2/param/humble/burger.yaml` are already tuned to reduce this: global costmap uses smaller inflation (0.32 m radius, cost_scaling_factor 5.0) and local costmap inflation is 0.45 m. If it still happens:

- Wait for recovery (wait → backup → spin) to move the robot into free space; often the next plan then succeeds.
- Drive the robot slightly away from the wall/corner so its center is in clearly free space.
- Optionally reduce `inflation_radius` further in the global/local costmap in the same param file, then rebuild and restart Nav2.

---

#### 12. Nav2: "No goal checker was specified in parameter 'current_goal_checker'"

**Cause:** The default Nav2 behavior tree does not set `goal_checker_id` on the FollowPath node, so the controller server reports that it’s using the only loaded goal checker (e.g. `general_goal_checker`).

**Fix:** None required. The warning appears **once** and is harmless; the server uses the correct goal checker. You can ignore it.

---

#### 13. Odom TF jumping away from map TF / map at a weird angle / straight walls look curved (odometry drift)

**Cause:** The `map`→`odom` transform is published by SLAM Toolbox. When it corrects for odometry drift, that correction can appear as a “jump” if updates are infrequent or large. Curved walls usually mean rotational odometry drift during mapping (robot thinks it’s going straight but odom says it’s turning).

**Symptoms:**

- In RViz, the robot or map seems to jump; odom frame moves away from map then snaps back.
- Jumps happen more often the farther the robot is from the start.
- Map looks rotated or straight corridors/walls appear curved.

**What we’ve done:** SLAM params in `mapper_params_online_async_fast.yaml` are tuned for smoother behavior:

- `map_update_interval: 0.35` — balance between update frequency and stability.
- `minimum_travel_distance` / `minimum_travel_heading: 0.18` — match often enough so pose updates are smaller.
- `transform_timeout: 0.2` — keeps map→odom timestamp closer to current time.

**If it still happens:**

- Ensure only **one** node publishes `map`→`odom` (SLAM Toolbox when using SLAM; do not run AMCL at the same time). The Nav2 panel showing “Localization: inactive” is normal when using SLAM.
- Check odometry: wheel slip, uneven floors, or miscalibrated wheel radius/separation (TurtleBot3: `turtlebot3_node/param/burger.yaml` — `wheels.separation`, `wheels.radius`) can cause drift and curved maps.
- If the robot has an IMU, ensure `use_imu: true` in the diff_drive/odometry config so orientation drift is reduced.
- **Periodic revisit:** In long or similar-looking corridors, enabling the explorer's periodic revisit can create loop-closure opportunities and reduce drift. In `src/m-explore-ros2/explore/config/params.yaml` set `revisit_enabled: true` and adjust `revisit_after_n_goals` (e.g. 3–5). The robot will return toward the start every N reached goals, then resume exploration.

---

#### 14. Two maps showing in RViz (static map + SLAM map)

**Cause:** Nav2 is loading a default static map file, and SLAM Toolbox is also publishing its live map. Both appear in RViz.

**Fix:**

- Hide the static map display in RViz (keep the live `/map` display), or
- Use the SLAM Nav2 launch (`navigation2_slam.launch.py`) so Nav2 does not load a static map.

**Note:** When doing SLAM/exploration, you typically want to use the live map from SLAM Toolbox, not a static map file.

---

#### 15. Odometry not publishing (`/odom` exists but no data)

**Cause:** Odometry needs robot movement to initialize, or parameters not loaded.

**Fix:**

- Move the robot slightly:

  ```bash
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'
  ```

- Check odometry:
  - `ros2 topic echo /odom --once`
- If still not working, restart robot bringup on the robot (`robot.launch.py`) and re-check `/odom`.

**Check:** `ros2 topic list | grep odom` - topic should exist and be publishing data.

---

#### 16. Multi-robot: TF wait timeout / "Can't update static costmap layer"

**Symptoms:**

- `wait_for_tf_multirobot` times out before starting Nav2
- Nav2 logs: `Can't update static costmap layer, no map received`
- Message Filter: `frame 'base_scan'... timestamp earlier than transform cache`

**Causes and fixes:**

1. **TF wait timeout:** Ensure your robots are running bringup + SLAM + Nav2 and that `./scripts/start_central.sh` is active on the central PC. Run diagnostics:

   ```bash
   ROS_DOMAIN_ID=50 python3 scripts/diagnose_multirobot_tf.py
   ```

2. **No map received:** In multi-robot mode, map merge publishes `/map` only after each robot’s SLAM has built an initial map. Wait 20–30 seconds after starting the robots and `start_central.sh` before expecting a global `/map`.
3. **frame 'base_scan':** Fixed by using `scan_normalized` with correct `frame_id` (e.g. `blinky/base_scan`, `pinky/base_scan`) from the robot-side workspace. Ensure the robot-side SLAM + Nav2 launch is up to date and running.

---

#### 17. Measuring pose jumps with `pose_jump_monitor.py`

Use the helper script `scripts/pose_jump_monitor.py` to quantify how much the robot’s reported pose in the map frame is "jumping" during SLAM. This is useful when the robot appears to teleport in RViz or when the explorer seems confused by localization corrections.

**How it works:**

- Subscribes to TF and repeatedly looks up the transform from `map` → `base_footprint` (configurable).
- Computes the linear distance between successive poses and logs when the distance is greater than or equal to a threshold.
- Tracks basic statistics (total jumps, max jump, average jump) and prints a summary when you stop it with Ctrl+C.

**Default behavior:**

- Map frame: `map`
- Base frame: `base_footprint`
- Polling rate: 10 Hz
- Minimum jump distance to log: 0.10 m

**Usage (on Remote PC):**

```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source scripts/set_robot_env.sh blinky   # or pinky, inky, clyde <IP>

python3 scripts/pose_jump_monitor.py
```

Leave this running while SLAM + Nav2 + explorer are active. When you press Ctrl+C, it will print a short summary such as:

```text
===== Pose Jump Summary =====
  Total jumps:     42
  Min distance:    0.300 m
  Max jump:        0.380 m
  Avg jump:        0.215 m
```

**Tuning via environment variables (optional):**

You can adjust frames, threshold, and rate before launching the script:

```bash
export POSE_JUMP_MAP_FRAME=map
export POSE_JUMP_BASE_FRAME=base_footprint
export POSE_JUMP_MIN_DISTANCE_M=0.3   # only log jumps >= 30 cm
export POSE_JUMP_RATE_HZ=5.0          # poll TF at 5 Hz

python3 scripts/pose_jump_monitor.py
```

These settings help you focus on larger corrections that are more likely to confuse exploration and navigation.

---

## Diagnostic Commands

Use these commands to diagnose issues and verify system status:

### Check What's Running

```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Check scan frequency
ros2 topic hz /scan

# Check map publishing rate
ros2 topic hz /map
```

### Check TF Tree

```bash
# Show all frames
ros2 run tf2_ros tf2_monitor

# Check specific transform
ros2 run tf2_ros tf2_echo map base_footprint

# Check transform between map and odom
ros2 run tf2_ros tf2_echo map odom
```

### Check if Topics Are Publishing

```bash
# Should show laser data
ros2 topic echo /scan --once

# Should show odometry
ros2 topic echo /odom --once

# Should show map (wait a few seconds after SLAM starts)
ros2 topic echo /map --once
```

### Check Nav2 Status

```bash
# List Nav2 lifecycle services
ros2 service list | grep lifecycle

# List costmap topics
ros2 topic list | grep costmap

# Check Nav2 nodes
ros2 node list | grep nav2
```

### Check Explorer Status

```bash
# Check explorer node
ros2 node list | grep explore

# Check goals being sent
ros2 action list | grep navigate_to_pose

# Check explorer topics
ros2 topic list | grep explore
```

### Check ROS Domain ID

```bash
echo $ROS_DOMAIN_ID
```

### Run multi-robot TF diagnostics

```bash
cd ~/turtlebot3_ws
ROS_DOMAIN_ID=50 python3 scripts/diagnose_multirobot_tf.py
```

Reports topic publishers, TF chain connectivity, and identifies missing transforms. Run when TF wait times out or Nav2 fails to receive the map.

---

## Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/index.html)
- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [TurtleBot3](https://www.turtlebot.com/)
- [Ubuntu 22.04 LTS Download](https://releases.ubuntu.com/22.04/)
- [Ubuntu Installation Guide](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
- [ROS 2 Humble Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

---

## Workspace Structure

This workspace includes:

- **TurtleBot3 packages**: Core robot packages from ROBOTIS
- **Navigation2**: Navigation stack (uses system packages)
- **SLAM Toolbox**: For mapping (installed via apt)
- **Explore Lite**: Autonomous exploration package (in `src/m-explore-ros2/`)
- **Custom launch files**: Modified launch files for SLAM-based navigation
