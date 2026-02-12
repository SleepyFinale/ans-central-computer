# Autonomous Navigation Systems: Central Computer - Autonomous Exploration Setup

This workspace contains editable TurtleBot3 packages for ROS 2 Humble, configured for autonomous exploration and mapping with Nav2, SLAM Toolbox, and Explore Lite.

## Table of Contents

1. [Prerequisites](#prerequisites)
   - [Installing Ubuntu 22.04 LTS Desktop](#installing-ubuntu-2204-lts-desktop)
   - [Installing ROS 2 Humble](#installing-ros-2-humble)
2. [Workspace Setup](#workspace-setup)
   - [Cloning the Repository](#cloning-the-repository)
   - [Building the Workspace](#building-the-workspace)
3. [ROS Domain Configuration](#ros-domain-configuration)
4. [Connecting to a Robot](#connecting-to-a-robot)
   - [Terminal 1: SSH Connection and Robot Launch](#terminal-1-ssh-connection-and-robot-launch)
   - [Terminal 2: SLAM Toolbox](#terminal-2-slam-toolbox)
   - [Terminal 3: Navigation2](#terminal-3-navigation2)
   - [Terminal 4: Explorer](#terminal-4-explorer)
5. [Troubleshooting](#troubleshooting)
6. [Diagnostic Commands](#diagnostic-commands)
7. [Additional Resources](#additional-resources)

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

# Install Navigation2 packages (required - workspace uses system packages)
sudo apt install ros-humble-navigation2

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

## ROS Domain Configuration

ROS 2 uses `ROS_DOMAIN_ID` to separate different robot networks. Each robot and the Remote PC must use the **same** `ROS_DOMAIN_ID` value to communicate.

**Robot configuration:**

| Robot  | ROS_DOMAIN_ID | Hostname / SSH target     |
| ------ | ------------- | ------------------------- |
| Blinky | 30            | blinky@192.168.0.158      |
| Pinky  | 31            | pinky@192.168.0.194       |
| Inky   | 32            | inky@\<IP\>               |
| Clyde  | 33            | clyde@\<IP\>              |

### Recommended: use the setup script

From the workspace root, **source** the script so `ROS_DOMAIN_ID` and `ROBOT_SSH` are set in your current shell:

```bash
cd ~/turtlebot3_ws

# Blinky or Pinky (fixed IPs)
source scripts/set_robot_env.sh blinky
# or
source scripts/set_robot_env.sh pinky

# Inky or Clyde (pass the robot's IP address)
source scripts/set_robot_env.sh inky 192.168.50.xxx
source scripts/set_robot_env.sh clyde 192.168.50.xxx
```

Then connect with:

```bash
ssh $ROBOT_SSH
```

**Manual setup (alternative):**

```bash
# Set ROS_DOMAIN_ID for the robot you're using (see table above)
export ROS_DOMAIN_ID=30   # Blinky
# export ROS_DOMAIN_ID=31   # Pinky
# export ROS_DOMAIN_ID=32   # Inky
# export ROS_DOMAIN_ID=33   # Clyde
```

**To make it permanent (add to `~/.bashrc`):**

```bash
# Example: always use Blinky
echo "source ~/turtlebot3_ws/scripts/set_robot_env.sh blinky" >> ~/.bashrc
source ~/.bashrc
```

**Verify it's set:**

```bash
echo $ROS_DOMAIN_ID
echo $ROBOT_SSH   # if you used the script
```

**Important:**

- If `ROS_DOMAIN_ID` is not set, ROS 2 defaults to 0
- The Remote PC and robot must use the **same** `ROS_DOMAIN_ID` value
- When switching between robots, run `source scripts/set_robot_env.sh <robot> [ip]` again in each terminal (or open new terminals and source once)

---

## Connecting to a Robot

This section describes the steps to connect to a TurtleBot3 robot and start autonomous exploration. You can connect to **Blinky**, **Pinky**, **Inky**, or **Clyde**—use the [robot table](#ros-domain-configuration) and `scripts/set_robot_env.sh` so `ROS_DOMAIN_ID` and `ROBOT_SSH` match the robot you want.

**Prerequisites:**

- Robot is powered on and connected to the network
- Remote PC has ROS 2 Humble installed
- Workspace is built (see [Building the Workspace](#building-the-workspace))
- Robot environment is set (see [ROS Domain Configuration](#ros-domain-configuration)): `source scripts/set_robot_env.sh <robot> [ip]`

**Startup order is critical:** Start terminals in sequence and wait between steps for proper initialization.

---

### Terminal 1: SSH Connection and Robot Launch

**Purpose:** Connect to the robot and launch the robot bringup node.

**Commands:**

```bash
# Set environment for the robot you're connecting to (Blinky, Pinky, Inky, or Clyde)
cd ~/turtlebot3_ws
source scripts/set_robot_env.sh blinky
# For Inky/Clyde, pass the IP: source scripts/set_robot_env.sh inky 192.168.50.xxx

# SSH into the robot
ssh $ROBOT_SSH

# After connection, on the robot:
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

**Expected output (if working correctly):**

```text
[INFO] [launch]: All log files can be found below /home/<user>/.ros/log/<date-time>-<robot>-<pid>
[INFO] [launch]: Default logging verbosity is set to INFO
urdf_file_name : turtlebot3_burger.urdf
[INFO] [robot_state_publisher-1]: process started with pid [<pid>]
[INFO] [ld08_driver-2]: process started with pid [<pid>]
[INFO] [turtlebot3_ros-3]: process started with pid [<pid>]
[turtlebot3_ros-3] [INFO] [...] [turtlebot3_node]: Init TurtleBot3 Node Main
[turtlebot3_ros-3] [INFO] [...] [turtlebot3_node]: Init DynamixelSDKWrapper
[robot_state_publisher-1] [INFO] [...] [robot_state_publisher]: got segment base_footprint
[robot_state_publisher-1] [INFO] [...] [robot_state_publisher]: got segment base_link
[robot_state_publisher-1] [INFO] [...] [robot_state_publisher]: got segment base_scan
[robot_state_publisher-1] [INFO] [...] [robot_state_publisher]: got segment caster_back_link
[robot_state_publisher-1] [INFO] [...] [robot_state_publisher]: got segment imu_link
[robot_state_publisher-1] [INFO] [...] [robot_state_publisher]: got segment wheel_left_link
[robot_state_publisher-1] [INFO] [...] [robot_state_publisher]: got segment wheel_right_link
[turtlebot3_ros-3] [INFO] [...] [DynamixelSDKWrapper]: Succeeded to open the port(/dev/ttyACM0)!
[turtlebot3_ros-3] [INFO] [...] [DynamixelSDKWrapper]: Succeeded to change the baudrate!
[turtlebot3_ros-3] [INFO] [...] [turtlebot3_node]: Start Calibration of Gyro
[ld08_driver-2] /dev/ttyUSB0    CP2102 USB to UART Bridge Controller
[ld08_driver-2] /dev/ttyACM0    OpenCR Virtual ComPort in FS Mode
[ld08_driver-2] FOUND LDS-02
[ld08_driver-2] LDS-02 started successfully
[turtlebot3_ros-3] [INFO] [...] [turtlebot3_node]: Calibration End
[turtlebot3_ros-3] [INFO] [...] [turtlebot3_node]: Add Motors
[turtlebot3_ros-3] [INFO] [...] [turtlebot3_node]: Add Wheels
[turtlebot3_ros-3] [INFO] [...] [turtlebot3_node]: Add Sensors
[turtlebot3_ros-3] [INFO] [...] [turtlebot3_node]: Succeeded to create battery state publisher
[turtlebot3_ros-3] [INFO] [...] [turtlebot3_node]: Succeeded to create imu publisher
[turtlebot3_ros-3] [INFO] [...] [turtlebot3_node]: Succeeded to create sensor state publisher
[turtlebot3_ros-3] [INFO] [...] [turtlebot3_node]: Succeeded to create joint state publisher
[turtlebot3_ros-3] [INFO] [...] [turtlebot3_node]: Add Devices
[turtlebot3_ros-3] [INFO] [...] [turtlebot3_node]: Succeeded to create motor power server
[turtlebot3_ros-3] [INFO] [...] [turtlebot3_node]: Succeeded to create reset server
[turtlebot3_ros-3] [INFO] [...] [turtlebot3_node]: Succeeded to create sound server
[turtlebot3_ros-3] [INFO] [...] [turtlebot3_node]: Run!
[turtlebot3_ros-3] [INFO] [...] [diff_drive_controller]: Init Odometry
[turtlebot3_ros-3] [INFO] [...] [diff_drive_controller]: Run!
```

**What to look for:**

- No error messages about device connections
- Messages indicating successful initialization
- Topics should be publishing: `/scan`, `/odom`, `/joint_states`
- Robot should respond to velocity commands

**Verification:**

```bash
# In a new terminal on Remote PC (source set_robot_env.sh for your robot first)
ros2 topic list | grep -E "(scan|odom|joint_states)"
ros2 topic echo /scan --once  # Should show laser scan data
```

---

### Terminal 2: SLAM Toolbox

**Purpose:** Creates the map as the robot explores using SLAM (Simultaneous Localization and Mapping).

**Commands:**

```bash
# Set environment for your robot (if not already set in this terminal)
cd ~/turtlebot3_ws
source scripts/set_robot_env.sh blinky   # or pinky, inky <IP>, clyde <IP>

# Launch SLAM Toolbox with laser scan normalizer (recommended)
# This automatically handles variable laser scan readings and uses fast map updates
./scripts/start_slam_with_normalizer.sh
```

**What this script does:**

- Starts a laser scan normalizer that fixes variable reading counts (216-230 readings → 228 readings)
- Launches SLAM Toolbox with fast map update configuration (0.2s intervals by default)
- Automatically remaps scan topic to use normalized scans
- Prevents "LaserRangeScan contains X range readings, expected Y" errors
- Defaults to `use_sim_time:=False` (real robot). To use sim time, run:

```bash
USE_SIM_TIME=1 ./scripts/start_slam_with_normalizer.sh
```

**Alternative (manual setup - not recommended):**

If you need to run SLAM Toolbox without the normalizer (not recommended due to scan reading issues):

```bash
# Set environment for your robot
cd ~/turtlebot3_ws
source scripts/set_robot_env.sh blinky   # or your robot
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger

# Launch SLAM Toolbox with fast config
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$(pwd)/src/turtlebot3/turtlebot3_navigation2/param/humble/mapper_params_online_async_fast.yaml
```

**Note:** Without the normalizer, you may see "LaserRangeScan contains X range readings, expected Y" errors, which will prevent the map from updating properly. The `./scripts/start_slam_with_normalizer.sh` script is the recommended approach.

**Expected output (if working correctly):**

```text
==========================================
Starting SLAM with Laser Scan Normalizer
==========================================

This will:
  1. Start laser scan normalizer (fixes variable reading counts)
  2. Start SLAM Toolbox with fast config (0.5s map updates)

Press Ctrl+C to stop both processes

Starting laser scan normalizer...
[INFO] [...] [laser_scan_normalizer]: Laser scan normalizer started: /scan -> /scan_normalized (normalizing to 228 readings)
[INFO] [...] [laser_scan_normalizer]: Scan <N>: received <N_readings> readings, normalizing to 228
[INFO] [...] [laser_scan_normalizer]: Scan <N>: published 228 readings (target: 228)
... (repeats for each incoming scan; received counts may vary) ...
Starting SLAM Toolbox with fast config...
Using normalized scan topic: /scan_normalized

use_sim_time: False
[INFO] [launch]: All log files can be found below /home/<user>/.ros/log/<date-time>-central-<pid>
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [async_slam_toolbox_node-1]: process started with pid [<pid>]
[async_slam_toolbox_node-1] [INFO] [...] [slam_toolbox]: Node using stack size 40000000
[async_slam_toolbox_node-1] [INFO] [...] [slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
[async_slam_toolbox_node-1] [INFO] [...] [slam_toolbox]: CeresSolver: Using SCHUR_JACOBI preconditioner.
```

**What to look for:**

- No error messages about missing topics or nodes
- Messages indicating SLAM Toolbox has started
- After 20-30 seconds, the `/map` topic should appear and start publishing

**Important:** Wait 20-30 seconds after starting SLAM Toolbox before proceeding to Terminal 3. SLAM needs time to:

- Receive scan data from the robot
- Process several scan messages
- Build the initial map
- Start publishing the `/map` topic

**Verification:**

```bash
# Wait 20-30 seconds, then check:
ros2 topic list | grep "^/map$"  # Should show /map topic
ros2 topic echo /map --once      # Should show map data (may need to wait a few more seconds)
```

---

### Terminal 3: Navigation2

**Purpose:** Provides navigation and path planning capabilities (obstacle avoidance + goal execution) while using SLAM Toolbox’s live `/map`.

**Commands:**

```bash
# Set environment for your robot (if not already set)
cd ~/turtlebot3_ws
source scripts/set_robot_env.sh blinky   # or pinky, inky <IP>, clyde <IP>
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger

# Launch Navigation2 for SLAM / exploration (includes RViz)
# - Does NOT load a static map file
# - Uses SLAM's live map (/map)
# - Waits for TF (map->odom and odom->base_*) before starting Nav2
ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py use_sim_time:=False
```

**Expected output (if working correctly):**

```text
[INFO] [launch]: All log files can be found below /home/<user>/.ros/log/<date-time>-central-<pid>
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [python3-1]: process started with pid [<pid>]
[INFO] [component_container_isolated-2]: process started with pid [<pid>]
[INFO] [map_saver_server-3]: process started with pid [<pid>]
[INFO] [lifecycle_manager-4]: process started with pid [<pid>]
[INFO] [sync_slam_toolbox_node-5]: process started with pid [<pid>]
[INFO] [rviz2-6]: process started with pid [<pid>]
[rviz2-6] Warning: Ignoring XDG_SESSION_TYPE=wayland on Gnome. Use QT_QPA_PLATFORM=wayland to run on Wayland anyway.
[sync_slam_toolbox_node-5] [INFO] [...] [slam_toolbox]: Node using stack size 40000000
[lifecycle_manager-4] [INFO] [...] [lifecycle_manager_slam]: Starting managed nodes bringup...
[map_saver_server-3] [INFO] [...] [map_saver]: Creating
[map_saver_server-3] [INFO] [...] [map_saver]: Configuring
[lifecycle_manager-4] [INFO] [...] [lifecycle_manager_slam]: Managed nodes are active
[sync_slam_toolbox_node-5] [INFO] [...] [slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
[python3-1] [INFO] [...] [wait_for_tf]: Waiting for TF. Need map->odom and odom->(one of ['base_footprint', 'base_link']). Timeout: 30.0s
[python3-1] [INFO] [...] [wait_for_tf]: TF ready: odom -> base_footprint
[python3-1] [INFO] [...] [wait_for_tf]: TF tree looks ready.
[INFO] [python3-1]: process has finished cleanly [pid <pid>]
[component_container_isolated-2] [INFO] [...] [lifecycle_manager_navigation]: Managed nodes are active
[component_container_isolated-2] [INFO] [...] [lifecycle_manager_navigation]: Creating bond timer...

# Optional (may appear depending on RViz / drivers / installed plugins):
[rviz2-6] [ERROR] [...] [rviz2]: PluginlibFactory: The plugin for class 'nav2_rviz_plugins/Selector' failed to load. (plugin not installed)
[rviz2-6] [ERROR] [...] [rviz2]: PluginlibFactory: The plugin for class 'nav2_rviz_plugins/Docking' failed to load. (plugin not installed)
[rviz2-6] [ERROR] [...] [rviz2]: ... GLSL link result: active samplers with a different type refer to the same texture image unit
```

**What to look for:**

- Nav2 nodes starting successfully
- RViz window should open automatically
- After 20-30 seconds, costmap topics should be available

**Important:**

- Wait 20-30 seconds after starting Nav2 before proceeding to Terminal 4
- For SLAM/exploration you generally do **not** set an AMCL initial pose (Nav2 is using SLAM localization).
- If you see Nav2 waiting on TF (e.g. `base_* frame does not exist`), make sure robot bringup is running and TF is publishing.

**Verification:**

```bash
# Check Nav2 nodes
ros2 node list | grep nav2

# Check costmap topics
ros2 topic list | grep costmap

# Check TF chain needed for navigation
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

---

### Terminal 4: Explorer

**Purpose:** Detects frontiers (unexplored areas) and sends exploration goals to Nav2 for autonomous exploration.

**Commands:**

```bash
# Set environment for your robot (if not already set)
cd ~/turtlebot3_ws
source scripts/set_robot_env.sh blinky   # or pinky, inky <IP>, clyde <IP>

# Start explorer
./scripts/start_explorer_simple.sh
```

**Expected output (if working correctly):**

```text
Starting explorer with SLAM map (direct from slam_toolbox)...
Explorer will wait for /map topic to become available.

Note: Using SLAM map directly instead of costmap for better frontier detection
      when the map is still small or narrow.

[INFO] [...] [explore_node]: Waiting for costmap to become available, topic: map
[INFO] [...] [explore_node]: Received full costmap update: <W>x<H> cells, resolution=<res>, origin=(<ox>, <oy>)
Warning: TF_OLD_DATA ignoring data from the past for frame odom at time <t> according to authority Authority undetectable
Possible reasons are listed at http://wiki.ros.org/tf/Errors%20explained
         at line <N> in ./src/buffer_core.cpp
[INFO] [...] [explore_node]: Waiting to connect to move_base nav2 server
[INFO] [...] [explore_node]: Connected to move_base nav2 server
[INFO] [...] [explore_node]: Exploration timer started with frequency <Hz> Hz
[INFO] [...] [explore_node]: Costmap stats - Unknown: <N> (<P>%), Free: <N> (<P>%), Occupied: <N> (<P>%), Total: <N>
[INFO] [...] [explore_node]: Robot at costmap cell (<cx>, <cy>), value: <v> (0=free, 255=unknown, 254=lethal), pose: (<x>, <y>)
[INFO] [...] [explore_node]: Nearby cells (radius <r>): <N> unknown, <N> free
[INFO] [...] [explore_node]: Found <N> frontiers
[INFO] [...] [explore_node]:   Frontier 0: cost=<c>, distance=<d>, size=<s>
[INFO] [...] [explore_node]: After distance filtering: <N> valid frontiers (from <N> total)
[INFO] [...] [explore_node]: Selected frontier at (<x>, <y>), cost=<c>, distance=<d>
[INFO] [...] [explore_node]: Sending goal to move base nav2: (<x>, <y>)
[INFO] [...] [explore_node]: Goal accepted by Nav2, navigating to (<x>, <y>)
```

**What to look for:**

- Explorer waits for Nav2's costmap (this is normal - takes 20-40 seconds after Nav2 starts)
- You'll see it connect to the Nav2 action server (“Connected to move_base nav2 server”)
- Once you see “Goal accepted by Nav2…”, the explorer is working and the robot should start moving autonomously

**Important:**

- Start this **after** Nav2 is running and initialized (wait 20-30 seconds after starting Nav2)
- The explorer automatically waits for the costmap - be patient
- Total startup time from robot launch to exploration: ~60-90 seconds

**Verification:**

```bash
# Check explorer node
ros2 node list | grep explore

# Check goals being sent
ros2 topic echo /goal_pose  # Should see goals being published

# Robot should be moving autonomously
```

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

- **Step 3**: Set the same value everywhere. Easiest: use the setup script for your robot (see [ROS Domain Configuration](#ros-domain-configuration)).

  ```bash
  cd ~/turtlebot3_ws
  source scripts/set_robot_env.sh blinky   # or pinky, inky <IP>, clyde <IP>
  ```

  Or set manually: `export ROS_DOMAIN_ID=30` (Blinky), 31 (Pinky), 32 (Inky), 33 (Clyde).

- **Step 4 (optional)**: Make it persistent.

  ```bash
  echo "source ~/turtlebot3_ws/scripts/set_robot_env.sh blinky" >> ~/.bashrc
  source ~/.bashrc
  ```

- **Step 5**: Restart terminals (or `source ~/.bashrc`) so every process uses the same domain.

**Note:** If `ROS_DOMAIN_ID` is not set, ROS 2 defaults to 0. Make sure both robot and Remote PC explicitly set the same value!

---

#### 3. TF errors: `base_link` / `base_footprint` / `odom` frame does not exist

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

#### 4. AMCL warning: “Please set the initial pose…” (wrong launch file)

**When this happens:** You launched the non-SLAM Nav2 bringup (AMCL/static-map workflow) while expecting SLAM-based exploration.

**Fix (SLAM/exploration):**

```bash
ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py use_sim_time:=False
```

**Fix (static map + AMCL):**

- Use the non-SLAM bringup (e.g. `navigation2.launch.py`) and then set the initial pose in RViz.

---

#### 5. RViz errors about Nav2 panels / GLSL

**Symptoms:**

- `nav2_rviz_plugins/Selector` or `nav2_rviz_plugins/Docking` failed to load
- GLSL error: `active samplers with a different type refer to the same texture image unit`

**Cause:** RViz plugin / GPU driver quirks. These do not usually prevent navigation.

**Workarounds:**

- If the map still renders and Nav2 works, you can ignore these.
- If RViz rendering is broken, try software rendering:

```bash
LIBGL_ALWAYS_SOFTWARE=1 rviz2
```

---

#### 6. (Optional) Manually set initial pose (static map + AMCL only)

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
```

Adjust x, y, z, w values to match robot's actual position.

---

#### 7. Costmap warning: “Sensor origin is out of map bounds”

**Cause:** Nav2 doesn't know where the robot is on the map yet, so it can't determine if the sensor is within map bounds.

**Symptoms:**

```text
[WARN] [global_costmap.global_costmap]: Sensor origin at (-0.03, -0.00) is out of map bounds (0.00, 0.00) to (4.98, 4.98)
```

**Fix:**

- Ensure SLAM is publishing a map:
  - `ros2 topic echo /map --once`
- Ensure TF is valid:
  - `ros2 run tf2_ros tf2_echo map odom`
  - `ros2 run tf2_ros tf2_echo odom base_footprint`
- If you are using a static-map + AMCL workflow, set the initial pose in RViz (or use the manual initial pose command above).

**Note:** This warning is normal and expected until you set the initial pose. It doesn't prevent Nav2 from working, but you should set the initial pose to resolve it.

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
  - `ros2 topic echo /goal_pose`
- Check Nav2 is up:
  - `ros2 node list | grep nav2`
- Check TF is valid:
  - `ros2 run tf2_ros tf2_echo map odom`
  - `ros2 run tf2_ros tf2_echo odom base_footprint`
- Ensure the map has some free space and unknown space (explorer needs frontiers).

---

#### 11. "Starting point in lethal space" / "Collision Ahead - Exiting Spin" (robot stuck near walls/corners)

**Cause:** The planner thinks the robot is inside an obstacle (often due to costmap inflation when the robot is close to a wall or corner). Recovery (spin) then sees inflated obstacles and aborts.

**Symptoms:**

- `GridBased: failed to create plan, invalid use: Starting point in lethal space!`
- `spin failed` / `Collision Ahead - Exiting Spin`
- Robot gets close to a corner or doorframe and then stays there, not moving.

**Fix:** The Nav2 params in `turtlebot3_navigation2/param/humble/burger.yaml` are already tuned to reduce this: global costmap uses smaller inflation (0.40 m radius, cost_scaling_factor 4.0) and local costmap inflation is 0.55 m. If it still happens:

- Drive the robot slightly away from the wall/corner so its center is in clearly free space.
- Optionally reduce `inflation_radius` further in the global costmap (e.g. to 0.35) in the same param file, then restart Nav2.

---

#### 12. Odom TF jumping away from map TF / map at a weird angle / straight walls look curved

**Cause:** The `map`→`odom` transform is published by SLAM Toolbox. When it corrects for odometry drift, that correction can appear as a “jump” if updates are infrequent or large. Curved walls usually mean rotational odometry drift during mapping (robot thinks it’s going straight but odom says it’s turning).

**Symptoms:**

- In RViz, the robot or map seems to jump; odom frame moves away from map then snaps back.
- Jumps happen more often the farther the robot is from the start.
- Map looks rotated or straight corridors/walls appear curved.

**What we’ve done:** SLAM params in `mapper_params_online_async_fast.yaml` are tuned for smoother behavior:

- `map_update_interval: 0.1` (was 0.2) — more frequent scan matching so corrections are smaller.
- `minimum_travel_distance` / `minimum_travel_heading: 0.08` (were 0.15) — match more often so pose updates are smaller.
- `transform_timeout: 0.1` — keeps map→odom timestamp closer to current time.

**If it still happens:**

- Ensure only **one** node publishes `map`→`odom` (SLAM Toolbox when using SLAM; do not run AMCL at the same time). The Nav2 panel showing “Localization: inactive” is normal when using SLAM.
- Check odometry: wheel slip, uneven floors, or miscalibrated wheel radius/separation (TurtleBot3: `turtlebot3_node/param/burger.yaml` — `wheels.separation`, `wheels.radius`) can cause drift and curved maps.
- If the robot has an IMU, ensure `use_imu: true` in the diff_drive/odometry config so orientation drift is reduced.

---

#### 13. Two maps showing in RViz (static map + SLAM map)

**Cause:** Nav2 is loading a default static map file, and SLAM Toolbox is also publishing its live map. Both appear in RViz.

**Fix:**

- Hide the static map display in RViz (keep the live `/map` display), or
- Use the SLAM Nav2 launch (`navigation2_slam.launch.py`) so Nav2 does not load a static map.

**Note:** When doing SLAM/exploration, you typically want to use the live map from SLAM Toolbox, not a static map file.

---

#### 14. Odometry not publishing (`/odom` exists but no data)

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
ros2 topic echo /goal_pose

# Check explorer topics
ros2 topic list | grep explore
```

### Check ROS Domain ID

```bash
# On Remote PC
echo $ROS_DOMAIN_ID

# On Robot (via SSH)
echo $ROS_DOMAIN_ID

# Both should show the same value (30, 31, 32, or 33)
```

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
