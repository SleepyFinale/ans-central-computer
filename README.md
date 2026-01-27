# TurtleBot3 Workspace - Autonomous Exploration Setup

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

The workspace includes two build scripts to help you build and source everything:

#### `clean_rebuild.sh`

Performs a complete clean rebuild of the entire workspace:

- Removes all build artifacts (`build/`, `install/`, `log/` directories)
- Checks for system dependencies
- Builds all packages from scratch
- Sources the workspace automatically after build

**Usage:**

```bash
cd ~/turtlebot3_ws
./clean_rebuild.sh
```

**When to use:**

- First-time setup
- After major changes to multiple packages
- When experiencing build issues that require a clean slate
- After pulling significant changes from the repository

#### `minimal_rebuild.sh`

Performs a minimal rebuild of only essential packages:

- Removes build artifacts
- Builds only packages needed for:
  - `turtlebot3_bringup robot.launch.py`
  - `turtlebot3_navigation2 navigation2.launch.py`
  - `explore_lite` (for the explorer)

**Usage:**

```bash
cd ~/turtlebot3_ws
./minimal_rebuild.sh
```

**When to use:**

- After making small changes to specific packages
- Faster rebuild times during development
- When you only need to update navigation or exploration components

**Note:** Both scripts automatically source the workspace after building. If you need to manually source the workspace:

```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## ROS Domain Configuration

ROS 2 uses `ROS_DOMAIN_ID` to separate different robot networks. Each robot and the Remote PC must use the **same** `ROS_DOMAIN_ID` value to communicate.

**Robot Domain IDs:**

- **Blinky**: `ROS_DOMAIN_ID=30`
- **Pinky**: `ROS_DOMAIN_ID=31`
- **Inky**: `ROS_DOMAIN_ID=32`
- **Clyde**: `ROS_DOMAIN_ID=33`

**Setting ROS_DOMAIN_ID:**

**On Remote PC (for each robot connection):**

```bash
# For Blinky
export ROS_DOMAIN_ID=30

# For Pinky
export ROS_DOMAIN_ID=31

# For Inky
export ROS_DOMAIN_ID=32

# For Clyde
export ROS_DOMAIN_ID=33
```

**To make it permanent (add to `~/.bashrc`):**

```bash
# For Blinky (example)
echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
source ~/.bashrc
```

**Verify it's set:**

```bash
echo $ROS_DOMAIN_ID
```

**Important:**

- If `ROS_DOMAIN_ID` is not set, ROS 2 defaults to 0
- The Remote PC and robot must use the **same** `ROS_DOMAIN_ID` value
- When switching between robots, make sure to update `ROS_DOMAIN_ID` accordingly

---

## Connecting to a Robot

This section describes the steps to connect to a TurtleBot3 robot and start autonomous exploration. The example uses **Blinky** (ROS_DOMAIN_ID=30), but the same process applies to other robots with their respective domain IDs.

**Prerequisites:**

- Robot is powered on and connected to the network
- Remote PC has ROS 2 Humble installed
- Workspace is built (see [Building the Workspace](#building-the-workspace))
- `ROS_DOMAIN_ID` is set correctly (see [ROS Domain Configuration](#ros-domain-configuration))

**Startup order is critical:** Start terminals in sequence and wait between steps for proper initialization.

---

### Terminal 1: SSH Connection and Robot Launch

**Purpose:** Connect to the robot and launch the robot bringup node.

**Commands:**

```bash
# Set ROS_DOMAIN_ID for Blinky
export ROS_DOMAIN_ID=30

# SSH into the robot
ssh ubuntu@172.20.10.8
# Password: ubuntu

# After connection, on the robot:
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

**Expected output (if working correctly):**

```text
[INFO] [turtlebot3_node]: TurtleBot3 node has been initialized
[INFO] [turtlebot3_node]: Subscribed to /cmd_vel
[INFO] [turtlebot3_node]: Publishing to /odom, /joint_states
[INFO] [turtlebot3_node]: Publishing to /scan
```

**What to look for:**

- No error messages about device connections
- Messages indicating successful initialization
- Topics should be publishing: `/scan`, `/odom`, `/joint_states`
- Robot should respond to velocity commands

**Verification:**

```bash
# In a new terminal on Remote PC (with ROS_DOMAIN_ID=30 set)
ros2 topic list | grep -E "(scan|odom|joint_states)"
ros2 topic echo /scan --once  # Should show laser scan data
```

---

### Terminal 2: SLAM Toolbox

**Purpose:** Creates the map as the robot explores using SLAM (Simultaneous Localization and Mapping).

**Commands:**

```bash
# Set ROS_DOMAIN_ID for Blinky
export ROS_DOMAIN_ID=30

# Navigate to workspace
cd ~/turtlebot3_ws

# Launch SLAM Toolbox with laser scan normalizer (recommended)
# This automatically handles variable laser scan readings and uses fast map updates
./start_slam_with_normalizer.sh
```

**What this script does:**

- Starts a laser scan normalizer that fixes variable reading counts (216-230 readings → 228 readings)
- Launches SLAM Toolbox with fast map update configuration (0.2s intervals by default)
- Automatically remaps scan topic to use normalized scans
- Prevents "LaserRangeScan contains X range readings, expected Y" errors
- Defaults to `use_sim_time:=False` (real robot). To use sim time, run:

```bash
USE_SIM_TIME=1 ./start_slam_with_normalizer.sh
```

**Alternative (manual setup - not recommended):**

If you need to run SLAM Toolbox without the normalizer (not recommended due to scan reading issues):

```bash
# Set ROS_DOMAIN_ID for Blinky
export ROS_DOMAIN_ID=30

# Setup ROS 2 environment
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger

# Launch SLAM Toolbox with fast config
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$(pwd)/src/turtlebot3/turtlebot3_navigation2/param/humble/mapper_params_online_async_fast.yaml
```

**Note:** Without the normalizer, you may see "LaserRangeScan contains X range readings, expected Y" errors, which will prevent the map from updating properly. The `./start_slam_with_normalizer.sh` script is the recommended approach.

**Expected output (if working correctly):**

```text
[INFO] [slam_toolbox]: Node using stack size 40000000
[INFO] [slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
[INFO] [slam_toolbox]: Registering sensor: [Custom Described Lidar]
[INFO] [slam_toolbox]: Slam Toolbox started
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
# Set ROS_DOMAIN_ID for Blinky
export ROS_DOMAIN_ID=30

# Navigate to workspace
cd ~/turtlebot3_ws

# Setup ROS 2 environment
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
[INFO] [nav2_controller]: Creating controller server
[INFO] [nav2_planner]: Creating planner server
[INFO] [nav2_recoveries]: Creating recovery server
[INFO] [nav2_bt_navigator]: Creating BT navigator
[INFO] [nav2_lifecycle_manager]: Creating lifecycle manager
[INFO] [rviz2]: RViz2 started
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
# Set ROS_DOMAIN_ID for Blinky
export ROS_DOMAIN_ID=30

# Navigate to workspace
cd ~/turtlebot3_ws

# Start explorer
./start_explorer_simple.sh
```

**Expected output (if working correctly):**

```text
[INFO] [explore_node]: Waiting for costmap to become available, topic: /global_costmap/costmap
[INFO] [explore_node]: Costmap available, starting exploration
[INFO] [explore_node]: Exploration started
[INFO] [explore_node]: Found frontier at (x, y)
[INFO] [explore_node]: Sending goal to Nav2
```

**What to look for:**

- Explorer waits for Nav2's costmap (this is normal - takes 20-40 seconds after Nav2 starts)
- Once costmap is ready, you'll see "Exploration started"
- Explorer will begin finding frontiers and sending goals
- Robot should start moving autonomously

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

#### 0. Build Error: "Could not find a package configuration file provided by 'nav2_msgs'"

**Symptoms:**

```text
CMake Error at CMakeLists.txt:40 (find_package):
  By not providing "Findnav2_msgs.cmake" in CMAKE_MODULE_PATH this project
  has asked CMake to find a package configuration file provided by
  "nav2_msgs", but CMake did not find one.
```

**Cause:** Navigation2 system packages are not installed. The workspace is configured to use system Navigation2 packages (installed via apt) rather than building them from source.

**Solution:**

Install Navigation2 packages:

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
./clean_rebuild.sh
```

**Note:** The build scripts now automatically check for Navigation2 packages and will provide a helpful error message if they're missing.

---

#### 1. ROS_DOMAIN_ID Mismatch (Topics Not Visible)

**Symptoms:**

- Topics from robot not visible on Remote PC (or vice versa)
- `ros2 topic list` shows different topics on robot vs Remote PC
- Nodes can't see each other

**Solution:**

1. Check `ROS_DOMAIN_ID` on robot:

   ```bash
   echo $ROS_DOMAIN_ID
   ```

2. Check `ROS_DOMAIN_ID` on Remote PC:

   ```bash
   echo $ROS_DOMAIN_ID
   ```

3. Set the same value on both (e.g., 30 for Blinky):

   ```bash
   export ROS_DOMAIN_ID=30
   ```

4. Make it permanent:

   ```bash
   echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
   source ~/.bashrc
   ```

5. Restart terminals or source bashrc on both systems

**Note:** If `ROS_DOMAIN_ID` is not set, ROS 2 defaults to 0. Make sure both robot and Remote PC explicitly set the same value!

---

#### 2. AMCL Warning: "AMCL cannot publish a pose or update the transform. Please set the initial pose..."

**Note (SLAM/exploration workflow):** If you are running `navigation2_slam.launch.py`, Nav2 uses SLAM localization and you normally will **not** use AMCL initial pose. If you see this warning, you likely launched the non-SLAM Nav2 launch by mistake.

**Solution:**

Use the SLAM Nav2 launch instead:

```bash
ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py use_sim_time:=False
```

---

#### 2. TF Errors: `base_link` / `base_footprint` frame does not exist

**Symptoms:**

```text
Timed out waiting for transform from base_footprint to odom to become available, tf error:
Invalid frame ID "base_footprint" ... frame does not exist
```

**Cause:** Nav2 is starting before it has received the robot TF (`odom -> base_*`), or the Remote PC is not receiving TF from the robot (often a `ROS_DOMAIN_ID` mismatch).

**Solution:**

1. Verify your domain ID matches across all terminals:

```bash
echo $ROS_DOMAIN_ID
```

1. Verify the Remote PC can see robot TF:

```bash
ros2 topic echo /tf --once
```

You should see at least `map -> odom` (from SLAM) and `odom -> base_*` (from robot bringup / odometry / robot_state_publisher).

1. Verify the specific transforms:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

**Note:** `navigation2_slam.launch.py` includes a TF wait step (`wait_for_tf.py`) to reduce this startup race. If TF never appears, the issue is upstream (robot bringup or networking / DDS).

---

#### 3. RViz errors about Nav2 panels / GLSL

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

#### 4. (Optional) Manually setting initial pose (only for static-map AMCL workflows)

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
```

Adjust x, y, z, w values to match robot's actual position.

---

#### 3. Costmap Warnings: "Sensor origin is out of map bounds"

**Cause:** Nav2 doesn't know where the robot is on the map yet, so it can't determine if the sensor is within map bounds.

**Symptoms:**

```text
[WARN] [global_costmap.global_costmap]: Sensor origin at (-0.03, -0.00) is out of map bounds (0.00, 0.00) to (4.98, 4.98)
```

**Solution:**

1. Set the initial pose in RViz (see issue #2 above)
2. Once AMCL knows where the robot is, the warnings will stop
3. Make sure SLAM has published the map first: `ros2 topic echo /map --once`

**Note:** This warning is normal and expected until you set the initial pose. It doesn't prevent Nav2 from working, but you should set the initial pose to resolve it.

---

#### 4. No Map Appearing in SLAM (`/map` topic doesn't exist or not publishing)

**Cause:** SLAM Toolbox not receiving scan data, not initialized yet, or needs more time.

**Symptoms:**

- `/map` topic doesn't appear in `ros2 topic list`
- `/map` topic exists but `ros2 topic echo /map --once` shows "does not appear to be published yet"

**Solution:**

1. Check scan data is available:

   ```bash
   ros2 topic echo /scan --once
   ```

   Should show laser scan data. If not, check robot connection.

2. Check scan frequency:

   ```bash
   ros2 topic hz /scan
   ```

   Should show ~10 Hz (depends on lidar). If 0 Hz, scan isn't publishing.

3. Move robot slightly to help SLAM initialize:

   ```bash
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'
   ```

4. Wait longer: SLAM needs 20-30 seconds AND several scan messages before publishing map
   - The `/map` topic won't appear until SLAM has processed enough scans
   - This is normal - be patient!

5. Verify SLAM is running:

   ```bash
   ros2 node list | grep slam
   ```

   Should show `async_slam_toolbox_node`

6. Check SLAM logs for errors in the terminal where you launched it

**Note:** It's normal for `/map` to not appear immediately. SLAM Toolbox needs to receive scan data, process several scans, build initial map, then start publishing `/map` topic. This typically takes 20-40 seconds from when SLAM starts.

---

#### 5. Explorer Waiting for Costmap

**Cause:** Nav2 costmap hasn't initialized yet (normal - takes 20-40 seconds).

**Symptoms:**

```text
[INFO] [explore_node]: Waiting for costmap to become available, topic: /global_costmap/costmap
```

**Solution:**

- **This is normal!** The explorer is designed to wait. Just be patient.
- The explorer will automatically connect once Nav2's costmap is ready.
- You'll see: `[INFO] [explore_node]: Exploration started` when ready.

**Total wait time:** Usually 20-40 seconds after Nav2 starts, but can take up to 60-90 seconds total from robot launch.

---

#### 6. TF Errors: "Invalid frame ID 'odom' passed to canTransform"

**Cause:** Nav2 started before SLAM Toolbox initialized, or odometry isn't publishing, or initial pose not set.

**Solution:**

1. Set initial pose first (see issue #2 above)
2. Stop Nav2 (Ctrl+C)
3. Verify SLAM Toolbox is running: `ros2 node list | grep slam`
4. Wait 20-30 seconds for SLAM to initialize
5. Check TF tree: `ros2 run tf2_ros tf2_echo map odom` (should work)
6. Restart Nav2 and set initial pose again

**Prevention:** Always wait 20-30 seconds after starting SLAM before starting Nav2, and set initial pose after Nav2 starts.

---

#### 7. Robot Not Moving / Explorer Not Finding Frontiers

**Cause:** System still initializing, or map too small.

**Solution:**

1. Wait 60-90 seconds total from startup
2. Check explorer status: Should see `[INFO] [explore_node]: Exploration started`
3. Check goals: `ros2 topic echo /goal_pose` (should see goals being sent)
4. Check Nav2: `ros2 service list | grep lifecycle` (should see Nav2 services)
5. Wait for map to build - explorer needs some map data before finding frontiers
6. Verify initial pose is set in RViz

---

#### 8. Two Maps Showing in RViz (Default Test Map + SLAM Map)

**Cause:** Nav2 is loading a default static map file, and SLAM Toolbox is also publishing its live map. Both appear in RViz.

**Solution:**

1. **Option A: Hide static map in RViz:**
   - In RViz, find the "Map" display (there may be two)
   - Disable/hide the one showing the default test map
   - Keep the one showing the live SLAM map

2. **Option B: Use only SLAM map:**
   - In RViz, remove the Map display that's showing the static map
   - Keep only the Map display subscribed to `/map` (from SLAM Toolbox)

**Note:** When doing SLAM/exploration, you typically want to use the live map from SLAM Toolbox, not a static map file.

---

#### 9. Odometry Not Publishing (`/odom` topic exists but no data)

**Cause:** Odometry needs robot movement to initialize, or parameters not loaded.

**Solution:**

1. Move robot slightly:

   ```bash
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'
   ```

2. Check odometry: `ros2 topic echo /odom --once`

3. If still not working, restart robot launch on the robot

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
