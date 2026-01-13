# TurtleBot3 Workspace - Autonomous Exploration Setup

This workspace contains editable TurtleBot3 packages for ROS 2 Humble, configured for autonomous exploration and mapping with Nav2, SLAM Toolbox, and Explore Lite.

## Prerequisites

Before proceeding, ensure you have the following installed:

```bash
sudo apt update
sudo apt install git
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
```

## Setup Instructions

### 1. Clone the TurtleBot3 Packages

Navigate to the src directory and clone the required packages:

```bash
cd ~/turtlebot3_ws/src
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
```

### 2. Remove Git Directories from Cloned Packages

To make these packages part of your single repository (instead of submodules), remove their .git directories:

```bash
cd ~/turtlebot3_ws/src
rm -rf DynamixelSDK/.git
rm -rf turtlebot3_msgs/.git
rm -rf turtlebot3/.git
```

### 3. Build the Workspace

Install colcon build tools if needed:
```bash
sudo apt install python3-colcon-common-extensions
```

Build the workspace:
```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 4. Build Explore Lite

If you want to use Explore Lite for autonomous exploration, build it:

```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select explore_lite --packages-ignore nav2_msgs nav2_voxel_grid nav2_util nav2_lifecycle_manager nav2_map_server nav2_costmap_2d nav2_ros_common navigation2
source install/setup.bash
```

Or use the provided build script:
```bash
./BUILD_EXPLORE_LITE.sh
```

---

# Autonomous Exploration Guide

**After your robot is running** (`ros2 launch turtlebot3_bringup robot.launch.py` on the robot), run these commands on your **central computer** in **separate terminals**:

## Setup (Run once per terminal session)

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
```

---

## Terminal 1: SLAM Toolbox (Mapping)

**Purpose**: Creates the map as the robot explores

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch slam_toolbox online_async_launch.py
```

**Alternative**: If you prefer Cartographer (your old setup):
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py configuration_basename:=turtlebot3_lds_2d_improved.lua
```

---

## Terminal 2: Nav2 (Navigation)

**Purpose**: Provides navigation and path planning capabilities

**Option A: TurtleBot3 Nav2 (Recommended - includes RViz)**
```bash
# First install nav2_bringup if not already installed:
sudo apt install ros-humble-nav2-bringup

# Then launch:
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py
```
*Note: This launches both Nav2 and RViz. If you use this, you can skip Terminal 4.*

**Option B: Standard Nav2 (if nav2_bringup is installed)**
```bash
sudo apt install ros-humble-nav2-bringup  # Install first if needed
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch nav2_bringup navigation_launch.py
```

---

## Terminal 3: Explorer (Autonomous Exploration)

**Purpose**: Decides where to explore next and sends goals to Nav2

**Option A: Explore Lite (Recommended - more robust)**
```bash
cd ~/turtlebot3_ws
./start_explorer_simple.sh
```

**Option B: Your Custom Explorer**
```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run custom_explorer explorer
```

---

## Terminal 4: RViz (Visualization - Optional but Recommended)

**Purpose**: Visualize the map, robot position, and exploration progress

**Option A: Using TurtleBot3 Navigation2 (Recommended)**
```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py
```
*Note: This launches both Nav2 and RViz together*

**Option B: Launch RViz directly**
```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 run rviz2 rviz2
```
*Then manually add displays: Map, TF, LaserScan, RobotModel*

**Option C: Install nav2_bringup (if you want the standard Nav2 RViz)**
```bash
sudo apt install ros-humble-nav2-bringup
```
*Then you can use: `ros2 launch nav2_bringup rviz_launch.py`*

---

## Quick Reference: Complete Sequence

**IMPORTANT**: Start in this exact order and wait between steps:

1. **Robot**: `ros2 launch turtlebot3_bringup robot.launch.py` (on robot)
2. **Computer Terminal 1**: `ros2 launch slam_toolbox online_async_launch.py`
   - **Wait 5-10 seconds** for SLAM to initialize (creates `odom` frame)
3. **Computer Terminal 2**: `ros2 launch turtlebot3_navigation2 navigation2.launch.py`
   - **Wait 10-30 seconds** for Nav2 to fully initialize
4. **Computer Terminal 3**: `ros2 run explore_lite explore`
5. **Computer Terminal 4**: RViz (optional - already included in Terminal 2 if using turtlebot3_navigation2)

---

## What Each Terminal Does

| Terminal | Component | Purpose |
|----------|-----------|---------|
| Robot | Robot Launch | Publishes sensor data (`/scan`), odometry, TF |
| Terminal 1 | SLAM Toolbox | Builds the map from sensor data, publishes `/map` |
| Terminal 2 | Nav2 | Plans paths, avoids obstacles, executes navigation |
| Terminal 3 | Explorer | Detects frontiers, sends exploration goals to Nav2 |
| Terminal 4 | RViz | Visualizes everything in real-time |

---

## Verification Checklist

After starting all terminals, check:

1. **Map is being created**: In RViz, you should see the map growing
2. **Robot is moving**: The robot should start exploring autonomously
3. **Topics are active**: `ros2 topic list | grep -E "(map|scan|cmd_vel)"`
4. **TF is working**: `ros2 run tf2_ros tf2_echo map base_link` (should show transform)

---

## Troubleshooting

- **Robot not moving?** 
  - Wait 10-30 seconds after starting Nav2 (it needs time to initialize)
  - Check Nav2 lifecycle: `ros2 service list | grep lifecycle`
  
- **No map appearing?**
  - Check SLAM Toolbox is receiving `/scan`: `ros2 topic echo /scan --once`
  - Make sure robot launch is running and publishing scan data
  
- **Explorer not finding frontiers?**
  - This is normal - wait for the map to build up first
  - The explorer needs some map data before it can find frontiers

- **TF errors? (e.g., "Invalid frame ID 'odom'")**
  - **Most common**: Nav2 started before SLAM Toolbox
  - **Solution**: Stop Nav2, make sure SLAM Toolbox is running, wait 10 seconds, then restart Nav2
  - Check TF tree: `ros2 run tf2_ros tf2_echo map base_link` (should show transforms)
  - Verify frames exist: `ros2 run tf2_ros tf2_monitor` (should see map, odom, base_link)
  - **Launch order matters**: Robot → SLAM → Nav2 → Explorer

---

## Workspace Structure

This workspace includes:
- **TurtleBot3 packages**: Core robot packages from ROBOTIS
- **Navigation2**: Navigation stack (can use system packages or workspace version)
- **SLAM Toolbox**: For mapping (installed via apt)
- **Explore Lite**: Autonomous exploration package (in `src/m-explore-ros2/`)
- **Custom Explorer**: Alternative exploration node (in `src/Autonomous-Explorer-and-Mapper-ros2-nav2/`)

---

## Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/index.html)
- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [TurtleBot3](https://www.turtlebot.com/)
