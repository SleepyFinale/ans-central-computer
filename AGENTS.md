# AGENTS.md

## Cursor Cloud specific instructions

### Overview

This is a ROS 2 workspace for TurtleBot3 Burger autonomous exploration. It builds 14 colcon packages from source (TurtleBot3 core, DynamixelSDK, turtlebot3_msgs, explore_lite, multirobot_map_merge) and relies on system-installed Navigation2 and SLAM Toolbox packages.

### ROS 2 distribution: Jazzy (not Humble)

The Cloud VM runs Ubuntu 24.04 (Noble). ROS 2 Humble packages require Ubuntu 22.04 libraries (Python 3.10, libtinyxml2-9, etc.) and **cannot be installed** on Noble. ROS 2 **Jazzy** is used instead—the workspace code is compatible with both distributions (confirmed by Jazzy Dockerfile in `src/turtlebot3/docker/jazzy/`).

**Consequence:** The shell scripts in `scripts/` hard-code `source /opt/ros/humble/setup.bash`. In the Cloud VM you must source Jazzy manually instead:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
export TURTLEBOT3_MODEL=burger
```

### Compiler: GCC must be the default

The VM's default `c++` points to clang, which cannot find `-lstdc++`. The update script sets `c++` → `g++-13` and `cc` → `gcc-13` via `update-alternatives`. If you see linker errors about `-lstdc++`, verify `c++ --version` shows GCC.

### Building

```bash
source /opt/ros/jazzy/setup.bash
cd /workspace
colcon build --symlink-install --parallel-workers 4 --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

Do **not** use the `scripts/clean_rebuild.sh` or `scripts/minimal_rebuild.sh` scripts in the Cloud VM—they source `/opt/ros/humble/` and have interactive prompts.

### Testing

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
colcon test --packages-select <package>
colcon test-result --verbose
```

Core functional tests pass (542 tests, 0 failures). Lint tests (ament_copyright, flake8, xmllint) have pre-existing failures in `explore_lite` and `multirobot_map_merge`—these are cosmetic and not caused by the Jazzy port.

### Running nodes (no physical robot)

The full autonomous exploration pipeline requires a physical TurtleBot3. Without hardware, you can still:

- Launch `robot_state_publisher` with the TurtleBot3 URDF to publish TF frames
- Run `explore_lite` (it will wait for a costmap that never arrives)
- Publish/subscribe to `/cmd_vel` and other ROS 2 topics
- Verify launch files load correctly

### Key file locations

- Build scripts: `scripts/` (designed for Humble, not usable as-is in Cloud VM)
- Nav2 params: `src/turtlebot3/turtlebot3_navigation2/param/humble/`
- SLAM params: `src/turtlebot3/turtlebot3_navigation2/param/humble/mapper_params_online_async_fast.yaml`
- Explorer config: `src/m-explore-ros2/explore/config/params.yaml`
- Robot URDF: `src/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf`
- README with full startup sequence: `README.md`
