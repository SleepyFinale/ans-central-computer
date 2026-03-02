# AGENTS.md

## Cursor Cloud specific instructions

### Overview

Central-computer side of a multi-robot TurtleBot3 exploration system. Robots run SLAM + Nav2 locally; this repo runs on the central PC to merge maps and coordinate exploration. Key packages built from source: TurtleBot3 core, DynamixelSDK, turtlebot3_msgs, explore_lite, multirobot_map_merge.

### ROS 2 distribution: Jazzy (not Humble)

The Cloud VM runs Ubuntu 24.04 (Noble). ROS 2 Humble cannot be installed (requires Python 3.10 / libtinyxml2-9). ROS 2 **Jazzy** is used instead — the workspace code is compatible with both. Shell scripts try `source /opt/ros/humble/setup.bash` then fall back to Jazzy.

**Consequence:** Build scripts in `scripts/` reference Humble paths. Source Jazzy manually:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
```

### Compiler: GCC must be default

The VM's default `c++` points to clang which cannot find `-lstdc++`. The update script sets `c++` → `g++-13` via `update-alternatives`.

### Building

```bash
source /opt/ros/jazzy/setup.bash
cd /workspace
colcon build --symlink-install --parallel-workers 4 --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

### Testing

```bash
colcon test --packages-select multirobot_map_merge
colcon test-result --verbose
```

Core functional tests pass. Lint tests have pre-existing failures.

### Running (no physical robots)

The full system requires physical TurtleBot3 robots. Without hardware you can verify the build, run unit tests, and test individual nodes with fake maps (see previous test logs).

### Key file locations

- Central computer entry point: `scripts/start_central.sh`
- Explorer: `scripts/multi_robot_explorer.py`
- TF relay: `scripts/tf_relay_multirobot.py`
- Domain bridge configs: `config/domain_bridge/`
- Map merge config: `config/map_merge/multirobot_params_unknown_poses.yaml`
- Explorer config: `config/multi_robot_explorer.yaml`
- Full documentation: `README.md`
