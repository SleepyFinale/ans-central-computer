# AGENTS.md

## Cursor Cloud specific instructions

### Overview

This is a **ROS 2 Humble** workspace for autonomous multi-robot exploration using TurtleBot3 Burger robots. It builds 14 ROS packages (TurtleBot3 core, DynamixelSDK, explore_lite, multirobot_map_merge) and relies on system-installed Nav2, SLAM Toolbox, and domain_bridge packages.

### Docker-based development (required)

The Cloud VM runs Ubuntu 24.04 but this workspace requires ROS 2 Humble (Ubuntu 22.04). All ROS 2 commands must run inside the Docker container `ros2-dev`:

```bash
# The container is already running with the workspace mounted at /workspace
sudo docker exec ros2-dev bash -c '
  source /opt/ros/humble/setup.bash
  source /workspace/install/setup.bash
  export TURTLEBOT3_MODEL=burger
  <your-command>
'
```

If the container is not running, start it:

```bash
sudo dockerd &>/tmp/dockerd.log &
sleep 3
sudo docker start ros2-dev || sudo docker run -d --name ros2-dev --network host -v /workspace:/workspace -w /workspace ros2-humble-dev sleep infinity
```

### Build

```bash
sudo docker exec ros2-dev bash -c '
  source /opt/ros/humble/setup.bash
  cd /workspace
  colcon build --symlink-install --parallel-workers 4 --cmake-args -DBUILD_TESTING=OFF
'
```

Or use the existing build scripts (but note they have interactive prompts — prefer the colcon command above):
- `scripts/clean_rebuild.sh` — full clean rebuild
- `scripts/minimal_rebuild.sh` — minimal rebuild of essential packages

### Lint

```bash
# C++
sudo docker exec ros2-dev bash -c 'source /opt/ros/humble/setup.bash && ament_cpplint /workspace/src/m-explore-ros2/explore/src/'

# Python
sudo docker exec ros2-dev bash -c 'source /opt/ros/humble/setup.bash && ament_flake8 /workspace/src/turtlebot3/turtlebot3_navigation2/'

# XML (package.xml)
sudo docker exec ros2-dev bash -c 'source /opt/ros/humble/setup.bash && find /workspace/src -maxdepth 3 -name "package.xml" -exec ament_xmllint {} +'

# CMake
sudo docker exec ros2-dev bash -c 'source /opt/ros/humble/setup.bash && find /workspace/src -maxdepth 3 -name "CMakeLists.txt" -exec ament_lint_cmake {} +'
```

### Test

```bash
sudo docker exec ros2-dev bash -c '
  source /opt/ros/humble/setup.bash
  source /workspace/install/setup.bash
  cd /workspace
  colcon test --return-code-on-test-failure
  colcon test-result --all
'
```

### Key gotchas

- The workspace was built with `-DBUILD_TESTING=OFF` for speed. To run tests with testing enabled, rebuild without that flag.
- The `turtlebot3_state_publisher.launch.py` requires a `namespace` argument (use `namespace:=blinky` or similar).
- Physical TurtleBot3 robots are required for full autonomous exploration testing. Without hardware, you can verify builds, launch individual nodes (robot_state_publisher, SLAM Toolbox, explore_lite), and confirm they initialize correctly.
- The `.devcontainer/Dockerfile` at the repo root is the Cloud Agent dev container definition — it is separate from the project's own Docker files under `src/turtlebot3/docker/`.
- After modifying C++ or CMake files, you must rebuild inside the container. Python scripts under `install/` are symlinked, so edits to source `.py` files take effect immediately.
