#!/bin/bash
# Simple explorer launcher - starts with Nav2 costmap config
# The explorer will wait for the costmap automatically

cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Starting explorer with Nav2 costmap configuration..."
echo "Explorer will wait for /global_costmap/costmap to become available."
echo ""

# Use Nav2 costmap (recommended when Nav2 is running)
ros2 run explore_lite explore --ros-args \
    --params-file src/m-explore-ros2/explore/config/params_costmap.yaml
