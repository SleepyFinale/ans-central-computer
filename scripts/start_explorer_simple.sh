#!/bin/bash
# Simple explorer launcher - starts with Nav2 costmap config
# The explorer will wait for the costmap automatically

# Set ROS_DOMAIN_ID if not already set (defaults to 0 if not set)
if [ -z "$ROS_DOMAIN_ID" ]; then
    export ROS_DOMAIN_ID=30
    echo "Note: ROS_DOMAIN_ID not set, using default: 30"
    echo "  To make permanent, add to ~/.bashrc: export ROS_DOMAIN_ID=30"
fi

cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Starting explorer with SLAM map (direct from slam_toolbox)..."
echo "Explorer will wait for /map topic to become available."
echo ""
echo "Note: Using SLAM map directly instead of costmap for better frontier detection"
echo "      when the map is still small or narrow."
echo ""

# Use SLAM map directly (better for exploration when map is small/narrow)
# The costmap might be too restrictive - using SLAM map gives better frontier detection
# Set use_sim_time to match Nav2 configuration (global_costmap uses True)
ros2 run explore_lite explore --ros-args \
    --params-file src/m-explore-ros2/explore/config/params.yaml \
    -p use_sim_time:=True
