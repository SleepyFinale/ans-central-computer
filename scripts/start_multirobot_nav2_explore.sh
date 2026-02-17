#!/bin/bash
# Multi-robot Nav2 + Explorer: multiple robots explore autonomously
# Prerequisites: multirobot_slam running, domain bridges, tf_relay running

if [ -z "$ROS_DOMAIN_ID" ]; then
    export ROS_DOMAIN_ID=50
    echo "Note: ROS_DOMAIN_ID not set, using 50 (central)"
fi

cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Starting Multi-Robot Nav2 + Explorer"
echo ""
echo "Prerequisites (must be running in separate terminals):"
echo "  1. ./scripts/start_domain_bridges.sh"
echo "  2. ./scripts/start_multirobot_slam.sh   (includes tf_relay)"
echo ""
echo "If TF wait times out, run diagnostics: python3 scripts/diagnose_multirobot_tf.py"
echo ""

USE_SIM_TIME="${USE_SIM_TIME:-false}"
ros2 launch turtlebot3_navigation2 multirobot_nav2_explore.launch.py use_sim_time:="$USE_SIM_TIME"
