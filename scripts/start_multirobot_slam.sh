#!/bin/bash
# Start multi-robot SLAM (Blinky + Pinky + Inky + Clyde) with map merge
# Requires: domain bridges running, ROS_DOMAIN_ID=50
#
# Run domain bridges first: ./scripts/start_domain_bridges.sh
# Then run this script.

export ROS_DOMAIN_ID=50

cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=========================================="
echo "Starting Multi-Robot SLAM                 "
echo "=========================================="
echo ""
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  Requires domain bridges to be running!"
echo ""

ros2 launch turtlebot3_navigation2 multirobot_slam.launch.py use_sim_time:=false
