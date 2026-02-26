#!/bin/bash
# Start multi-robot map merge + TF relay only (no SLAM, no normalizers on central PC).
# Use when each robot runs its own SLAM and Nav2; domain bridges forward maps and TF.
#
# Run domain bridges first: ./scripts/start_domain_bridges.sh
# Then run this script.

export ROS_DOMAIN_ID=50

cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=========================================="
echo "Starting Multi-Robot Map Merge (no SLAM)  "
echo "=========================================="
echo ""
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  Requires domain bridges to be running!"
echo "  Expects /blinky/map, /pinky/map from robots (via bridges)."
echo ""

ros2 launch turtlebot3_navigation2 multirobot_map_merge_only.launch.py use_sim_time:=false
