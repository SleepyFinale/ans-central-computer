#!/bin/bash
# Start Nav2 for multi-robot (controls Blinky on merged map)
# Requires: multi-robot SLAM running, ROS_DOMAIN_ID=50
#
# Run after: ./scripts/start_multirobot_slam.sh

export ROS_DOMAIN_ID=50
export TF_WAIT_MAP_FRAME=map
export TF_WAIT_ODOM_FRAME=blinky/odom
export TF_WAIT_BASE_FRAMES=blinky/base_footprint

cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

PARAMS="$(pwd)/install/turtlebot3_navigation2/share/turtlebot3_navigation2/param/humble/burger_multirobot_blinky.yaml"
if [ ! -f "$PARAMS" ]; then
    # Fallback: source build
    PARAMS="$(pwd)/src/turtlebot3/turtlebot3_navigation2/param/humble/burger_multirobot_blinky.yaml"
fi

echo "=========================================="
echo "Starting Nav2 (multi-robot, Blinky)"
echo "=========================================="
echo "  Params: burger_multirobot_blinky.yaml"
echo "  TF: map -> blinky/odom -> blinky/base_footprint"
echo ""

ros2 launch turtlebot3_navigation2 navigation2_slam.launch.py \
    params_file:="$PARAMS" \
    use_sim_time:=false
