#!/bin/bash
# Start domain bridges for Blinky and Pinky (multi-robot SLAM)
# Bridges: Blinky (domain 30), Pinky (domain 31), Inky (domain 32), Clyde (domain 33) -> domain 50
# Aggregation domain: 50. Run SLAM/map_merge with ROS_DOMAIN_ID=50

cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

CONFIG_DIR="$(pwd)/config/domain_bridge"

if [ ! -f "${CONFIG_DIR}/blinky_bridge.yaml" ] || [ ! -f "${CONFIG_DIR}/pinky_bridge.yaml" ]; then
    echo "ERROR: Bridge configs not found in ${CONFIG_DIR}"
    exit 1
fi

echo "=========================================="
echo "Starting Domain Bridges                   "
echo "=========================================="
echo ""
echo "  Blinky: domain 30 -> 50  (blinky/scan, blinky/odom, ...)"
echo "  Pinky:  domain 31 -> 50  (pinky/scan, pinky/odom, ...)"
echo "  Inky:   domain 32 -> 50  (inky/scan, inky/odom, ...)"
echo "  Clyde:  domain 33 -> 50  (clyde/scan, clyde/odom, ...)"
echo ""
echo "Aggregation domain: 50. Set ROS_DOMAIN_ID=50 for SLAM/Nav2."
echo "Press Ctrl+C to stop both bridges."
echo ""

ros2 run domain_bridge domain_bridge "${CONFIG_DIR}/blinky_bridge.yaml" &
BLINKY_PID=$!

ros2 run domain_bridge domain_bridge "${CONFIG_DIR}/pinky_bridge.yaml" &
PINKY_PID=$!

trap "kill $BLINKY_PID $PINKY_PID 2>/dev/null; exit" SIGINT SIGTERM
wait
