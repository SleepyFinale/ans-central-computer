#!/bin/bash
# Start the multi-robot frontier explorer on the central PC.
#
# Prerequisites (single-domain, namespaced setup):
#   - All robots and the central PC use the same ROS_DOMAIN_ID (typically 50)
#   - Each robot is running Nav2 + SLAM in its own namespace
#     (e.g. navigation2_slam_namespaced.launch.py robot_name:=blinky ...)
#   - TF relay and map_merge are running on the central PC
#     (for example via ./scripts/start_central.sh)
#
# Usage:
#   ROS_DOMAIN_ID=50 ./scripts/start_multirobot_explorer.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null

CONFIG_FILE="${WORKSPACE_DIR}/config/multi_robot_explorer.yaml"

if [ ! -f "$CONFIG_FILE" ]; then
    echo "ERROR: Config file not found: $CONFIG_FILE"
    exit 1
fi

echo "=========================================="
echo "Starting Multi-Robot Explorer"
echo "=========================================="
echo ""
echo "  Config: $CONFIG_FILE"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
echo ""
echo "  Robots will explore autonomously."
echo "  Press Ctrl+C to stop."
echo ""

python3 "${SCRIPT_DIR}/multi_robot_explorer.py" --ros-args \
    --params-file "$CONFIG_FILE"
