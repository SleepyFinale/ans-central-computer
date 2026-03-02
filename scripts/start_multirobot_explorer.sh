#!/bin/bash
# Start the multi-robot frontier explorer on the central PC.
#
# Prerequisites:
#   - Domain bridges running (start_domain_bridges.sh)
#   - Multi-robot SLAM running (start_multirobot_slam.sh)
#   - Multi-robot Nav2 running for each robot
#
# Usage:
#   ROS_DOMAIN_ID=50 ./scripts/start_multirobot_explorer.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash 2>/dev/null
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
