#!/bin/bash
# Start domain bridges for multi-robot operation.
#
# Bridges robot data TO the central computer (maps, TF)
# and exploration waypoints FROM the central computer TO robots (goal_pose).
#
# Robot domains:  Blinky=30  Pinky=31  Inky=32
# Central domain: 50

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
CONFIG_DIR="${WORKSPACE_DIR}/config/domain_bridge"

source /opt/ros/humble/setup.bash

echo "=========================================="
echo "Starting Domain Bridges"
echo "=========================================="
echo ""
echo "  Robot → Central:"
echo "    Blinky (domain 30 → 50): map, TF"
echo "    Pinky  (domain 31 → 50): map, TF"
echo "    Inky   (domain 32 → 50): map, TF"
echo ""
echo "  Central → Robot (waypoints):"
echo "    Blinky (domain 50 → 30): goal_pose"
echo "    Pinky  (domain 50 → 31): goal_pose"
echo "    Inky   (domain 50 → 32): goal_pose"
echo ""
echo "Press Ctrl+C to stop all bridges."
echo ""

PIDS=()

ros2 run domain_bridge domain_bridge "${CONFIG_DIR}/blinky_bridge.yaml" &
PIDS+=($!)
ros2 run domain_bridge domain_bridge "${CONFIG_DIR}/pinky_bridge.yaml" &
PIDS+=($!)
ros2 run domain_bridge domain_bridge "${CONFIG_DIR}/blinky_goals_bridge.yaml" &
PIDS+=($!)
ros2 run domain_bridge domain_bridge "${CONFIG_DIR}/pinky_goals_bridge.yaml" &
PIDS+=($!)
ros2 run domain_bridge domain_bridge "${CONFIG_DIR}/inky_bridge.yaml" &
PIDS+=($!)
ros2 run domain_bridge domain_bridge "${CONFIG_DIR}/inky_goals_bridge.yaml" &
PIDS+=($!)

cleanup() {
    echo ""
    echo "Stopping bridges..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    wait
}
trap cleanup SIGINT SIGTERM
wait
