#!/bin/bash
# Start all central-computer processes for multi-robot exploration.
#
# This is the single entry point for the central computer.  It starts:
#   1. Domain bridges  (robot ↔ central topic forwarding)
#   2. TF relay        (prefix robot TF frames: odom → blinky/odom, etc.)
#   3. Map merge       (merge individual maps into a global map)
#   4. Explorer        (detect frontiers, send waypoints to robots)
#
# Prerequisites:
#   - Robots are powered on, running bringup + SLAM + Nav2
#   - Central PC and robots are on the same WiFi network
#   - domain_bridge package is installed
#
# Usage:
#   export ROS_DOMAIN_ID=50
#   ./scripts/start_central.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
CONFIG_DIR="${WORKSPACE_DIR}/config"

cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash 2>/dev/null
source install/setup.bash 2>/dev/null

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-50}

echo "=========================================="
echo "  Central Computer — Multi-Robot Exploration"
echo "=========================================="
echo ""
echo "  ROS_DOMAIN_ID = $ROS_DOMAIN_ID"
echo ""

PIDS=()
cleanup() {
    echo ""
    echo "Shutting down..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    wait 2>/dev/null
    echo "Done."
}
trap cleanup SIGINT SIGTERM EXIT

# ---- 1. Domain bridges ----
echo "[1/4] Starting domain bridges..."
BRIDGE_DIR="${CONFIG_DIR}/domain_bridge"
for cfg in "$BRIDGE_DIR"/*.yaml; do
    ros2 run domain_bridge domain_bridge "$cfg" &
    PIDS+=($!)
done
sleep 2

# ---- 2. TF relay ----
echo "[2/4] Starting TF relay (prefix robot frames)..."
python3 "${SCRIPT_DIR}/tf_relay_multirobot.py" &
PIDS+=($!)
sleep 1

# ---- 3. Map merge ----
echo "[3/4] Starting map merge (unknown poses)..."
ros2 run multirobot_map_merge map_merge --ros-args \
    --params-file "${CONFIG_DIR}/map_merge/multirobot_params_unknown_poses.yaml" &
PIDS+=($!)
sleep 2

# ---- 4. Explorer ----
echo "[4/4] Starting multi-robot explorer..."
python3 "${SCRIPT_DIR}/multi_robot_explorer.py" --ros-args \
    --params-file "${CONFIG_DIR}/multi_robot_explorer.yaml" &
PIDS+=($!)

echo ""
echo "=========================================="
echo "  All services running.  Press Ctrl+C to stop."
echo "=========================================="
echo ""
echo "  To visualise: rviz2  (add /map display, set frame to 'map')"
echo ""

wait
