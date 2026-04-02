#!/usr/bin/env bash
#
# Sources ROS 2 Humble plus this workspace install/ overlay.
#
# Fleet: use ROS_DOMAIN_ID=50 on central PC and every robot (see robot README).
# Per-robot SSH targets: source scripts/set_robot_env.sh <blinky|pinky|inky|clyde>
# Namespaces match hostnames: /blinky, /pinky, /inky, /clyde.

# Base ROS 2 Humble underlay
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "ERROR: /opt/ros/humble/setup.bash not found. Is ROS 2 Humble installed?"
  return 1 2>/dev/null || exit 1
fi

# Workspace overlay
# If WS_DIR is not provided, infer it from this script's location (…/turtlebot3_ws/scripts).
if [ -z "${WS_DIR:-}" ]; then
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  WS_DIR="$(dirname "$SCRIPT_DIR")"
fi

if [ -f "${WS_DIR}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WS_DIR}/install/setup.bash"
else
  echo "ERROR: ${WS_DIR}/install/setup.bash not found."
  echo "Make sure you are in the correct workspace and have run: colcon build"
  return 1 2>/dev/null || exit 1
fi

echo "ROS 2 Humble and workspace environment loaded from:"
echo "  Underlay: /opt/ros/humble"
echo "  Overlay : ${WS_DIR}"
