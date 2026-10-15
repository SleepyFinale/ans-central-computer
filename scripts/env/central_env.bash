#!/usr/bin/env bash
# Source in a central-PC terminal to see the fleet the way the central stack does:
#   source scripts/env/central_env.bash
#   ros2 topic list            # /tf, /map, /<robot>/map, /<robot>/navigate_to_pose, ...
#
# Loads ROS 2 Humble, this workspace's install/ overlay, and the DDS settings
# that see what the central Zenoh bridge injects (Cyclone, localhost,
# central ROS_DOMAIN_ID from config/fleet_domain_map.yaml).

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "ERROR: /opt/ros/humble/setup.bash not found. Is ROS 2 Humble installed?"
  return 1 2>/dev/null || exit 1
fi

_central_env_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=fleet_common.bash
source "${_central_env_dir}/fleet_common.bash"
unset _central_env_dir

if [ -f "${FLEET_WORKSPACE_DIR}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${FLEET_WORKSPACE_DIR}/install/setup.bash"
else
  echo "WARNING: ${FLEET_WORKSPACE_DIR}/install/setup.bash not found; run scripts/build/rebuild_common.sh"
fi

fleet_use_zenoh_dds
echo "Central ROS env: ROS_DOMAIN_ID=${ROS_DOMAIN_ID} RMW=${RMW_IMPLEMENTATION} ROS_LOCALHOST_ONLY=1"
