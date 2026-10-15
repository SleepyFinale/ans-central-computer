#!/usr/bin/env bash
# Shared helpers for the central fleet scripts. Source, do not run.
#
#   FLEET_ROBOTS            every robot, in fleet order
#   fleet_select_robots -ic sets ROBOTS=(inky clyde); no filter means all
#   fleet_central_domain    prints central_domain_id from fleet_domain_map.yaml
#   fleet_use_zenoh_dds     DDS settings that see what the Zenoh bridge injects
#   fleet_robots_with_maps  prints the robots whose /<robot>/map has arrived

_FLEET_ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FLEET_WORKSPACE_DIR="$(cd "${_FLEET_ENV_DIR}/../.." && pwd)"
FLEET_DOMAIN_MAP="${FLEET_WORKSPACE_DIR}/config/fleet_domain_map.yaml"

FLEET_ROBOTS=(blinky pinky inky clyde)
declare -A FLEET_LETTER_TO_ROBOT=([b]=blinky [p]=pinky [i]=inky [c]=clyde)

# Usage: fleet_select_robots [-<letters>]. Exits on a bad letter.
fleet_select_robots() {
  local letters="${1#-}" ch robot i
  local -A chosen=()
  ROBOTS=()
  if [[ -z "$letters" ]]; then
    ROBOTS=("${FLEET_ROBOTS[@]}")
    return 0
  fi
  for ((i = 0; i < ${#letters}; i++)); do
    ch="${letters:$i:1}"
    if [[ -z "${FLEET_LETTER_TO_ROBOT[$ch]:-}" ]]; then
      echo "ERROR: invalid robot letter '${ch}' in '-${letters}'." >&2
      echo "Use any combination of: b=blinky, p=pinky, i=inky, c=clyde" >&2
      exit 1
    fi
    chosen["${FLEET_LETTER_TO_ROBOT[$ch]}"]=1
  done
  for robot in "${FLEET_ROBOTS[@]}"; do
    if [[ -n "${chosen[$robot]:-}" ]]; then
      ROBOTS+=("$robot")
    fi
  done
}

fleet_central_domain() {
  python3 - "$FLEET_DOMAIN_MAP" <<'PY' 2>/dev/null || echo 50
import sys, yaml
with open(sys.argv[1], encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}
print(int((data.get("fleet_domain_map") or {}).get("central_domain_id", 50)))
PY
}

# The central Zenoh bridge injects robot topics through Cyclone's built-in
# localhost profile (ROS_LOCALHOST_ONLY=1, no CYCLONEDDS_URI). Every central
# process that should see them must use the same profile. A URI that also
# selects 127.0.0.1 does not see the injected topics. RViz is the exception;
# see start_rviz_central.sh.
fleet_use_zenoh_dds() {
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_LOCALHOST_ONLY=1
  unset CYCLONEDDS_URI
  if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
    ROS_DOMAIN_ID="$(fleet_central_domain)"
    export ROS_DOMAIN_ID
  fi
}

# Usage: fleet_robots_with_maps <timeout_sec> <robot>...
# A robot counts once its map has arrived. Topic names alone do not count:
# fleet_zenoh_receive.py advertises /<robot>/map before the robot connects.
fleet_robots_with_maps() {
  python3 - "$@" <<'PY'
import sys, time
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

timeout, robots = float(sys.argv[1]), sys.argv[2:]
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                 reliability=ReliabilityPolicy.RELIABLE)
rclpy.init()
node = rclpy.create_node('fleet_robot_discovery')
seen = set()
for r in robots:
    node.create_subscription(OccupancyGrid, f'/{r}/map', lambda _m, r=r: seen.add(r), qos)
deadline = time.monotonic() + timeout
while time.monotonic() < deadline and len(seen) < len(robots):
    rclpy.spin_once(node, timeout_sec=0.5)
for r in robots:
    if r in seen:
        print(r)
node.destroy_node()
rclpy.shutdown()
PY
}
