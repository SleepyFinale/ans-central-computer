#!/usr/bin/env bash
# Start the central Zenoh router and the fleet receiver.
#
# Usage:
#   ./scripts/comms/start_zenoh_central.sh           # all fleet robots
#   ./scripts/comms/start_zenoh_central.sh -ic       # inky + clyde
#
# Robot letters: b=blinky p=pinky i=inky c=clyde (same as start_central.sh).
#
# One zenoh-bridge-ros2dds router listens on tcp/0.0.0.0:7447 and puts every
# selected robot's /<robot>/... topics and Nav2 actions into the central ROS
# domain on localhost. fleet_zenoh_receive.py then turns the robots'
# tf_zenoh / map_wire_z streams into /tf, /tf_static and /<robot>/map.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../env/fleet_common.bash
source "${SCRIPT_DIR}/../env/fleet_common.bash"
TEMPLATE="${FLEET_WORKSPACE_DIR}/config/zenoh/central_router.json5.tmpl"

case "${1:-}" in
  -h|--help)
    sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//'
    exit 0
    ;;
  ""|-[bpic]*) ;;
  *)
    echo "ERROR: unknown argument '$1'. Use a robot filter like -c or -bpic, or --help." >&2
    exit 1
    ;;
esac
fleet_select_robots "${1:-}"

# shellcheck source=zenoh_bridge_path.bash
source "${SCRIPT_DIR}/zenoh_bridge_path.bash"
if ! ZENOH_BIN="$(resolve_zenoh_bridge_ros2dds)"; then
  echo "ERROR: zenoh-bridge-ros2dds not found."
  echo "  Install: ${FLEET_WORKSPACE_DIR}/scripts/comms/install_zenoh_bridge.sh"
  exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
# Set after setup.bash, which may have exported ROS_LOCALHOST_ONLY=0.
# The bridge must not also get a CYCLONEDDS_URI: one that selects lo again
# aborts it with "the same interface may not be selected twice".
fleet_use_zenoh_dds
export ROS_DOMAIN_ID="$(fleet_central_domain)"

RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}/zenoh_bridge_central"
mkdir -p "$RUNTIME_DIR"
CONFIG="${RUNTIME_DIR}/router.json5"
ROBOT_REGEX="($(IFS='|'; echo "${ROBOTS[*]}"))"
sed -e "s/__DOMAIN__/${ROS_DOMAIN_ID}/g" -e "s/__ROBOTS__/${ROBOT_REGEX}/g" \
  "$TEMPLATE" > "$CONFIG"

echo "Starting central Zenoh router"
echo "  robots:   ${ROBOTS[*]}"
echo "  listen:   tcp/0.0.0.0:7447"
echo "  domain:   ${ROS_DOMAIN_ID} (localhost, ${RMW_IMPLEMENTATION})"
echo "  binary:   ${ZENOH_BIN}"
echo "  config:   ${CONFIG}"
echo ""
echo "On each robot: ./scripts/comms/start_zenoh_robot.sh $(hostname -s | tr '[:upper:]' '[:lower:]')"
echo ""

PIDS=()
cleanup() {
  local pid
  for pid in "${PIDS[@]+"${PIDS[@]}"}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

"$ZENOH_BIN" -c "$CONFIG" &
ROUTER_PID=$!
PIDS+=("$ROUTER_PID")
python3 "${SCRIPT_DIR}/fleet_zenoh_receive.py" "${ROBOTS[@]}" &
PIDS+=("$!")

wait "$ROUTER_PID"
