#!/usr/bin/env bash
# Set a robot's top linear speed in its running Nav2 (factor D of the ENTC 641
# experiment). start_fleet.sh calls this through FLEET_AFTER_SLAM_HOOK, once per
# robot after Nav2 is up and before any goal is sent:
#
#   ENTC_MAX_SPEED=0.15 entc641/set_robot_speed.sh <robot> <ssh target>
#
# Sets the regulated pure pursuit controller's desired_linear_vel and the
# velocity smoother's linear limits, then reads both back. Exits non-zero if
# either node is missing or a value did not take.
set -euo pipefail

ROBOT="$1"
TARGET="$2"
SPEED="${ENTC_MAX_SPEED:?set ENTC_MAX_SPEED (m/s)}"
: "${SSHPASS:=${ROBOT_SSH_PASSWORD:-ubuntu}}"
export SSHPASS

if [[ ! "$SPEED" =~ ^0\.[0-9]+$ ]]; then
  echo "ERROR: ENTC_MAX_SPEED must look like 0.15, got '${SPEED}'" >&2
  exit 1
fi

echo "$(date +%H:%M:%S) ${ROBOT}: setting max linear speed ${SPEED} m/s"
sshpass -e ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$TARGET" bash -s <<EOF
set +u
cd "\$HOME/turtlebot3"
source scripts/env/ros_robot_env.bash >/dev/null
set -e
nodes=\$(ros2 node list 2>/dev/null)
ctrl=\$(grep -E '/controller_server\$' <<<"\$nodes" | head -1)
smoother=\$(grep -E '/velocity_smoother\$' <<<"\$nodes" | head -1)
if [[ -z "\$ctrl" || -z "\$smoother" ]]; then
  echo "ERROR: controller_server or velocity_smoother not found:" >&2
  echo "\$nodes" >&2
  exit 1
fi
ros2 param set "\$ctrl" FollowPath.desired_linear_vel ${SPEED}
ros2 param set "\$smoother" max_velocity "[${SPEED}, 0.0, 2.0]"
ros2 param set "\$smoother" min_velocity "[-${SPEED}, 0.0, -2.0]"
got_ctrl=\$(ros2 param get "\$ctrl" FollowPath.desired_linear_vel)
got_max=\$(ros2 param get "\$smoother" max_velocity)
echo "\$ctrl FollowPath.desired_linear_vel: \$got_ctrl"
echo "\$smoother max_velocity: \$got_max"
grep -q "${SPEED}" <<<"\$got_ctrl"
grep -q "${SPEED}" <<<"\$got_max"
EOF
echo "$(date +%H:%M:%S) ${ROBOT}: speed set"
