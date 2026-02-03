#!/usr/bin/env bash
# Set ROS_DOMAIN_ID and ROBOT_SSH for the selected robot.
# Must be sourced so variables apply to the current shell:
#   source scripts/set_robot_env.sh blinky
#   source scripts/set_robot_env.sh inky 192.168.50.100

set_robot_usage() {
  echo "Usage: source scripts/set_robot_env.sh <robot> [ip]"
  echo ""
  echo "  robot   One of: blinky, pinky, inky, clyde"
  echo "  ip      Optional. Required for inky/clyde if not using hostname."
  echo ""
  echo "  Robot   ROS_DOMAIN_ID   SSH target"
  echo "  ------  --------------  -------------------------"
  echo "  Blinky  30              blinky@192.168.50.193"
  echo "  Pinky   31              pinky@192.168.50.219"
  echo "  Inky    32              inky@<IP>  (pass ip as second argument)"
  echo "  Clyde   33              clyde@<IP> (pass ip as second argument)"
}

robot=$(echo "${1:-}" | tr '[:upper:]' '[:lower:]')
ip="${2:-}"

case "$robot" in
  blinky)
    export ROS_DOMAIN_ID=30
    export ROBOT_SSH=blinky@192.168.50.193
    echo "Robot: Blinky  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  ROBOT_SSH=$ROBOT_SSH"
    ;;
  pinky)
    export ROS_DOMAIN_ID=31
    export ROBOT_SSH=pinky@192.168.50.219
    echo "Robot: Pinky  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  ROBOT_SSH=$ROBOT_SSH"
    ;;
  inky)
    export ROS_DOMAIN_ID=32
    if [ -n "$ip" ]; then
      export ROBOT_SSH="inky@$ip"
      echo "Robot: Inky  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  ROBOT_SSH=$ROBOT_SSH"
    else
      unset ROBOT_SSH 2>/dev/null || true
      echo "Robot: Inky  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  ROBOT_SSH not set (pass IP: source scripts/set_robot_env.sh inky <IP>)"
    fi
    ;;
  clyde)
    export ROS_DOMAIN_ID=33
    if [ -n "$ip" ]; then
      export ROBOT_SSH="clyde@$ip"
      echo "Robot: Clyde  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  ROBOT_SSH=$ROBOT_SSH"
    else
      unset ROBOT_SSH 2>/dev/null || true
      echo "Robot: Clyde  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  ROBOT_SSH not set (pass IP: source scripts/set_robot_env.sh clyde <IP>)"
    fi
    ;;
  "")
    set_robot_usage
    return 1 2>/dev/null || exit 1
    ;;
  -h|--help)
    set_robot_usage
    return 0 2>/dev/null || exit 0
    ;;
  *)
    echo "Unknown robot: $1"
    set_robot_usage
    return 1 2>/dev/null || exit 1
    ;;
esac
