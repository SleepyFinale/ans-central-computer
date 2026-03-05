#!/usr/bin/env bash
# Set ROS_DOMAIN_ID and ROBOT_SSH for the selected robot.
# Auto-detects WiFi (SNS, RaspAP, or Azure) to pick the correct IP for Blinky/Pinky.
# Must be sourced so variables apply to the current shell:
#   source scripts/set_robot_env.sh blinky
#   source scripts/set_robot_env.sh inky 192.168.0.100

set_robot_usage() {
  echo "Usage: source scripts/set_robot_env.sh <robot> [ip]"
  echo ""
  echo "  robot   One of: blinky, pinky, inky, clyde"
  echo "  ip      Optional. Required for inky/clyde if not using hostname."
  echo ""
  echo "  WiFi auto-detection: Script detects SNS (Lab), Azure, or RaspAP (RPi) and uses the right IP."
  echo ""
  echo "  Robot   ROS_DOMAIN_ID   SNS (lab)            RaspAP (rpi)         Azure"
  echo "  ------  --------------  -------------------  -------------------  -------------------"
  echo "  Blinky  30              blinky@192.168.0.158 blinky@10.3.141.220  blinky@172.20.10.13"
  echo "  Pinky   31              pinky@192.168.0.194  pinky@10.3.141.194   pinky@172.20.10.14"
  echo "  Inky    32              inky@<IP>  (pass ip as second argument)"
  echo "  Clyde   33              clyde@<IP> (pass ip as second argument)"
}

# Detect current WiFi SSID. Returns "SNS", "RaspAP", "Azure", or empty if unknown/not connected.
get_wifi_ssid() {
  local ssid
  if command -v nmcli >/dev/null 2>&1; then
    ssid=$(nmcli -t -f active,ssid dev wifi 2>/dev/null | awk -F: '$1=="yes" {print $2; exit}')
  elif command -v iwgetid >/dev/null 2>&1; then
    ssid=$(iwgetid -r 2>/dev/null)
  else
    echo ""
    return
  fi
  echo "${ssid:-}"
}

# Resolve network name from SSID: SNS -> lab, RaspAP -> rpi, Azure -> azure
get_network_from_ssid() {
  case "$1" in
    SNS)    echo "lab" ;;
    RaspAP) echo "rpi" ;;
    Azure)  echo "azure" ;;
    *)      echo "unknown" ;;
  esac
}

robot=$(echo "${1:-}" | tr '[:upper:]' '[:lower:]')
ip="${2:-}"

# Robot IPs by network (SNS vs RaspAP vs Azure). Inky/Clyde use user-provided IP.
BLINKY_LAB=192.168.0.158
BLINKY_AZURE=172.20.10.13
BLINKY_RPI=10.3.141.220
PINKY_LAB=192.168.0.194
PINKY_AZURE=172.20.10.14
PINKY_RPI=10.3.141.194

case "$robot" in
  blinky)
    export ROS_DOMAIN_ID=30
    ssid=$(get_wifi_ssid)
    net=$(get_network_from_ssid "$ssid")
    case "$net" in
      lab)   export ROBOT_SSH="blinky@$BLINKY_LAB" ;;
      rpi)   export ROBOT_SSH="blinky@$BLINKY_RPI" ;;
      azure) export ROBOT_SSH="blinky@$BLINKY_AZURE" ;;
      *)     export ROBOT_SSH="blinky@$BLINKY_LAB"
             echo "Warning: Unknown WiFi '$ssid', defaulting to Lab (SNS) IP"
             ;;
    esac
    echo "Robot: Blinky  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  ROBOT_SSH=$ROBOT_SSH  (network: $net)"
    ;;
  pinky)
    export ROS_DOMAIN_ID=31
    ssid=$(get_wifi_ssid)
    net=$(get_network_from_ssid "$ssid")
    case "$net" in
      lab)   export ROBOT_SSH="pinky@$PINKY_LAB" ;;
      rpi)   export ROBOT_SSH="pinky@$PINKY_RPI" ;;
      azure) export ROBOT_SSH="pinky@$PINKY_AZURE" ;;
      *)     export ROBOT_SSH="pinky@$PINKY_LAB"
             echo "Warning: Unknown WiFi '$ssid', defaulting to Lab (SNS) IP"
             ;;
    esac
    echo "Robot: Pinky  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  ROBOT_SSH=$ROBOT_SSH  (network: $net)"
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
