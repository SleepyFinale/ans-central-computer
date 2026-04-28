#!/usr/bin/env bash
# Set ROBOT_SSH for the selected robot.
# Auto-detects WiFi (SNS, GCRI_LAB, or RaspAP) to pick the correct IP for Blinky, Pinky, Inky, and Clyde.
# Must be sourced so variables apply to the current shell:
#   source scripts/env/set_robot_env.sh blinky

set_robot_usage() {
  echo "Usage: source scripts/env/set_robot_env.sh <robot> [ip]"
  echo ""
  echo "  robot   blinky|pinky|inky|clyde (auto-detected) OR custom robot name"
  echo "  ip      Required for custom robot names; ignored for the original four."
  echo ""
  echo "  WiFi auto-detection: Script detects SNS (lab), GCRI_LAB (gcri), or RaspAP (rpi)."
  echo ""
  echo "  Robot   SNS (lab)              GCRI_LAB (gcri)       RaspAP (rpi)"
  echo "  ------  ---------------------  ---------------------  -------------------"
  echo "  Blinky  blinky@192.168.0.158   blinky@192.168.50.158  blinky@10.3.141.158"
  echo "  Pinky   pinky@192.168.0.194    pinky@192.168.50.194   pinky@10.3.141.194"
  echo "  Inky    inky@192.168.0.139     inky@192.168.50.139    inky@10.3.141.139"
  echo "  Clyde   clyde@192.168.0.236    clyde@192.168.50.236   clyde@10.3.141.236"
}

# Detect current WiFi SSID. Returns "SNS", "GCRI_LAB", "RaspAP", or empty if unknown/not connected.
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

# Resolve network name from SSID: SNS -> lab, GCRI_LAB -> gcri, RaspAP -> rpi
get_network_from_ssid() {
  case "$1" in
    SNS)           echo "lab" ;;
    GCRI_LAB)      echo "gcri" ;;
    RaspAP)        echo "rpi" ;;
    *)             echo "unknown" ;;
  esac
}

robot=$(echo "${1:-}" | tr '[:upper:]' '[:lower:]')
ip="${2:-}"

case "$robot" in
  blinky|pinky|inky|clyde)
    if [ -n "$ip" ]; then
      echo "Note: Ignoring manual IP override for original robot '$robot'; using WiFi auto-detection."
    fi
    ;;
esac

# Robot IPs by network. Matches ans-turtlebot3 scripts/wifi/switch_wifi.sh.
BLINKY_LAB=192.168.0.158
BLINKY_GCRI=192.168.50.158
BLINKY_RPI=10.3.141.158
PINKY_LAB=192.168.0.194
PINKY_GCRI=192.168.50.194
PINKY_RPI=10.3.141.194
INKY_LAB=192.168.0.139
INKY_GCRI=192.168.50.139
INKY_RPI=10.3.141.139
CLYDE_LAB=192.168.0.236
CLYDE_GCRI=192.168.50.236
CLYDE_RPI=10.3.141.236

case "$robot" in
  blinky)
    ssid=$(get_wifi_ssid)
    net=$(get_network_from_ssid "$ssid")
    case "$net" in
      lab)   export ROBOT_SSH="blinky@$BLINKY_LAB" ;;
      gcri)  export ROBOT_SSH="blinky@$BLINKY_GCRI" ;;
      rpi)   export ROBOT_SSH="blinky@$BLINKY_RPI" ;;
      *)     export ROBOT_SSH="blinky@$BLINKY_LAB"
             echo "Warning: Unknown WiFi '$ssid', defaulting to Lab (SNS) IP"
             ;;
    esac
    echo "Robot: Blinky  ROBOT_SSH=$ROBOT_SSH  (network: $net)"
    ;;
  pinky)
    ssid=$(get_wifi_ssid)
    net=$(get_network_from_ssid "$ssid")
    case "$net" in
      lab)   export ROBOT_SSH="pinky@$PINKY_LAB" ;;
      gcri)  export ROBOT_SSH="pinky@$PINKY_GCRI" ;;
      rpi)   export ROBOT_SSH="pinky@$PINKY_RPI" ;;
      *)     export ROBOT_SSH="pinky@$PINKY_LAB"
             echo "Warning: Unknown WiFi '$ssid', defaulting to Lab (SNS) IP"
             ;;
    esac
    echo "Robot: Pinky  ROBOT_SSH=$ROBOT_SSH  (network: $net)"
    ;;
  inky)
    ssid=$(get_wifi_ssid)
    net=$(get_network_from_ssid "$ssid")
    case "$net" in
      lab)   export ROBOT_SSH="inky@$INKY_LAB" ;;
      gcri)  export ROBOT_SSH="inky@$INKY_GCRI" ;;
      rpi)   export ROBOT_SSH="inky@$INKY_RPI" ;;
      *)     export ROBOT_SSH="inky@$INKY_LAB"
             echo "Warning: Unknown WiFi '$ssid', defaulting to Lab (SNS) IP"
             ;;
    esac
    echo "Robot: Inky  ROBOT_SSH=$ROBOT_SSH  (network: $net)"
    ;;
  clyde)
    ssid=$(get_wifi_ssid)
    net=$(get_network_from_ssid "$ssid")
    case "$net" in
      lab)   export ROBOT_SSH="clyde@$CLYDE_LAB" ;;
      gcri)  export ROBOT_SSH="clyde@$CLYDE_GCRI" ;;
      rpi)   export ROBOT_SSH="clyde@$CLYDE_RPI" ;;
      *)     export ROBOT_SSH="clyde@$CLYDE_LAB"
             echo "Warning: Unknown WiFi '$ssid', defaulting to Lab (SNS) IP"
             ;;
    esac
    echo "Robot: Clyde  ROBOT_SSH=$ROBOT_SSH  (network: $net)"
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
    if [ -z "$ip" ]; then
      echo "Unknown robot: $1"
      echo "For non-original robots, provide an explicit IP override:"
      echo "  source scripts/env/set_robot_env.sh <robot> <ip>"
      set_robot_usage
      return 1 2>/dev/null || exit 1
    fi
    export ROBOT_SSH="${robot}@${ip}"
    echo "Robot: ${robot}  ROBOT_SSH=$ROBOT_SSH  (manual IP override for non-original robot)"
    ;;
esac
