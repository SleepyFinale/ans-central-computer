#!/usr/bin/env bash
# Set ROBOT_SSH for the selected robot.
# Auto-detects WiFi (SNS, Starlink, RaspAP, or Azure) to pick the correct IP for Blinky, Pinky, Inky, and Clyde.
# Must be sourced so variables apply to the current shell:
#   source scripts/set_robot_env.sh blinky

set_robot_usage() {
  echo "Usage: source scripts/set_robot_env.sh <robot> [ip]"
  echo ""
  echo "  robot   One of: blinky, pinky, inky, clyde"
  echo "  ip      Optional. Override auto-detected IP (e.g. non-standard network)."
  echo ""
  echo "  WiFi auto-detection: Script detects SNS (lab), Starlink (ANS_starlink), RaspAP (rpi), or Azure."
  echo ""
  echo "  Robot   SNS (lab)              ANS_starlink (star)   RaspAP (rpi)         Azure (azure)"
  echo "  ------  ---------------------  --------------------  -------------------  -------------------"
  echo "  Blinky  blinky@192.168.0.158   blinky@192.168.1.158  blinky@10.3.141.158  blinky@172.20.10.13"
  echo "  Pinky   pinky@192.168.0.194    pinky@192.168.1.194   pinky@10.3.141.194   pinky@172.20.10.14"
  echo "  Inky    inky@192.168.0.139     inky@192.168.1.139    inky@10.3.141.139    inky@172.20.10.15"
  echo "  Clyde   clyde@192.168.0.236    clyde@192.168.1.236   clyde@10.3.141.236   clyde@172.20.10.16"
}

# Detect current WiFi SSID. Returns "SNS", "ANS_starlink", "RaspAP", "Azure", or empty if unknown/not connected.
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

# Resolve network name from SSID: SNS -> lab, ANS_starlink -> star, RaspAP -> rpi, Azure -> azure
get_network_from_ssid() {
  case "$1" in
    SNS)           echo "lab" ;;
    ANS_starlink)  echo "star" ;;
    RaspAP)        echo "rpi" ;;
    Azure)         echo "azure" ;;
    *)             echo "unknown" ;;
  esac
}

robot=$(echo "${1:-}" | tr '[:upper:]' '[:lower:]')
ip="${2:-}"

# Robot IPs by network. Matches ans-turtlebot3 scripts/wifi/switch_wifi.sh.
BLINKY_LAB=192.168.0.158
BLINKY_STAR=192.168.1.158
BLINKY_AZURE=172.20.10.13
BLINKY_RPI=10.3.141.158
PINKY_LAB=192.168.0.194
PINKY_STAR=192.168.1.194
PINKY_AZURE=172.20.10.14
PINKY_RPI=10.3.141.194
INKY_LAB=192.168.0.139
INKY_STAR=192.168.1.139
INKY_RPI=10.3.141.139
INKY_AZURE=172.20.10.15
CLYDE_LAB=192.168.0.236
CLYDE_STAR=192.168.1.236
CLYDE_RPI=10.3.141.236
CLYDE_AZURE=172.20.10.16

case "$robot" in
  blinky)
    ssid=$(get_wifi_ssid)
    net=$(get_network_from_ssid "$ssid")
    case "$net" in
      lab)   export ROBOT_SSH="blinky@$BLINKY_LAB" ;;
      star)  export ROBOT_SSH="blinky@$BLINKY_STAR" ;;
      rpi)   export ROBOT_SSH="blinky@$BLINKY_RPI" ;;
      azure) export ROBOT_SSH="blinky@$BLINKY_AZURE" ;;
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
      star)  export ROBOT_SSH="pinky@$PINKY_STAR" ;;
      rpi)   export ROBOT_SSH="pinky@$PINKY_RPI" ;;
      azure) export ROBOT_SSH="pinky@$PINKY_AZURE" ;;
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
      star)  export ROBOT_SSH="inky@$INKY_STAR" ;;
      rpi)   export ROBOT_SSH="inky@$INKY_RPI" ;;
      azure) export ROBOT_SSH="inky@$INKY_AZURE" ;;
      *)     export ROBOT_SSH="inky@$INKY_LAB"
             echo "Warning: Unknown WiFi '$ssid', defaulting to Lab (SNS) IP"
             ;;
    esac
    echo "Robot: Inky  ROBOT_SSH=$ROBOT_SSH  (network: $net)"
    ;;
  clyde)
    if [ -n "$ip" ]; then
      export ROBOT_SSH="clyde@$ip"
      echo "Robot: Clyde  ROBOT_SSH=$ROBOT_SSH  (manual IP override)"
    else
      ssid=$(get_wifi_ssid)
      net=$(get_network_from_ssid "$ssid")
      case "$net" in
        lab)   export ROBOT_SSH="clyde@$CLYDE_LAB" ;;
        star)  export ROBOT_SSH="clyde@$CLYDE_STAR" ;;
        rpi)   export ROBOT_SSH="clyde@$CLYDE_RPI" ;;
        azure) export ROBOT_SSH="clyde@$CLYDE_AZURE" ;;
        *)     export ROBOT_SSH="clyde@$CLYDE_LAB"
               echo "Warning: Unknown WiFi '$ssid', defaulting to Lab (SNS) IP"
               ;;
      esac
      echo "Robot: Clyde  ROBOT_SSH=$ROBOT_SSH  (network: $net)"
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
