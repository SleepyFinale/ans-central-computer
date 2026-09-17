#!/usr/bin/env bash
# Set ROBOT_SSH for the selected robot.
# Auto-detects WiFi (Azure or TAMU_WiFi) to pick the correct SSH target.
# Azure uses static fleet IPs; TAMU_WiFi uses DHCP (manual IP required).
# Must be sourced so variables apply to the current shell:
#   source scripts/env/set_robot_env.sh blinky

set_robot_usage() {
  echo "Usage: source scripts/env/set_robot_env.sh <robot> [ip]"
  echo ""
  echo "  robot   blinky|pinky|inky|clyde (auto-detected) OR custom robot name"
  echo "  ip      Required for custom robot names; also required on TAMU_WiFi for original robots."
  echo ""
  echo "  WiFi auto-detection: Script detects Azure (azure) or TAMU_WiFi (tamu)."
  echo "  Note: TAMU_WiFi is DHCP. For blinky/pinky/inky/clyde on TAMU, pass [ip] explicitly."
  echo ""
  echo "  Robot   Azure (azure)"
  echo "  ------  ---------------------"
  echo "  Blinky  blinky@172.20.10.13"
  echo "  Pinky   pinky@172.20.10.14"
  echo "  Inky    inky@172.20.10.15"
  echo "  Clyde   clyde@172.20.10.16"
}

# Detect current WiFi SSID. Returns current SSID or empty if unknown/not connected.
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

# Resolve network name from SSID.
get_network_from_ssid() {
  case "$1" in
    Azure)         echo "azure" ;;
    TAMU_WiFi)     echo "tamu" ;;
    *)             echo "unknown" ;;
  esac
}

# Azure hotspot static IPs. Matches ans-turtlebot3 scripts/network/ AZURE_* layout
# (gateway 172.20.10.1, prefix 28).
get_azure_ip() {
  case "$1" in
    blinky) echo "172.20.10.13" ;;
    pinky)  echo "172.20.10.14" ;;
    inky)   echo "172.20.10.15" ;;
    clyde)  echo "172.20.10.16" ;;
    *)      echo "" ;;
  esac
}

robot_display_name() {
  case "$1" in
    blinky) echo "Blinky" ;;
    pinky)  echo "Pinky" ;;
    inky)   echo "Inky" ;;
    clyde)  echo "Clyde" ;;
    *)      echo "$1" ;;
  esac
}

# Set ROBOT_SSH for an original fleet robot from detected WiFi.
set_fleet_robot_ssh() {
  local name="$1"
  local azure_ip
  azure_ip=$(get_azure_ip "$name")
  ssid=$(get_wifi_ssid)
  net=$(get_network_from_ssid "$ssid")
  case "$net" in
    azure)
      export ROBOT_SSH="${name}@${azure_ip}"
      ;;
    tamu)
      if [ -z "$ip" ]; then
        echo "Error: WiFi '$ssid' uses DHCP; provide current robot IP for '$name'."
        echo "Example: source scripts/env/set_robot_env.sh ${name} 10.42.0.123"
        return 1 2>/dev/null || exit 1
      fi
      export ROBOT_SSH="${name}@${ip}"
      ;;
    *)
      export ROBOT_SSH="${name}@${azure_ip}"
      echo "Warning: Unknown WiFi '$ssid', defaulting to Azure IP"
      ;;
  esac
  echo "Robot: $(robot_display_name "$name")  ROBOT_SSH=$ROBOT_SSH  (network: $net)"
}

robot=$(echo "${1:-}" | tr '[:upper:]' '[:lower:]')
ip="${2:-}"

case "$robot" in
  blinky|pinky|inky|clyde)
    set_fleet_robot_ssh "$robot" || return 1 2>/dev/null || exit 1
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
