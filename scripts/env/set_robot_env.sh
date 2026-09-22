#!/usr/bin/env bash
# Set ROBOT_SSH for the selected robot.
# Auto-detects WiFi (Azure or TAMU_WiFi) to pick the correct SSH target.
# Azure uses static fleet IPs.
# TAMU_WiFi prefers Tailscale MagicDNS (<robot>@<robot>) when Tailscale is up;
# otherwise falls back to a required DHCP IP override.
# Must be sourced so variables apply to the current shell:
#   source scripts/env/set_robot_env.sh clyde

set_robot_usage() {
  echo "Usage: source scripts/env/set_robot_env.sh <robot> [ip]"
  echo ""
  echo "  robot   blinky|pinky|inky|clyde (auto-detected) OR custom robot name"
  echo "  ip      Optional override. On TAMU_WiFi without Tailscale, required for"
  echo "          original robots. Always required for custom robot names."
  echo ""
  echo "  WiFi auto-detection: Script detects Azure (azure) or TAMU_WiFi (tamu)."
  echo "  TAMU_WiFi: prefers Tailscale MagicDNS (e.g. clyde@clyde) when available."
  echo ""
  echo "  Robot   Azure (azure)           TAMU + Tailscale"
  echo "  ------  ---------------------   ----------------"
  echo "  Blinky  blinky@172.20.10.13     blinky@blinky"
  echo "  Pinky   pinky@172.20.10.14      pinky@pinky"
  echo "  Inky    inky@172.20.10.15       inky@inky"
  echo "  Clyde   clyde@172.20.10.16      clyde@clyde"
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

# Return 0 if Tailscale MagicDNS name for the robot looks reachable.
# Uses `tailscale status --json` peer HostName match (Online) when possible;
# otherwise falls back to getent on the short MagicDNS name.
tailscale_robot_reachable() {
  local name="$1"
  if ! command -v tailscale >/dev/null 2>&1; then
    return 1
  fi

  if tailscale status --json 2>/dev/null | python3 -c '
import json, sys
name = sys.argv[1].lower()
try:
    data = json.load(sys.stdin)
except Exception:
    sys.exit(1)
for peer in (data.get("Peer") or {}).values():
    host = (peer.get("HostName") or "").lower()
    dns = (peer.get("DNSName") or "").lower().rstrip(".")
    if not peer.get("Online"):
        continue
    if host == name or dns == name or dns.startswith(name + "."):
        sys.exit(0)
sys.exit(1)
' "$name"
  then
    return 0
  fi

  if getent hosts "$name" >/dev/null 2>&1; then
    return 0
  fi
  return 1
}

# Set ROBOT_SSH for an original fleet robot from detected WiFi.
set_fleet_robot_ssh() {
  local name="$1"
  local azure_ip
  local net_label
  azure_ip=$(get_azure_ip "$name")
  ssid=$(get_wifi_ssid)
  net=$(get_network_from_ssid "$ssid")
  net_label="$net"
  case "$net" in
    azure)
      export ROBOT_SSH="${name}@${azure_ip}"
      ;;
    tamu)
      if [ -n "$ip" ]; then
        export ROBOT_SSH="${name}@${ip}"
        net_label="tamu"
      elif tailscale_robot_reachable "$name"; then
        export ROBOT_SSH="${name}@${name}"
        net_label="tamu/tailscale"
      else
        echo "Error: WiFi '$ssid' (TAMU) and Tailscale peer '$name' not reachable."
        echo "  - Ensure the robot ran: sudo tailscale up && sudo tailscale set --hostname=${name}"
        echo "  - Or pass a current DHCP IP: source scripts/env/set_robot_env.sh ${name} <ip>"
        return 1 2>/dev/null || exit 1
      fi
      ;;
    *)
      # Prefer Tailscale if available (e.g. Ethernet + Tailscale, or unknown SSID).
      if [ -n "$ip" ]; then
        export ROBOT_SSH="${name}@${ip}"
        net_label="manual"
      elif tailscale_robot_reachable "$name"; then
        export ROBOT_SSH="${name}@${name}"
        net_label="tailscale"
      else
        export ROBOT_SSH="${name}@${azure_ip}"
        net_label="unknown"
        echo "Warning: Unknown WiFi '$ssid', defaulting to Azure IP"
        echo "  Tip (TAMU): ensure Tailscale is up and robot hostname is '${name}', then re-run."
      fi
      ;;
  esac
  echo "Robot: $(robot_display_name "$name")  ROBOT_SSH=$ROBOT_SSH  (network: $net_label)"
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
      if tailscale_robot_reachable "$robot"; then
        export ROBOT_SSH="${robot}@${robot}"
        echo "Robot: ${robot}  ROBOT_SSH=$ROBOT_SSH  (network: tailscale)"
      else
        echo "Unknown robot: $1"
        echo "For non-original robots, provide an explicit IP override or Tailscale hostname:"
        echo "  source scripts/env/set_robot_env.sh <robot> <ip>"
        set_robot_usage
        return 1 2>/dev/null || exit 1
      fi
    else
      export ROBOT_SSH="${robot}@${ip}"
      echo "Robot: ${robot}  ROBOT_SSH=$ROBOT_SSH  (manual IP override for non-original robot)"
    fi
    ;;
esac
