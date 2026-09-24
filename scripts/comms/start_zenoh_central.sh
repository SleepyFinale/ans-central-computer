#!/usr/bin/env bash
# Start Zenoh router (+ optional local client bridges) on the central PC.
# Usage:
#   ./scripts/comms/start_zenoh_central.sh           # all fleet robots
#   ./scripts/comms/start_zenoh_central.sh -c        # Clyde only
#   ./scripts/comms/start_zenoh_central.sh -bpic     # Blinky+Pinky+Inky+Clyde
#   ./scripts/comms/start_zenoh_central.sh -pi       # Pinky+Inky
#
# Robot filter letters match start_central.sh: b=blinky, p=pinky, i=inky, c=clyde.
#
# First selected robot (fleet order) runs as the router on tcp/0.0.0.0:7447.
# Additional robots run as background clients to tcp/127.0.0.1:7447 so each
# ROS_DOMAIN_ID is injected on localhost for domain_bridge / start_central.sh.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DOMAIN_MAP="${WORKSPACE_DIR}/config/fleet_domain_map.yaml"
ROUTER_TMPL="${WORKSPACE_DIR}/config/zenoh/central_router_robot.json5.tmpl"
CLIENT_TMPL="${WORKSPACE_DIR}/config/zenoh/central_client_robot.json5.tmpl"

ALL_ROBOTS=(blinky pinky inky clyde)

domain_for_robot() {
  case "$1" in
    blinky) echo 5 ;;
    pinky) echo 22 ;;
    inky) echo 19 ;;
    clyde) echo 80 ;;
    *) return 1 ;;
  esac
}

# Prefer fleet_domain_map.yaml when present (keeps IDs authoritative).
domain_from_map() {
  local robot="$1"
  if [[ ! -f "$DOMAIN_MAP" ]] || ! command -v python3 >/dev/null 2>&1; then
    domain_for_robot "$robot"
    return
  fi
  python3 - "$DOMAIN_MAP" "$robot" <<'PY' 2>/dev/null || domain_for_robot "$robot"
import sys
path, robot = sys.argv[1], sys.argv[2]
try:
    import yaml
except ImportError:
    # Minimal YAML scrape for robot_domain_ids.<name>: <int>
    text = open(path, encoding="utf-8").read().split("robot_domain_ids:", 1)[-1]
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        if ":" not in line:
            continue
        k, v = [p.strip() for p in line.split(":", 1)]
        if k == robot:
            print(int(v, 10))
            raise SystemExit(0)
    raise SystemExit(1)
else:
    data = yaml.safe_load(open(path, encoding="utf-8"))
    ids = (data.get("fleet_domain_map") or data).get("robot_domain_ids") or {}
    if robot not in ids:
        raise SystemExit(1)
    print(int(ids[robot]))
PY
}

render_config() {
  local tmpl="$1" out="$2" robot="$3" domain="$4"
  python3 - "$tmpl" "$out" "$robot" "$domain" <<'PY'
import pathlib, sys
src, dst, robot, domain = (
    pathlib.Path(sys.argv[1]),
    pathlib.Path(sys.argv[2]),
    sys.argv[3],
    sys.argv[4],
)
text = src.read_text(encoding="utf-8")
text = text.replace("__ROBOT__", robot).replace("__DOMAIN__", domain)
dst.write_text(text, encoding="utf-8")
print(f"Wrote {dst}")
PY
}

# Parse robot filter (same -bpic style as start_central.sh).
#   (no args)  → all robots
#   -c         → clyde only
#   -bpi       → blinky + pinky + inky
declare -A LETTER_TO_ROBOT=(
  [b]="blinky"
  [p]="pinky"
  [i]="inky"
  [c]="clyde"
)
SELECTION=""
while (($# > 0)); do
  case "$1" in
    -[bpic]*)
      SELECTION="${1#-}"
      ;;
    -h|--help)
      cat <<'EOF'
Usage:
  ./scripts/comms/start_zenoh_central.sh [robot_filter]

Robot filter (same letters as start_central.sh):
  b=blinky  p=pinky  i=inky  c=clyde
  No filter → all four robots.

Examples:
  ./scripts/comms/start_zenoh_central.sh
  ./scripts/comms/start_zenoh_central.sh -c
  ./scripts/comms/start_zenoh_central.sh -bpic
  ./scripts/comms/start_zenoh_central.sh -pi
EOF
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument '$1'"
      echo "Use a robot filter like -c or -bpic (b=blinky, p=pinky, i=inky, c=clyde), or --help."
      exit 1
      ;;
  esac
  shift
done

declare -A SELECTED_ROBOT_SET=()
if [[ -z "$SELECTION" ]]; then
  for r in "${ALL_ROBOTS[@]}"; do
    SELECTED_ROBOT_SET["$r"]=1
  done
else
  for ((idx=0; idx<${#SELECTION}; idx++)); do
    ch="${SELECTION:$idx:1}"
    if [[ -z "${LETTER_TO_ROBOT[$ch]:-}" ]]; then
      echo "ERROR: invalid robot selector '${ch}' in '-${SELECTION}'"
      echo "Use any combination of: b=blinky, p=pinky, i=inky, c=clyde"
      exit 1
    fi
    SELECTED_ROBOT_SET["${LETTER_TO_ROBOT[$ch]}"]=1
  done
fi

# Fleet order (blinky → pinky → inky → clyde); first selected becomes the router.
ROBOTS=()
for r in "${ALL_ROBOTS[@]}"; do
  [[ -n "${SELECTED_ROBOT_SET[$r]:-}" ]] && ROBOTS+=("$r")
done

if [[ ${#ROBOTS[@]} -eq 0 ]]; then
  echo "ERROR: no robots selected"
  exit 1
fi

for tmpl in "$ROUTER_TMPL" "$CLIENT_TMPL"; do
  if [[ ! -f "$tmpl" ]]; then
    echo "ERROR: missing template: $tmpl"
    exit 1
  fi
done

# shellcheck source=zenoh_bridge_path.bash
source "${SCRIPT_DIR}/zenoh_bridge_path.bash"
if ! ZENOH_BIN="$(resolve_zenoh_bridge_ros2dds)"; then
  echo "ERROR: zenoh-bridge-ros2dds not found."
  echo "  Install: ${WORKSPACE_DIR}/scripts/comms/install_zenoh_bridge.sh"
  exit 1
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS_DISTRO="${ROS_DISTRO:-humble}"
if [[ -f /opt/ros/${ROS_DISTRO}/setup.bash ]]; then
  # shellcheck disable=SC1090
  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
fi

RUNTIME_ROOT="${XDG_RUNTIME_DIR:-/tmp}/zenoh_bridge_central"
mkdir -p "$RUNTIME_ROOT"

CLIENT_PIDS=()
ROUTER_PID=""
cleanup() {
  local pid
  for pid in "${CLIENT_PIDS[@]+"${CLIENT_PIDS[@]}"}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
  if [[ -n "${ROUTER_PID}" ]] && kill -0 "$ROUTER_PID" 2>/dev/null; then
    kill "$ROUTER_PID" 2>/dev/null || true
    wait "$ROUTER_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

ROUTER_ROBOT="${ROBOTS[0]}"
ROUTER_DOMAIN="$(domain_from_map "$ROUTER_ROBOT")"
ROUTER_CFG="${RUNTIME_ROOT}/${ROUTER_ROBOT}_router.json5"
render_config "$ROUTER_TMPL" "$ROUTER_CFG" "$ROUTER_ROBOT" "$ROUTER_DOMAIN"

echo "Starting Zenoh central bridge(s)"
if [[ -n "$SELECTION" ]]; then
  echo "  filter:   -$SELECTION  (b=blinky, p=pinky, i=inky, c=clyde)"
else
  echo "  filter:   (all robots)"
fi
echo "  robots:   ${ROBOTS[*]}"
echo "  router:   ${ROUTER_ROBOT} (domain ${ROUTER_DOMAIN}) listen tcp/0.0.0.0:7447"
echo "  binary:   ${ZENOH_BIN}"
echo "  RMW:      ${RMW_IMPLEMENTATION}"
echo "  distro:   ${ROS_DISTRO}"
echo "  localhost DDS only: ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo ""
echo "On each robot (turtlebot3 workspace): ./scripts/comms/start_zenoh_robot.sh [central-tailscale-name]"
echo "  e.g. ./scripts/comms/start_zenoh_robot.sh reverie"
echo ""

export ROS_DOMAIN_ID="$ROUTER_DOMAIN"
"${ZENOH_BIN}" -c "$ROUTER_CFG" &
ROUTER_PID=$!

# Extra domain bridges as local clients once the router is listening.
if [[ ${#ROBOTS[@]} -gt 1 ]]; then
  sleep 0.5
  for robot in "${ROBOTS[@]:1}"; do
    domain="$(domain_from_map "$robot")"
    cfg="${RUNTIME_ROOT}/${robot}_client.json5"
    render_config "$CLIENT_TMPL" "$cfg" "$robot" "$domain"
    echo "  client:   ${robot} (domain ${domain}) -> tcp/127.0.0.1:7447"
    (
      export ROS_DOMAIN_ID="$domain"
      export RMW_IMPLEMENTATION
      export ROS_LOCALHOST_ONLY
      exec "${ZENOH_BIN}" -c "$cfg"
    ) &
    CLIENT_PIDS+=("$!")
  done
  echo ""
fi

wait "$ROUTER_PID"
exit $?
