#!/usr/bin/env bash

# Start RViz for the central computer with sensible defaults.
# - Global mode (default): show the merged /map (or single-robot /map) in frame 'map'.
# - Local mode: show a specific robot's local map and topics using its namespace.
#   Includes namespaced global/local costmap overlays to visualize inflated/lethal zones.
#
# Usage:
#   ./scripts/core/start_rviz_central.sh                # global map view
#   ./scripts/core/start_rviz_central.sh --global       # same as default
#   ./scripts/core/start_rviz_central.sh --local pinky  # local view for /pinky/...
#   ./scripts/core/start_rviz_central.sh -r blinky      # shorthand for --local blinky

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

GLOBAL_CFG="${WORKSPACE_DIR}/config/rviz/central_global_map.rviz"
LOCAL_TEMPLATE_CFG="${WORKSPACE_DIR}/config/rviz/central_robot_local_map_template.rviz"
DOMAIN_MAP_FILE="${WORKSPACE_DIR}/config/fleet_domain_map.yaml"
DEFAULT_CENTRAL_DOMAIN_ID="$(
    python3 - <<'PY' "$DOMAIN_MAP_FILE"
import sys, yaml
path, fallback = sys.argv[1], 50
try:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    cid = data.get("fleet_domain_map", {}).get("central_domain_id")
    print(int(cid) if cid is not None else fallback)
except Exception:
    print(fallback)
PY
)"

declare -A ROBOT_DOMAIN_MAP=()
ROBOT_NAMES=()

print_usage() {
    cat <<EOF
Usage:
  $(basename "$0") [--global] [--local <robot>] [-r <robot>] [--help]

Modes:
  --global            Open RViz with the global map config (default).
  --local <robot>     Open RViz showing a specific robot's local map, e.g.:
                        $(basename "$0") --local pinky
  -r <robot>          Shorthand for --local <robot>.

Examples:
  $(basename "$0")
  $(basename "$0") --global
  $(basename "$0") --local pinky
  $(basename "$0") -r blinky
EOF
}

load_domain_map() {
    # Parse the fleet domain map once and cache it into bash arrays so mode
    # selection and diagnostics can reuse the same robot->domain lookup.
    if [[ ! -f "$DOMAIN_MAP_FILE" ]]; then
        return 1
    fi

    local map_lines=""
    map_lines="$(python3 - <<'PY' "$DOMAIN_MAP_FILE" 2>/dev/null || true
import sys, yaml
with open(sys.argv[1], "r", encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}
robot_map = data.get("fleet_domain_map", {}).get("robot_domain_ids", {})
for name, domain in sorted(robot_map.items()):
    print(f"{name} {int(domain)}")
PY
)"

    if [[ -z "$map_lines" ]]; then
        return 1
    fi

    local line name domain
    while IFS= read -r line; do
        [[ -z "$line" ]] && continue
        name="$(awk '{print $1}' <<<"$line")"
        domain="$(awk '{print $2}' <<<"$line")"
        [[ -z "$name" || -z "$domain" ]] && continue
        ROBOT_DOMAIN_MAP["$name"]="$domain"
        ROBOT_NAMES+=("$name")
    done <<<"$map_lines"

    [[ ${#ROBOT_NAMES[@]} -gt 0 ]]
}

get_robot_domain() {
    local robot="$1"
    if [[ -n "${ROBOT_DOMAIN_MAP[$robot]:-}" ]]; then
        echo "${ROBOT_DOMAIN_MAP[$robot]}"
        return 0
    fi
    return 1
}

detect_active_robots() {
    # Active means the robot domain currently exposes /<robot>/map.
    # We probe each domain directly rather than relying on central graph relay.
    if ! command -v ros2 >/dev/null 2>&1; then
        return 0
    fi

    local name domain topics_raw
    local active=()
    for name in "${ROBOT_NAMES[@]}"; do
        domain="$(get_robot_domain "$name" || true)"
        [[ -z "$domain" ]] && continue
        topics_raw="$(ROS_DOMAIN_ID="$domain" ros2 topic list 2>/dev/null || true)"
        if grep -qE "^/${name}/map$" <<<"$topics_raw"; then
            active+=("$name")
        fi
    done

    if [[ ${#active[@]} -gt 0 ]]; then
        printf '%s\n' "${active[@]}"
    fi
}

generate_local_cfg() {
    # Render a robot-specific RViz config from the template by swapping
    # namespace placeholders and fixed frame (map -> <robot>/map).
    local robot="$1"
    local out_cfg="$2"
    local local_fixed_frame="${robot}/map"

    if command -v perl >/dev/null 2>&1; then
        perl -pe "s/__ROBOT__/${robot}/g; s#Fixed Frame: map#Fixed Frame: ${local_fixed_frame}#g" "$LOCAL_TEMPLATE_CFG" > "$out_cfg"
    else
        sed -e "s/__ROBOT__/${robot}/g" -e "s#Fixed Frame: map#Fixed Frame: ${local_fixed_frame}#g" "$LOCAL_TEMPLATE_CFG" > "$out_cfg"
    fi
}

MODE="global"
ROBOT_NAME=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --global)
            MODE="global"
            shift
            ;;
        --local|-r)
            MODE="local"
            if [[ -z "${2:-}" ]]; then
                echo "ERROR: --local/-r requires a robot name (e.g. 'pinky')." >&2
                print_usage
                exit 1
            fi
            ROBOT_NAME="$2"
            shift 2
            ;;
        -h|--help)
            print_usage
            exit 0
            ;;
        *)
            echo "ERROR: Unknown argument: $1" >&2
            print_usage
            exit 1
            ;;
    esac
done

if ! command -v rviz2 >/dev/null 2>&1; then
    echo "ERROR: rviz2 is not in PATH. Make sure ROS 2 Humble is installed and sourced." >&2
    exit 1
fi

HAS_DOMAIN_MAP=false
if load_domain_map; then
    HAS_DOMAIN_MAP=true
fi

if [[ "$MODE" == "global" ]]; then
    if [[ ! -f "$GLOBAL_CFG" ]]; then
        echo "ERROR: Global RViz config not found at: $GLOBAL_CFG" >&2
        exit 1
    fi

    EFFECTIVE_CFG="$GLOBAL_CFG"
    if [[ -n "${ROS_DOMAIN_ID:-}" ]]; then
        EFFECTIVE_DOMAIN="${ROS_DOMAIN_ID}"
        DOMAIN_SOURCE="env(ROS_DOMAIN_ID)"
    else
        export ROS_DOMAIN_ID="${DEFAULT_CENTRAL_DOMAIN_ID}"
        EFFECTIVE_DOMAIN="${ROS_DOMAIN_ID}"
        DOMAIN_SOURCE="default(${DEFAULT_CENTRAL_DOMAIN_ID})"
    fi
    EFFECTIVE_MAP_TOPIC="/map"
    EFFECTIVE_ROBOT=""
    EFFECTIVE_FIXED_FRAME="map"
    EFFECTIVE_VIEW_MODE="global"
    ACTIVE_ROBOTS_RAW=""

    if [[ "$HAS_DOMAIN_MAP" == true ]]; then
        ACTIVE_ROBOTS_RAW="$(detect_active_robots || true)"
        if [[ -n "$ACTIVE_ROBOTS_RAW" ]]; then
            mapfile -t ACTIVE_ROBOTS <<<"$ACTIVE_ROBOTS_RAW"
            # Convenience behavior: in nominal "global" mode, when exactly one
            # robot is active we auto-switch to that robot's local map view to
            # avoid empty global map sessions during single-robot runs.
            if [[ ${#ACTIVE_ROBOTS[@]} -eq 1 ]]; then
                EFFECTIVE_ROBOT="${ACTIVE_ROBOTS[0]}"
                if robot_domain="$(get_robot_domain "$EFFECTIVE_ROBOT" || true)" && [[ -n "$robot_domain" ]]; then
                    export ROS_DOMAIN_ID="$robot_domain"
                    EFFECTIVE_DOMAIN="$ROS_DOMAIN_ID"
                    DOMAIN_SOURCE="auto(single_robot_local:${EFFECTIVE_ROBOT})"
                    EFFECTIVE_MAP_TOPIC="/${EFFECTIVE_ROBOT}/map"
                    EFFECTIVE_FIXED_FRAME="${EFFECTIVE_ROBOT}/map"
                    EFFECTIVE_VIEW_MODE="local"
                    if [[ ! -f "$LOCAL_TEMPLATE_CFG" ]]; then
                        echo "ERROR: Local RViz template config not found at: $LOCAL_TEMPLATE_CFG" >&2
                        exit 1
                    fi
                    EFFECTIVE_CFG="$(mktemp "/tmp/central_rviz_auto_local_${EFFECTIVE_ROBOT}_XXXX.rviz")"
                    generate_local_cfg "$EFFECTIVE_ROBOT" "$EFFECTIVE_CFG"
                fi
            fi
        fi
    fi

    echo "Starting RViz in ${EFFECTIVE_VIEW_MODE^^} mode using:"
    echo "  $EFFECTIVE_CFG"
    if [[ -n "$EFFECTIVE_ROBOT" ]]; then
        echo "Detected single active robot: ${EFFECTIVE_ROBOT}"
    else
        echo "Detected active robots: ${ACTIVE_ROBOTS_RAW:-none/unknown}"
    fi
    echo "Effective ROS_DOMAIN_ID: ${EFFECTIVE_DOMAIN}"
    echo "Domain source: ${DOMAIN_SOURCE}"
    echo "Fixed frame: ${EFFECTIVE_FIXED_FRAME}, map topic: ${EFFECTIVE_MAP_TOPIC}"
    if [[ "$EFFECTIVE_VIEW_MODE" == "local" ]]; then
        echo "Costmap overlays: /${EFFECTIVE_ROBOT}/global_costmap/costmap and /${EFFECTIVE_ROBOT}/local_costmap/costmap"
    fi
    exec rviz2 -d "$EFFECTIVE_CFG"
else
    if [[ -z "$ROBOT_NAME" ]]; then
        echo "ERROR: Robot name is required for local mode." >&2
        print_usage
        exit 1
    fi

    if [[ ! -f "$LOCAL_TEMPLATE_CFG" ]]; then
        echo "ERROR: Local RViz template config not found at: $LOCAL_TEMPLATE_CFG" >&2
        exit 1
    fi

    if [[ "$HAS_DOMAIN_MAP" != true ]]; then
        echo "ERROR: Domain map is required for --local mode auto-switch: $DOMAIN_MAP_FILE" >&2
        exit 1
    fi
    robot_domain="$(get_robot_domain "$ROBOT_NAME" || true)"
    if [[ -z "$robot_domain" ]]; then
        echo "ERROR: Robot '${ROBOT_NAME}' not found in domain map: $DOMAIN_MAP_FILE" >&2
        exit 1
    fi
    export ROS_DOMAIN_ID="$robot_domain"

    # Warn if the expected map topic is not currently available, but still start RViz.
    if command -v ros2 >/dev/null 2>&1; then
        if ! ros2 topic list 2>/dev/null | grep -qE "^/${ROBOT_NAME}/map$"; then
            echo "WARNING: /${ROBOT_NAME}/map not currently visible on this ROS domain."
            echo "         RViz will still start; use this to debug connectivity/SLAM if needed."
        fi
    fi

    TMP_CFG="$(mktemp "/tmp/central_rviz_${ROBOT_NAME}_XXXX.rviz")"

    # Substitute __ROBOT__ placeholder with the requested robot namespace.
    local_fixed_frame="${ROBOT_NAME}/map"
    generate_local_cfg "$ROBOT_NAME" "$TMP_CFG"

    echo "Starting RViz in LOCAL mode for robot '${ROBOT_NAME}' using:"
    echo "  $TMP_CFG"
    echo "Effective ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-unset}"
    echo "Domain source: local_robot_map(${ROBOT_NAME})"
    echo "Fixed frame: ${local_fixed_frame}, map topic: /${ROBOT_NAME}/map"
    echo "Costmap overlays: /${ROBOT_NAME}/global_costmap/costmap and /${ROBOT_NAME}/local_costmap/costmap"
    echo ""
    echo "Performance (Wi‑Fi / domain_bridge): RViz is capped at 10 Hz; global costmap overlay"
    echo "starts disabled (enable in Displays if you need it). Two Map layers + high FPS often"
    echo "cause GPU stalls and GLSL warnings on central machines."
    echo ""
    exec rviz2 -d "$TMP_CFG"
fi

