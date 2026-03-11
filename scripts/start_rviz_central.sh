#!/usr/bin/env bash

# Start RViz for the central computer with sensible defaults.
# - Global mode (default): show the merged /map (or single-robot /map) in frame 'map'.
# - Local mode: show a specific robot's local map and topics using its namespace.
#
# Usage:
#   ./scripts/start_rviz_central.sh                # global map view
#   ./scripts/start_rviz_central.sh --global       # same as default
#   ./scripts/start_rviz_central.sh --local pinky  # local view for /pinky/...
#   ./scripts/start_rviz_central.sh -r blinky      # shorthand for --local blinky

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

GLOBAL_CFG="${WORKSPACE_DIR}/config/rviz/central_global_map.rviz"
LOCAL_TEMPLATE_CFG="${WORKSPACE_DIR}/config/rviz/central_robot_local_map_template.rviz"

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

if [[ "$MODE" == "global" ]]; then
    if [[ ! -f "$GLOBAL_CFG" ]]; then
        echo "ERROR: Global RViz config not found at: $GLOBAL_CFG" >&2
        exit 1
    fi

    echo "Starting RViz in GLOBAL mode using:"
    echo "  $GLOBAL_CFG"
    echo "Fixed frame: map, map topic: /map"
    exec rviz2 -d "$GLOBAL_CFG"
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

    # Warn if the expected map topic is not currently available, but still start RViz.
    if command -v ros2 >/dev/null 2>&1; then
        if ! ros2 topic list 2>/dev/null | grep -qE "^/${ROBOT_NAME}/map$"; then
            echo "WARNING: /${ROBOT_NAME}/map not currently visible on this ROS domain."
            echo "         RViz will still start; use this to debug connectivity/SLAM if needed."
        fi
    fi

    TMP_CFG="$(mktemp "/tmp/central_rviz_${ROBOT_NAME}_XXXX.rviz")"

    # Substitute __ROBOT__ placeholder with the requested robot namespace.
    if command -v perl >/dev/null 2>&1; then
        perl -pe "s/__ROBOT__/${ROBOT_NAME}/g" "$LOCAL_TEMPLATE_CFG" > "$TMP_CFG"
    else
        sed "s/__ROBOT__/${ROBOT_NAME}/g" "$LOCAL_TEMPLATE_CFG" > "$TMP_CFG"
    fi

    echo "Starting RViz in LOCAL mode for robot '${ROBOT_NAME}' using:"
    echo "  $TMP_CFG"
    echo "Fixed frame: map, map topic: /${ROBOT_NAME}/map"
    exec rviz2 -d "$TMP_CFG"
fi

