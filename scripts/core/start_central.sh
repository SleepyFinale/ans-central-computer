#!/bin/bash
# Start the central exploration stack.
#
# Usage:
#   ./scripts/core/start_central.sh          # every robot that shows up
#   ./scripts/core/start_central.sh -ic      # inky + clyde
#
# Robot letters: b=blinky p=pinky i=inky c=clyde.
#
# Needs ./scripts/comms/start_zenoh_central.sh running (it provides /tf,
# /tf_static, /<robot>/map and the Nav2 actions) and, on each robot, bringup,
# navigation2_slam.launch.py and start_zenoh_robot.sh.
#
# Starts:
#   one robot:  static TF map -> <robot>/map, /<robot>/map -> /map, explorer
#   several:    map_merge (/map + map -> <robot>/map TF), merge-state monitor, explorer
#
# Environment knobs:
#   CENTRAL_DISCOVERY_TIMEOUT_SEC   wait for robot maps (default 30)
#   EXPLORER_FREQUENCY_HZ           override explore_frequency
#   EXPLORER_TERMINAL_EVENT_LOG_MODE=summary|verbose
#   EXPLORER_TERMINAL_SUMMARY_PERIOD_SEC=<sec>
#   CENTRAL_MAP_MERGE_RAW_STDOUT=true   keep map_merge's OpenCV matrix dumps
#   CENTRAL_AUTO_KILL=true              kill a stale central stack without asking
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../env/fleet_common.bash
source "${SCRIPT_DIR}/../env/fleet_common.bash"
WORKSPACE_DIR="$FLEET_WORKSPACE_DIR"
EXPLORE_DIR="${WORKSPACE_DIR}/src/m-explore-ros2/explore"
EXPLORER_SCRIPT="${EXPLORE_DIR}/scripts/multi_robot_explorer.py"
EXPLORE_CONFIG_FILE="${EXPLORE_DIR}/config/multi_robot_explorer.yaml"
SINGLE_ROBOT_WORLD_TF_SCRIPT="${EXPLORE_DIR}/scripts/single_robot_world_tf_bridge.py"
SINGLE_ROBOT_MAP_RELAY_SCRIPT="${EXPLORE_DIR}/scripts/single_robot_map_relay.py"
MAP_MERGE_CONFIG_FILE="${WORKSPACE_DIR}/src/m-explore-ros2/map_merge/config/multirobot_params_unknown_poses.yaml"
MERGE_MONITOR_SCRIPT="${WORKSPACE_DIR}/scripts/diagnostics/map_merge_state_monitor.py"

SELECTION=""
while (($# > 0)); do
    case "$1" in
        -[bpic]*)
            SELECTION="$1"
            ;;
        --comms-mode)
            # Zenoh is the only transport now; accept the old flag so
            # existing commands keep working.
            shift
            echo "NOTE: --comms-mode is no longer used (Zenoh into one domain)."
            ;;
        -h|--help)
            sed -n '2,25p' "$0" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *)
            echo "ERROR: unknown argument '$1'"
            exit 1
            ;;
    esac
    shift
done
fleet_select_robots "$SELECTION"

for f in "$EXPLORER_SCRIPT" "$EXPLORE_CONFIG_FILE" "$SINGLE_ROBOT_WORLD_TF_SCRIPT" \
         "$SINGLE_ROBOT_MAP_RELAY_SCRIPT" "$MAP_MERGE_CONFIG_FILE" "$MERGE_MONITOR_SCRIPT"; do
    if [[ ! -f "$f" ]]; then
        echo "ERROR: missing ${f}"
        exit 1
    fi
done

cd "$WORKSPACE_DIR"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source install/setup.bash 2>/dev/null || {
    echo "ERROR: build the workspace first: bash scripts/build/rebuild_common.sh"
    exit 1
}
fleet_use_zenoh_dds

echo "=========================================="
echo "  Central Computer — Multi-Robot Exploration"
echo "=========================================="
echo "  ROS_DOMAIN_ID = ${ROS_DOMAIN_ID} (localhost, ${RMW_IMPLEMENTATION})"
echo "  Robots        = ${ROBOTS[*]}"
echo ""

if ! pgrep -f 'zenoh-bridge-ros2dds' >/dev/null 2>&1; then
    echo "WARNING: no zenoh-bridge-ros2dds is running on this PC."
    echo "         Start ./scripts/comms/start_zenoh_central.sh first."
    echo ""
fi

# Two central stacks would send every robot two sets of goals.
ensure_no_central_stack_running() {
    local pattern='python3 .*(multi_robot_explorer|single_robot_world_tf_bridge|single_robot_map_relay|map_merge_state_monitor)\.py|multirobot_map_merge/map_merge'
    local existing pids
    existing="$(pgrep -af -- "$pattern" | grep -v "pgrep" || true)"
    [[ -z "$existing" ]] && return 0

    echo "A central stack is already running:"
    echo "$existing" | sed 's/^/  /'
    if [[ "${CENTRAL_AUTO_KILL:-false}" != "true" ]]; then
        if [[ ! -t 0 ]]; then
            echo "Stop it first, or set CENTRAL_AUTO_KILL=true."
            exit 1
        fi
        read -r -p "Kill these processes and continue? [y/N] " reply
        [[ "$reply" == "y" || "$reply" == "Y" ]] || exit 1
    fi
    pids="$(echo "$existing" | awk '{print $1}')"
    # shellcheck disable=SC2086
    kill -TERM $pids 2>/dev/null || true
    sleep 2
    # shellcheck disable=SC2086
    kill -KILL $pids 2>/dev/null || true
    echo ""
}
ensure_no_central_stack_running

DISCOVERY_TIMEOUT="${CENTRAL_DISCOVERY_TIMEOUT_SEC:-30}"
echo "Waiting up to ${DISCOVERY_TIMEOUT}s for robot maps (/<robot>/map)..."
readarray -t ACTIVE_ROBOTS < <(fleet_robots_with_maps "$DISCOVERY_TIMEOUT" "${ROBOTS[@]}")

if ((${#ACTIVE_ROBOTS[@]} == 0)); then
    echo "ERROR: no robot map arrived within ${DISCOVERY_TIMEOUT}s."
    echo "Check, in order:"
    echo "  1. ./scripts/comms/start_zenoh_central.sh is running here."
    echo "  2. On each robot: bringup, navigation2_slam.launch.py, start_zenoh_robot.sh $(hostname -s | tr '[:upper:]' '[:lower:]')."
    echo "  3. ros2 topic echo --once /<robot>/map_wire_z  (here, after: source scripts/env/fleet_common.bash; fleet_use_zenoh_dds)"
    echo "Wait longer with CENTRAL_DISCOVERY_TIMEOUT_SEC=60."
    exit 1
fi
if ((${#ACTIVE_ROBOTS[@]} < ${#ROBOTS[@]})); then
    echo "WARNING: no map yet from: $(comm -23 <(printf '%s\n' "${ROBOTS[@]}" | sort) <(printf '%s\n' "${ACTIVE_ROBOTS[@]}" | sort) | tr '\n' ' ')"
fi
echo "Using robots: ${ACTIVE_ROBOTS[*]}"
ROBOT_LIST_PARAM="$(IFS=,; echo "${ACTIVE_ROBOTS[*]}")"

PIDS=()
cleanup() {
    trap - SIGHUP SIGINT SIGTERM EXIT
    echo ""
    echo "Shutting down..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    wait 2>/dev/null
    echo "Done."
}
trap cleanup SIGHUP SIGINT SIGTERM EXIT

EXPLORER_ARGS=(
    "$EXPLORER_SCRIPT" --ros-args
    --params-file "$EXPLORE_CONFIG_FILE"
    -p "robot_names:=[${ROBOT_LIST_PARAM}]"
    -p "use_pose_goal_fallback:=false"
)
[[ -n "${EXPLORER_FREQUENCY_HZ:-}" ]] && EXPLORER_ARGS+=(-p "explore_frequency:=${EXPLORER_FREQUENCY_HZ}")
[[ -n "${EXPLORER_TERMINAL_SUMMARY_PERIOD_SEC:-}" ]] && EXPLORER_ARGS+=(-p "terminal_summary_period_sec:=${EXPLORER_TERMINAL_SUMMARY_PERIOD_SEC}")
[[ -n "${EXPLORER_TERMINAL_EVENT_LOG_MODE:-}" ]] && EXPLORER_ARGS+=(-p "terminal_event_log_mode:=${EXPLORER_TERMINAL_EVENT_LOG_MODE}")

if ((${#ACTIVE_ROBOTS[@]} == 1)); then
    ROBOT="${ACTIVE_ROBOTS[0]}"
    echo "  Mode = single robot (no map_merge)"
    echo "[1/3] Static TF map -> ${ROBOT}/map (identity)"
    python3 "$SINGLE_ROBOT_WORLD_TF_SCRIPT" --ros-args -p "robot_name:=${ROBOT}" &
    PIDS+=($!)
    echo "[2/3] Map relay /${ROBOT}/map -> /map"
    python3 "$SINGLE_ROBOT_MAP_RELAY_SCRIPT" --ros-args -p "source_map:=/${ROBOT}/map" &
    PIDS+=($!)
    echo "[3/3] Explorer"
    EXPLORER_ARGS+=(
        -p "map_topic:=/${ROBOT}/map"
        -p "world_frame:=map"
        -p "single_robot_offloaded_nav2:=true"
    )
else
    echo "  Mode = multi robot (map_merge)"
    echo "[1/3] map_merge (unknown initial poses)"
    MAP_MERGE_BIN="$(ros2 pkg prefix multirobot_map_merge)/lib/multirobot_map_merge/map_merge"
    if [[ "${CENTRAL_MAP_MERGE_RAW_STDOUT:-false}" == "true" ]]; then
        "$MAP_MERGE_BIN" --ros-args --params-file "$MAP_MERGE_CONFIG_FILE" &
    else
        # Drop the OpenCV matrix / feature-count dumps map_merge prints to stdout.
        # Process substitution keeps $! on map_merge itself, so cleanup stops it.
        "$MAP_MERGE_BIN" --ros-args --params-file "$MAP_MERGE_CONFIG_FILE" > >(
            grep --line-buffered -vE '^(features|matches|inliers|inliers/matches ratio|confidence):|^[0-9]+ +[0-9]+$|^ *\[? *-?[0-9.e-]+(, *-?[0-9.e-]+)*;?\]?$'
        ) 2>&1 &
    fi
    PIDS+=($!)
    echo "[2/3] Merge-state monitor (map_merge/merge_state)"
    # Publishes NO_OVERLAP / PARTIAL / MERGED so the explorer can switch from
    # per-robot frontiers to the merged map.
    python3 "$MERGE_MONITOR_SCRIPT" --ros-args \
        -p "robot_names:=[${ROBOT_LIST_PARAM}]" \
        -p "world_frame:=map" \
        -p "stability_pos_threshold:=0.28" \
        -p "merged_dwell_time:=20.0" \
        -p "motion_jitter_confirm_ticks:=3" \
        -p "min_known_cells_per_robot:=450" \
        -p "min_known_area_m2_per_robot:=1.1" &
    PIDS+=($!)
    echo "[3/3] Explorer"
fi
python3 "${EXPLORER_ARGS[@]}" &
PIDS+=($!)

echo ""
echo "=========================================="
echo "  All services running.  Press Ctrl+C to stop."
echo "=========================================="
echo ""
wait
