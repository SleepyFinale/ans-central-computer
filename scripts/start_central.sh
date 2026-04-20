#!/bin/bash
# Start all central-computer processes for multi-robot exploration.
#
# This is the single entry point for the central computer when using the
# single-domain, namespaced multi-robot setup. It starts:
#   1. TF relay        (/<robot>/tf → /tf with consistent frame prefixing)
#   1b. Single-robot only: static TF map → <robot>/map (identity bridge)
#   1c. Single-robot only: relay /<robot>/map → /map (fleet Nav2 + global RViz)
#   2. Map merge       (multi-robot: merge maps; skipped when one robot)
#   3. Explorer        (detect frontiers, send Nav2 action goals)
#
# Prerequisites:
#   - All robots and the central PC use the same ROS_DOMAIN_ID (typically 50)
#   - Each robot is powered on and running:
#       * bringup       (robot.launch.py)
#       * SLAM + Nav2   (navigation2_slam.launch.py fleet_mode:=True with this script)
#   - Central PC and robots are on the same WiFi network
#
# Usage:
#   export ROS_DOMAIN_ID=50
#   ./scripts/start_central.sh
#   ./scripts/start_central.sh -b      # only Blinky
#   ./scripts/start_central.sh -pi     # only Pinky + Inky
#   ./scripts/start_central.sh -bpic   # Blinky, Pinky, Inky, Clyde (subset must appear on the graph)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
EXPLORE_PKG_DIR="${WORKSPACE_DIR}/src/m-explore-ros2/explore"
EXPLORE_SCRIPT_DIR="${EXPLORE_PKG_DIR}/scripts"
MAP_MERGE_PKG_DIR="${WORKSPACE_DIR}/src/m-explore-ros2/map_merge"
MAP_MERGE_CONFIG_FILE="${MAP_MERGE_PKG_DIR}/config/multirobot_params_unknown_poses.yaml"
EXPLORE_CONFIG_FILE="${EXPLORE_PKG_DIR}/config/multi_robot_explorer.yaml"
TF_RELAY_SCRIPT="${EXPLORE_SCRIPT_DIR}/tf_relay_multirobot.py"
EXPLORER_SCRIPT="${EXPLORE_SCRIPT_DIR}/multi_robot_explorer.py"
SINGLE_ROBOT_WORLD_TF_SCRIPT="${EXPLORE_SCRIPT_DIR}/single_robot_world_tf_bridge.py"
SINGLE_ROBOT_MAP_RELAY_SCRIPT="${EXPLORE_SCRIPT_DIR}/single_robot_map_relay.py"

cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null

if [[ ! -f "$TF_RELAY_SCRIPT" ]]; then
    echo "ERROR: TF relay script not found: $TF_RELAY_SCRIPT"
    exit 1
fi
if [[ ! -f "$EXPLORER_SCRIPT" ]]; then
    echo "ERROR: Explorer script not found: $EXPLORER_SCRIPT"
    exit 1
fi
if [[ ! -f "$EXPLORE_CONFIG_FILE" ]]; then
    echo "ERROR: Explorer config not found: $EXPLORE_CONFIG_FILE"
    exit 1
fi
if [[ ! -f "$SINGLE_ROBOT_WORLD_TF_SCRIPT" ]]; then
    echo "ERROR: Single-robot world TF script not found: $SINGLE_ROBOT_WORLD_TF_SCRIPT"
    exit 1
fi
if [[ ! -f "$SINGLE_ROBOT_MAP_RELAY_SCRIPT" ]]; then
    echo "ERROR: Single-robot map relay script not found: $SINGLE_ROBOT_MAP_RELAY_SCRIPT"
    exit 1
fi
if [[ ! -f "$MAP_MERGE_CONFIG_FILE" ]]; then
    echo "ERROR: Map merge config not found: $MAP_MERGE_CONFIG_FILE"
    exit 1
fi

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-50}

# Optional selection string like "-bpi" to restrict robots.
SELECTION_RAW="${1:-}"
SELECTION=""
if [[ -n "$SELECTION_RAW" && "$SELECTION_RAW" == -* ]]; then
    SELECTION="${SELECTION_RAW#-}"
fi

echo "=========================================="
echo "  Central Computer — Multi-Robot Exploration"
echo "=========================================="
echo ""
echo "  ROS_DOMAIN_ID = $ROS_DOMAIN_ID"
if [[ -n "$SELECTION" ]]; then
    echo "  Robot filter    = -$SELECTION  (b=blinky, p=pinky, i=inky, c=clyde)"
else
    echo "  Robot filter    = (all detected robots)"
fi
echo ""

# Control whether the explorer falls back to publishing PoseStamped goals on
# /<robot>/goal_pose when the NavigateToPose action server is unavailable.
# Default is "false" so that Nav2 action usage is required and failures are
# surfaced clearly. Override by setting EXPLORER_USE_GOAL_POSE_FALLBACK=true
# before running this script if you need the legacy behaviour.
EXPLORER_USE_GOAL_POSE_FALLBACK="${EXPLORER_USE_GOAL_POSE_FALLBACK:-false}"
EXPLORER_FREQUENCY_HZ="${EXPLORER_FREQUENCY_HZ:-}"
echo "  Explorer fallback = use_pose_goal_fallback=${EXPLORER_USE_GOAL_POSE_FALLBACK}"
if [[ -n "$EXPLORER_FREQUENCY_HZ" ]]; then
    echo "  Explorer frequency override = ${EXPLORER_FREQUENCY_HZ} Hz"
else
    echo "  Explorer frequency override = (using YAML default)"
fi
echo "  Robots must run SLAM+Nav2 with fleet_mode:=True when using this script"
echo "  (global /tf + /map). See README: Robot Terminal 2 — SLAM + Nav2."
echo ""

# Prevent multiple central stacks from running at once. If you start this
# script multiple times (e.g., in different terminals), you'll end up with
# duplicate nodes with the same name publishing conflicting status and
# sending duplicate Nav2 goals.
#
# We validate this using OS processes (authoritative) rather than only the
# ROS graph, because the ROS graph can temporarily retain nodes after
# crashes/network hiccups.
ensure_no_central_stack_running() {
    # Match only central-computer processes launched by this script:
    #  - tf_relay_multirobot.py
    #  - single_robot_world_tf_bridge.py (single-robot mode)
    #  - single_robot_map_relay.py (single-robot mode)
    #  - multi_robot_explorer.py using the central params file
    local pattern_tf="python3 .*tf_relay_multirobot\\.py"
    local pattern_bridge="python3 .*single_robot_world_tf_bridge\\.py"
    local pattern_map_relay="python3 .*single_robot_map_relay\\.py"
    local pattern_explorer="python3 .*multi_robot_explorer\\.py .*--params-file .*multi_robot_explorer\\.yaml"

    # Collect matching PIDs + commands (skip the grep processes themselves).
    local existing
    existing="$(ps aux | grep -E "$pattern_tf|$pattern_bridge|$pattern_map_relay|$pattern_explorer" | grep -v grep || true)"

    if [[ -z "$existing" ]]; then
        return 0
    fi

    echo "ERROR: A central stack appears to already be running (found existing processes):"
    echo "$existing"
    echo ""
    echo "Typical causes:"
    echo "  - An earlier run of this script is still active in another terminal or tmux pane."
    echo "  - A previous run crashed or the terminal was closed without stopping the processes."
    echo ""

    # In non-interactive shells (e.g., launched from another script), do not
    # attempt to prompt; just fail safe unless explicitly overridden.
    if [[ ! -t 0 && "${CENTRAL_AUTO_KILL:-false}" != "true" ]]; then
        echo "This shell is non-interactive; not killing processes automatically."
        echo "To clean up manually, you can run for example:"
        echo "  ps aux | grep multi_robot_explorer.py | grep -v grep"
        echo "  kill <pid>"
        echo ""
        echo "Alternatively, re-run this script from an interactive terminal"
        echo "to be prompted to kill the detected processes, or set"
        echo "CENTRAL_AUTO_KILL=true if you understand the risks."
        exit 1
    fi

    echo "The following PID(s) and command lines were detected:"
    echo ""
    # Pretty-print: PID and full command.
    # ps aux format: USER PID %CPU %MEM VSZ RSS TTY STAT START TIME COMMAND
    echo "$existing" | while read -r user pid rest; do
        # 'rest' starts with %CPU; we want the command portion at the end.
        # For clarity we just echo the original line prefixed with PID.
        echo "  PID ${pid}: ${user} ${rest}"
    done
    echo ""

    # If non-interactive auto-kill is explicitly enabled, skip prompt.
    if [[ "${CENTRAL_AUTO_KILL:-false}" == "true" && ! -t 0 ]]; then
        echo "CENTRAL_AUTO_KILL=true and non-interactive shell detected — killing matching processes without prompting..."
        echo "$existing" | awk '{print $2}' | xargs -r kill || true
    else
        read -r -p "Kill these processes and continue? [y/N] " reply
        if [[ "$reply" != "y" && "$reply" != "Y" ]]; then
            echo "Aborting without killing any processes."
            exit 1
        fi
        echo "Killing matching processes..."
        echo "$existing" | awk '{print $2}' | xargs -r kill || true
    fi

    # Give the OS a moment, then re-check.
    sleep 1
    local remaining
    remaining="$(ps aux | grep -E "$pattern_tf|$pattern_bridge|$pattern_map_relay|$pattern_explorer" | grep -v grep || true)"
    if [[ -n "$remaining" ]]; then
        echo "WARNING: Some central processes still appear to be running:"
        echo "$remaining"
        echo "You may need to inspect and kill these manually before re-running."
        exit 1
    fi

    echo "All matching central processes have been terminated. Continuing startup..."
    echo ""
}

ensure_no_central_stack_running

# If the graph still shows these nodes, warn but continue.
NODES_RAW="$(ros2 node list 2>/dev/null || true)"
if [[ -n "$NODES_RAW" ]]; then
    explorer_count="$(echo "$NODES_RAW" | grep -c '^/multi_robot_explorer$' || true)"
    tf_relay_count="$(echo "$NODES_RAW" | grep -c '^/tf_relay_multirobot$' || true)"
    if (( explorer_count > 0 || tf_relay_count > 0 )); then
        echo "WARNING: ROS graph currently includes central node names already:"
        echo "  /multi_robot_explorer count = ${explorer_count}"
        echo "  /tf_relay_multirobot count  = ${tf_relay_count}"
        echo "Continuing because no matching OS processes were found."
        echo ""
    fi
fi

# Detect robot names from available topics (e.g., /blinky/tf, /pinky/tf).
# Important: transient discovery lag can momentarily hide one robot and
# accidentally push the stack into single-robot mode. Wait briefly for the
# expected count before deciding mode.
echo "Detecting robots from ROS topics..."

CENTRAL_DISCOVERY_TIMEOUT_SEC="${CENTRAL_DISCOVERY_TIMEOUT_SEC:-5}"
CENTRAL_DISCOVERY_POLL_SEC="${CENTRAL_DISCOVERY_POLL_SEC:-1}"
CENTRAL_MIN_ROBOTS_DEFAULT="${CENTRAL_MIN_ROBOTS_DEFAULT:-2}"
if ! [[ "$CENTRAL_DISCOVERY_TIMEOUT_SEC" =~ ^[0-9]+$ ]]; then
    CENTRAL_DISCOVERY_TIMEOUT_SEC=20
fi
if ! [[ "$CENTRAL_DISCOVERY_POLL_SEC" =~ ^[0-9]+$ ]]; then
    CENTRAL_DISCOVERY_POLL_SEC=1
fi
if (( CENTRAL_DISCOVERY_POLL_SEC < 1 )); then
    CENTRAL_DISCOVERY_POLL_SEC=1
fi

detect_robots_once() {
    local topics_raw="$1"
    local detected=()
    if [[ -n "$topics_raw" ]]; then
        while IFS= read -r line; do
            name="${line#/}"
            name="${name%%/*}"
            if [[ -n "$name" ]]; then
                detected+=("$name")
            fi
        done < <(echo "$topics_raw" | grep '^/[^/]\+/tf$' || true)
    fi

    if ((${#detected[@]} == 0)); then
        if [[ -n "$topics_raw" ]]; then
            while IFS= read -r line; do
                name="${line#/}"
                name="${name%%/*}"
                if [[ -n "$name" ]]; then
                    detected+=("$name")
                fi
            done < <(echo "$topics_raw" | grep '^/[^/]\+/map$' || true)
        fi
    fi

    # De-duplicate + sort for stable ordering.
    local uniq=()
    for r in "${detected[@]}"; do
        local skip=false
        for u in "${uniq[@]}"; do
            if [[ "$u" == "$r" ]]; then
                skip=true
                break
            fi
        done
        if [[ "$skip" == false ]]; then
            uniq+=("$r")
        fi
    done
    if ((${#uniq[@]} > 0)); then
        printf '%s\n' "${uniq[@]}" | sort
    fi
}

# Determine how many robots should be visible before finalizing mode.
if [[ -n "$SELECTION" ]]; then
    # Count unique selection letters (b/p/i/c).
    declare -A _sel_letters=()
    for ((i=0; i<${#SELECTION}; i++)); do
        ch="${SELECTION:$i:1}"
        _sel_letters["$ch"]=1
    done
    MIN_EXPECTED_ROBOTS="${#_sel_letters[@]}"
else
    MIN_EXPECTED_ROBOTS="${CENTRAL_MIN_ROBOTS_DEFAULT}"
fi
if ! [[ "$MIN_EXPECTED_ROBOTS" =~ ^[0-9]+$ ]]; then
    MIN_EXPECTED_ROBOTS=1
fi
if (( MIN_EXPECTED_ROBOTS < 1 )); then
    MIN_EXPECTED_ROBOTS=1
fi

DETECTED_ROBOTS=()
elapsed=0
while :; do
    TOPICS_RAW="$(ros2 topic list 2>/dev/null || true)"
    readarray -t DETECTED_ROBOTS < <(detect_robots_once "$TOPICS_RAW")

    if ((${#DETECTED_ROBOTS[@]} >= MIN_EXPECTED_ROBOTS)); then
        break
    fi

    if (( elapsed >= CENTRAL_DISCOVERY_TIMEOUT_SEC )); then
        break
    fi

    if (( elapsed == 0 )); then
        if [[ -n "$SELECTION" ]]; then
            echo "Waiting up to ${CENTRAL_DISCOVERY_TIMEOUT_SEC}s for selected robots (-${SELECTION}) to appear..."
        else
            echo "Waiting up to ${CENTRAL_DISCOVERY_TIMEOUT_SEC}s for >=${MIN_EXPECTED_ROBOTS} robots to appear..."
        fi
    fi
    sleep "$CENTRAL_DISCOVERY_POLL_SEC"
    elapsed=$((elapsed + CENTRAL_DISCOVERY_POLL_SEC))
done

if ((${#DETECTED_ROBOTS[@]} == 0)); then
    echo "ERROR: Could not detect any robot namespaces from topics."
    echo "Make sure at least one robot is running its namespaced bringup + Nav2/SLAM."
    exit 1
fi

# Apply selection filter (map letters to robot name first char).
SELECTED_ROBOTS=()
for r in "${DETECTED_ROBOTS[@]}"; do
    if [[ -z "$SELECTION" ]]; then
        SELECTED_ROBOTS+=("$r")
    else
        first_char="${r:0:1}"
        if [[ "$SELECTION" == *"$first_char"* ]]; then
            SELECTED_ROBOTS+=("$r")
        fi
    fi
done

if ((${#SELECTED_ROBOTS[@]} == 0)); then
    echo "ERROR: No robots matched selection '-$SELECTION'."
    echo "Detected robots: ${DETECTED_ROBOTS[*]}"
    exit 1
fi

echo "Detected robots: ${DETECTED_ROBOTS[*]}"
echo "Using robots   : ${SELECTED_ROBOTS[*]}"
echo ""

if ((${#SELECTED_ROBOTS[@]} == 1)); then
    MODE_DESC="single-robot (Nav2 on robot, no map_merge)"
else
    MODE_DESC="multi-robot (Nav2 on robots, map_merge enabled)"
fi
echo "  Mode          = ${MODE_DESC}"
echo ""

# Build a ROS 2 list parameter value like "[blinky,pinky]".
ROBOT_LIST_PARAM=""
for r in "${SELECTED_ROBOTS[@]}"; do
    if [[ -n "$ROBOT_LIST_PARAM" ]]; then
        ROBOT_LIST_PARAM+=","
    fi
    ROBOT_LIST_PARAM+="$r"
done

PIDS=()
CLEANUP_DONE=false
cleanup() {
    if [[ "$CLEANUP_DONE" == "true" ]]; then
        return
    fi
    CLEANUP_DONE=true
    echo ""
    echo "Shutting down..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    wait 2>/dev/null
    echo "Done."
}
trap cleanup SIGHUP SIGINT SIGTERM EXIT

# ---- 1. TF relay (same policy for 1 or N robots) ----
echo "[1/3] Starting TF relay (prefix_frames=true)..."
python3 "${TF_RELAY_SCRIPT}" --ros-args \
    -p "robot_prefixes:=[${ROBOT_LIST_PARAM}]" \
    -p "prefix_frames:=true" &
PIDS+=($!)
sleep 1

if ((${#SELECTED_ROBOTS[@]} == 1)); then
    SINGLE_ROBOT_BRIDGE="${SELECTED_ROBOTS[0]}"
    echo "[1b/3] Single-robot mode — static TF map -> ${SINGLE_ROBOT_BRIDGE}/map (identity)"
    python3 "${SINGLE_ROBOT_WORLD_TF_SCRIPT}" --ros-args \
        -p "robot_name:=${SINGLE_ROBOT_BRIDGE}" &
    PIDS+=($!)
    sleep 0.5
    echo "[1c/3] Single-robot mode — relay /${SINGLE_ROBOT_BRIDGE}/map_wire_z -> /map (zlib wire)"
    python3 "${SINGLE_ROBOT_MAP_RELAY_SCRIPT}" --ros-args \
        -p "source_map:=/${SINGLE_ROBOT_BRIDGE}/map" \
        -p "source_map_wire_z:=/${SINGLE_ROBOT_BRIDGE}/map_wire_z" &
    PIDS+=($!)
    sleep 0.5
fi

if ((${#SELECTED_ROBOTS[@]} == 1)); then
    # ------------------------------------------------------------------
    # Single-robot setup:
    #   - Skip map_merge.
    #   - Treat the robot's /map in frame 'map' as the world map
    #     (world_frame='map').
    # ------------------------------------------------------------------
    SINGLE_ROBOT="${SELECTED_ROBOTS[0]}"
    echo "[2/3] Single-robot setup detected (${SINGLE_ROBOT}) — skipping map merge."
    echo "[3/3] Starting single-robot explorer (Nav2 offloaded to robot)..."
    EXPLORER_ARGS=(
        "${EXPLORER_SCRIPT}"
        --ros-args
        --params-file "${EXPLORE_CONFIG_FILE}"
        -p "robot_names:=[${SINGLE_ROBOT}]"
        -p "map_topic:=/${SINGLE_ROBOT}/map"
        -p "world_frame:=map"
        -p "single_robot_offloaded_nav2:=true"
        -p "use_pose_goal_fallback:=${EXPLORER_USE_GOAL_POSE_FALLBACK}"
    )
    if [[ -n "$EXPLORER_FREQUENCY_HZ" ]]; then
        EXPLORER_ARGS+=(-p "explore_frequency:=${EXPLORER_FREQUENCY_HZ}")
    fi
    python3 "${EXPLORER_ARGS[@]}" &
    PIDS+=($!)
else
    # ---- 2. Map merge ----
    echo "[2/3] Starting map merge (unknown poses)..."
    ros2 run multirobot_map_merge map_merge --ros-args \
        --params-file "${MAP_MERGE_CONFIG_FILE}" &
    PIDS+=($!)
    sleep 2
    echo ""
    echo "  map_merge recovery: if logs show 'Grid pose estimation disabled permanently'"
    echo "  after an OpenCV/FLANN (miniflann) exception, restart this script or the"
    echo "  map_merge process; relative pose refinement stays off until then."
    echo ""

    # ---- 2b. Map merge state monitor ----
    # This helper publishes a coarse merge state string (NO_OVERLAP, PARTIAL,
    # MERGED) on map_merge/merge_state so the explorer can automatically switch
    # from per-robot local exploration to global merged exploration.
    python3 "${SCRIPT_DIR}/map_merge_state_monitor.py" --ros-args \
        -p "robot_names:=[${ROBOT_LIST_PARAM}]" \
        -p "world_frame:=map" \
        -p "stability_pos_threshold:=0.28" \
        -p "merged_dwell_time:=12.0" \
        -p "motion_jitter_confirm_ticks:=3" &
    PIDS+=($!)

    # ---- 3. Explorer ----
    echo "[3/3] Starting multi-robot explorer..."
    EXPLORER_ARGS=(
        "${EXPLORER_SCRIPT}"
        --ros-args
        --params-file "${EXPLORE_CONFIG_FILE}"
        -p "robot_names:=[${ROBOT_LIST_PARAM}]"
        -p "use_pose_goal_fallback:=${EXPLORER_USE_GOAL_POSE_FALLBACK}"
    )
    if [[ -n "$EXPLORER_FREQUENCY_HZ" ]]; then
        EXPLORER_ARGS+=(-p "explore_frequency:=${EXPLORER_FREQUENCY_HZ}")
    fi
    python3 "${EXPLORER_ARGS[@]}" &
    PIDS+=($!)
fi

echo ""
echo "=========================================="
echo "  All services running.  Press Ctrl+C to stop."
echo "=========================================="
echo ""
echo "  To visualise: rviz2  (add /map display, set frame to 'map')"
echo ""

wait
