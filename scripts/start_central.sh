#!/bin/bash
# Start all central-computer processes for multi-robot exploration.
#
# This is the single entry point for the central computer when using the
# single-domain, namespaced multi-robot setup. It starts:
#   1. TF relay        (prefix robot TF frames: odom → blinky/odom, etc.)
#   2. Map merge       (merge individual maps into a global map)
#   3. Explorer        (detect frontiers, send Nav2 action goals)
#
# Prerequisites:
#   - All robots and the central PC use the same ROS_DOMAIN_ID (typically 50)
#   - Each robot is powered on and running:
#       * namespaced bringup       (robot_namespaced.launch.py)
#       * namespaced SLAM + Nav2   (navigation2_slam_namespaced.launch.py)
#   - Central PC and robots are on the same WiFi network
#
# Usage:
#   export ROS_DOMAIN_ID=50
#   ./scripts/start_central.sh
#   ./scripts/start_central.sh -b      # only Blinky
#   ./scripts/start_central.sh -pi     # only Pinky + Inky

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
CONFIG_DIR="${WORKSPACE_DIR}/config"

cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null

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
echo "  Explorer fallback = use_pose_goal_fallback=${EXPLORER_USE_GOAL_POSE_FALLBACK}"
echo ""

# Detect robot names from available topics (e.g., /blinky/tf, /pinky/tf).
echo "Detecting robots from ROS topics..."
TOPICS_RAW="$(ros2 topic list 2>/dev/null || true)"
DETECTED_ROBOTS=()
if [[ -n "$TOPICS_RAW" ]]; then
    while IFS= read -r line; do
        name="${line#/}"
        name="${name%%/*}"
        if [[ -n "$name" ]]; then
            DETECTED_ROBOTS+=("$name")
        fi
    done < <(echo "$TOPICS_RAW" | grep '^/[^/]\+/tf$' || true)
fi

if ((${#DETECTED_ROBOTS[@]} == 0)); then
    echo "WARNING: No /<robot>/tf topics detected. Falling back to /<robot>/map detection..."
    if [[ -n "$TOPICS_RAW" ]]; then
        while IFS= read -r line; do
            name="${line#/}"
            name="${name%%/*}"
            if [[ -n "$name" ]]; then
                DETECTED_ROBOTS+=("$name")
            fi
        done < <(echo "$TOPICS_RAW" | grep '^/[^/]\+/map$' || true)
    fi
fi

if ((${#DETECTED_ROBOTS[@]} == 0)); then
    echo "ERROR: Could not detect any robot namespaces from topics."
    echo "Make sure at least one robot is running its namespaced bringup + Nav2/SLAM."
    exit 1
fi

# De-duplicate.
uniq_detected=()
for r in "${DETECTED_ROBOTS[@]}"; do
    skip=false
    for u in "${uniq_detected[@]}"; do
        if [[ "$u" == "$r" ]]; then
            skip=true
            break
        fi
    done
    if [[ "$skip" == false ]]; then
        uniq_detected+=("$r")
    fi
done
DETECTED_ROBOTS=("${uniq_detected[@]}")

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

# Build a ROS 2 list parameter value like "[blinky,pinky]".
ROBOT_LIST_PARAM=""
for r in "${SELECTED_ROBOTS[@]}"; do
    if [[ -n "$ROBOT_LIST_PARAM" ]]; then
        ROBOT_LIST_PARAM+=","
    fi
    ROBOT_LIST_PARAM+="$r"
done

PIDS=()
cleanup() {
    echo ""
    echo "Shutting down..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    wait 2>/dev/null
    echo "Done."
}
trap cleanup SIGINT SIGTERM EXIT

# ---- 1. TF relay ----
echo "[1/3] Starting TF relay (prefix robot frames)..."
python3 "${SCRIPT_DIR}/tf_relay_multirobot.py" --ros-args \
    -p "robot_prefixes:=[${ROBOT_LIST_PARAM}]" &
PIDS+=($!)
sleep 1

if ((${#SELECTED_ROBOTS[@]} == 1)); then
    # ------------------------------------------------------------------
    # Single-robot mode: use the robot's own map frame as world frame
    # and skip map_merge entirely.
    # ------------------------------------------------------------------
    SINGLE_ROBOT="${SELECTED_ROBOTS[0]}"
    echo "[2/3] Single-robot setup detected (${SINGLE_ROBOT}) — skipping map merge."
    echo "[3/3] Starting multi-robot explorer in single-robot mode..."
    # NOTE: For rclpy nodes, parameters passed via CLI should be the raw parameter
    # names (robot_names, map_topic, world_frame, ...) — not prefixed by node name.
    python3 "${SCRIPT_DIR}/multi_robot_explorer.py" --ros-args \
        --params-file "${CONFIG_DIR}/multi_robot_explorer.yaml" \
        -p "robot_names:=[${ROBOT_LIST_PARAM}]" \
        -p "map_topic:=/map" \
        -p "world_frame:=${SINGLE_ROBOT}/map" \
        -p "use_pose_goal_fallback:=${EXPLORER_USE_GOAL_POSE_FALLBACK}" &
    PIDS+=($!)
else
    # ---- 2. Map merge ----
    echo "[2/3] Starting map merge (unknown poses)..."
    ros2 run multirobot_map_merge map_merge --ros-args \
        --params-file "${CONFIG_DIR}/map_merge/multirobot_params_unknown_poses.yaml" &
    PIDS+=($!)
    sleep 2

    # ---- 3. Explorer ----
    echo "[3/3] Starting multi-robot explorer..."
    python3 "${SCRIPT_DIR}/multi_robot_explorer.py" --ros-args \
        --params-file "${CONFIG_DIR}/multi_robot_explorer.yaml" \
        -p "robot_names:=[${ROBOT_LIST_PARAM}]" \
        -p "use_pose_goal_fallback:=${EXPLORER_USE_GOAL_POSE_FALLBACK}" &
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
