#!/bin/bash
set -eo pipefail

# Copy-ready helper for central-computer.
# Records central-side exploration/correlation topics for one session.
#
# Usage:
#   ./scripts/start_central_debug_bag.sh pinky blinky
#   ./scripts/start_central_debug_bag.sh   # auto-detect from /<robot>/tf

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

cd "$WORKSPACE_DIR"
# Some ROS setup scripts may reference unset variables (for example
# AMENT_TRACE_SETUP_FILES), so avoid nounset during sourcing.
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true
set -u

SESSION_TS="$(date +%Y%m%d-%H%M%S)"
export DEBUG_SESSION_ID="${DEBUG_SESSION_ID:-$SESSION_TS}"
OUT_ROOT="${HOME}/.ros/nav2_debug_central"
mkdir -p "$OUT_ROOT"
OUT_DIR="${OUT_ROOT}/bag-${DEBUG_SESSION_ID}"
if [[ -e "$OUT_DIR" ]]; then
  i=1
  while [[ -e "${OUT_DIR}-${i}" ]]; do
    ((i++))
  done
  OUT_DIR="${OUT_DIR}-${i}"
fi

ROBOTS=("$@")
if [[ ${#ROBOTS[@]} -eq 0 ]]; then
  mapfile -t ROBOTS < <(ros2 topic list 2>/dev/null \
    | grep -E '^/[^/]+/tf$' \
    | sed -E 's#^/([^/]+)/tf$#\1#' \
    | sort -u)
fi

if [[ ${#ROBOTS[@]} -eq 0 ]]; then
  echo "No robots detected. Pass names explicitly: ./scripts/start_central_debug_bag.sh pinky blinky"
  exit 1
fi

TOPICS=(
  "/map"
  "/tf"
  "/tf_static"
  "/explore/frontiers"
  "/map_merge/merge_state"
)

for r in "${ROBOTS[@]}"; do
  TOPICS+=(
    "/${r}/map"
    "/${r}/plan"
    "/${r}/cmd_vel_nav"
    "/${r}/navigate_to_pose/_action/status"
    "/${r}/navigate_to_pose/_action/feedback"
    "/${r}/navigate_to_pose/_action/result"
  )
done

echo "=========================================="
echo " Central Debug Bag Capture"
echo "=========================================="
echo " Session ID : ${DEBUG_SESSION_ID}"
echo " Robots     : ${ROBOTS[*]}"
echo " Output dir : ${OUT_DIR}"
echo " Ctrl+C to stop."
echo "=========================================="

ros2 bag record --include-hidden-topics -o "$OUT_DIR" "${TOPICS[@]}"
