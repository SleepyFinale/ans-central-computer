#!/bin/bash
# Rebuild the central workspace from scratch.
#
#   bash scripts/build/rebuild_common.sh
#
# The central PC builds one package, multirobot_map_merge. The explorer and
# the other central nodes are Python scripts that run from src/ and scripts/.
# Removes build/ install/ log/ first. The old `clean` / `minimal` argument
# is accepted and ignored; both did this.
#
#   PARALLEL_JOBS=<n>          colcon workers (default: half the cores, max 4)
#   REBUILD_LOG_PROFILE=verbose  stream compiler output to the terminal

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "$WS_DIR"

case "${1:-}" in
    ""|clean|minimal) ;;
    *)
        echo "Usage: $0"
        exit 1
        ;;
esac

# Drop this workspace's old install/ from the prefix paths before rebuilding.
for var in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH; do
    if [[ -n "${!var:-}" ]]; then
        export "$var"="$(tr ':' '\n' <<<"${!var}" | grep -v "^${WS_DIR}/install" | paste -sd: -)"
    fi
done
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

if ! ros2 pkg prefix nav2_msgs >/dev/null 2>&1; then
    echo "[ERROR] Navigation2 is not installed: sudo apt install ros-humble-navigation2"
    exit 1
fi

if [[ -z "${PARALLEL_JOBS:-}" ]]; then
    PARALLEL_JOBS=$(( $(nproc) / 2 ))
    (( PARALLEL_JOBS < 1 )) && PARALLEL_JOBS=1
    (( PARALLEL_JOBS > 4 )) && PARALLEL_JOBS=4
fi

echo "[INFO] Removing build/ install/ log/"
rm -rf build/ install/ log/
mkdir -p log/

BUILD_LOG_FILE="log/rebuild_colcon_$(date +%Y%m%d_%H%M%S).log"
COLCON_ARGS=(build --symlink-install --parallel-workers "$PARALLEL_JOBS"
             --packages-select multirobot_map_merge
             --cmake-args -DBUILD_TESTING=OFF)
START_TS="$(date +%s)"
echo "[INFO] Building multirobot_map_merge (log: ${BUILD_LOG_FILE})"
if [[ "${REBUILD_LOG_PROFILE:-}" == "verbose" ]]; then
    colcon "${COLCON_ARGS[@]}" 2>&1 | tee "$BUILD_LOG_FILE"
else
    if ! colcon "${COLCON_ARGS[@]}" >"$BUILD_LOG_FILE" 2>&1; then
        echo "[ERROR] Build failed. Last 80 lines of ${BUILD_LOG_FILE}:" >&2
        tail -n 80 "$BUILD_LOG_FILE" >&2
        exit 1
    fi
fi

echo "[OK] Rebuild complete ($(( $(date +%s) - START_TS ))s)"
echo "Next: ./scripts/core/start_fleet.sh   (or start_zenoh_central.sh, start_central.sh, start_rviz_central.sh)"
