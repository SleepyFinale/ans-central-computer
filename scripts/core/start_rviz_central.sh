#!/usr/bin/env bash
# Start RViz on the central PC.
#
# Usage:
#   ./scripts/core/start_rviz_central.sh              # merged /map in frame map
#   ./scripts/core/start_rviz_central.sh -r pinky     # one robot's map, scan, plan, local costmap
#   ./scripts/core/start_rviz_central.sh --local pinky
#
# With no argument and exactly one robot sending a map, RViz opens that
# robot's view, since there is no merged map to show.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../env/fleet_common.bash
source "${SCRIPT_DIR}/../env/fleet_common.bash"
GLOBAL_CFG="${FLEET_WORKSPACE_DIR}/config/rviz/central_global_map.rviz"
LOCAL_TEMPLATE_CFG="${FLEET_WORKSPACE_DIR}/config/rviz/central_robot_local_map_template.rviz"

ROBOT=""
case "${1:-}" in
    "" | --global) ;;
    --local | -r)
        ROBOT="${2:-}"
        if [[ -z "$ROBOT" ]]; then
            echo "ERROR: $1 needs a robot name, e.g. $1 pinky" >&2
            exit 1
        fi
        ;;
    -h | --help)
        sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'
        exit 0
        ;;
    *)
        echo "ERROR: unknown argument '$1' (try --help)" >&2
        exit 1
        ;;
esac

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
fleet_use_zenoh_dds

if [[ -z "$ROBOT" ]]; then
    readarray -t ACTIVE < <(fleet_robots_with_maps 3 "${FLEET_ROBOTS[@]}")
    echo "Robots sending maps: ${ACTIVE[*]:-none}"
    if ((${#ACTIVE[@]} == 1)); then
        ROBOT="${ACTIVE[0]}"
    fi
fi

if [[ -n "$ROBOT" ]]; then
    CFG="$(mktemp "/tmp/central_rviz_${ROBOT}_XXXX.rviz")"
    sed -e "s/__ROBOT__/${ROBOT}/g" -e "s#Fixed Frame: map#Fixed Frame: ${ROBOT}/map#g" \
        "$LOCAL_TEMPLATE_CFG" > "$CFG"
    echo "Starting RViz for ${ROBOT}: fixed frame ${ROBOT}/map, map /${ROBOT}/map"
else
    CFG="$GLOBAL_CFG"
    echo "Starting RViz for the fleet: fixed frame map, map /map"
fi

# Cyclone's default cap is 10 participants per domain, which two robots'
# worth of central nodes already fill ("Failed to find a free participant
# index"). This file raises the cap and binds 127.0.0.1 itself, so
# ROS_LOCALHOST_ONLY must be unset or Cyclone aborts with
# "the same interface may not be selected twice".
unset ROS_LOCALHOST_ONLY
export CYCLONEDDS_URI="file://${FLEET_WORKSPACE_DIR}/config/cyclonedds/central_localhost.xml"
exec rviz2 -d "$CFG"
