#!/bin/bash
# Shared implementation for central workspace rebuild scripts.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

MODE="${1:-}"
if [[ "$MODE" != "clean" && "$MODE" != "minimal" ]]; then
    echo "Usage: $0 <clean|minimal>"
    exit 1
fi

if [[ "$MODE" == "clean" ]]; then
    BUILD_LABEL="Clean Rebuild"
    BUILD_CMD_HINT="./scripts/build/clean_rebuild.sh"
    BUILD_STEP_MSG="Step 5: Building workspace (this may take a while)..."
    BUILD_DESC_1="  - Using colcon build with symlink-install"
    BUILD_DESC_2="  - This will build all packages in dependency order"
    PACKAGES_UP_TO=""
else
    BUILD_LABEL="Minimal Rebuild"
    BUILD_CMD_HINT="./scripts/build/minimal_rebuild.sh"
    BUILD_STEP_MSG="Step 5: Building minimal central package set..."
    BUILD_DESC_1="  Building packages needed for:"
    BUILD_DESC_2="    - ./scripts/core/start_central.sh (TF relay, map merge, explorer)"
    BUILD_DESC_3="    - ./scripts/core/start_rviz_central.sh (RViz configs)"
    PACKAGES_UP_TO="multirobot_map_merge"
fi

echo "=========================================="
echo "${BUILD_LABEL} Script for central workspace"
echo "=========================================="
echo ""

cd "$WS_DIR"
if [ ! -d "src" ]; then
    echo "Error: Must run from workspace root (e.g. central-computer/)"
    exit 1
fi

WORKSPACE_ROOT="$(pwd)"
if [ -n "$AMENT_PREFIX_PATH" ]; then
    export AMENT_PREFIX_PATH=$(echo "$AMENT_PREFIX_PATH" | tr ':' '\n' | grep -v "^${WORKSPACE_ROOT}/install" | tr '\n' ':' | sed 's/:$//')
fi
if [ -n "$CMAKE_PREFIX_PATH" ]; then
    export CMAKE_PREFIX_PATH=$(echo "$CMAKE_PREFIX_PATH" | tr ':' '\n' | grep -v "^${WORKSPACE_ROOT}/install" | tr '\n' ':' | sed 's/:$//')
fi

echo "Step 1: Sourcing base ROS Humble environment..."
source /opt/ros/humble/setup.bash

echo ""
echo "Step 2: Cleaning build and install directories..."
echo "  - Removing build/ directory..."
rm -rf build/
echo "  - Removing install/ directory..."
rm -rf install/
echo "  - Removing log/ directory..."
rm -rf log/
echo "  ✓ Cleanup complete"

echo ""
echo "Step 3: Checking system dependencies..."
MISSING_DEPS=()
REQUIRED_DEPS=()
for dep_check in "${REQUIRED_DEPS[@]}"; do
    IFS=':' read -r pkg_name display_name <<< "$dep_check"
    if ! dpkg -l | grep -q "$pkg_name"; then
        MISSING_DEPS+=("$pkg_name")
        echo "  ✗ Missing: $display_name ($pkg_name)"
    else
        echo "  ✓ Found: $display_name"
    fi
done
if [ ${#MISSING_DEPS[@]} -gt 0 ]; then
    echo ""
    echo "  ⚠ Missing dependencies detected!"
    echo ""
    echo "  Please install them with:"
    echo "    sudo apt install -y ${MISSING_DEPS[*]}"
    echo ""
    read -p "  Continue anyway? (build may fail) (y/n): " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "  Exiting. Please install dependencies first."
        exit 1
    fi
    echo "  ⚠ Continuing without all dependencies (build may fail)"
else
    echo "  ✓ All listed dependencies found"
fi

echo ""
echo "Step 4: Verifying Navigation2 system packages..."
PKG_LIST_NAV2=$(ros2 pkg list 2>/dev/null || echo "")
if ! echo "$PKG_LIST_NAV2" | grep -q "^nav2_msgs$"; then
    echo "  ✗ Navigation2 packages not found!"
    echo ""
    echo "  This workspace uses system Navigation2 packages (installed via apt)"
    echo "  so that the multi-robot explorer can use nav2_msgs NavigateToPose."
    echo ""
    echo "  Please install Navigation2 before building:"
    echo ""
    echo "    sudo apt update"
    echo "    sudo apt install ros-humble-navigation2"
    echo ""
    echo "  After installing, source ROS again and retry:"
    echo "    source /opt/ros/humble/setup.bash"
    echo "    ${BUILD_CMD_HINT}"
    echo ""
    exit 1
else
    echo "  ✓ Navigation2 packages found (nav2_msgs present)"
fi

echo ""
echo "${BUILD_STEP_MSG}"
echo "${BUILD_DESC_1}"
echo "${BUILD_DESC_2}"
if [[ -n "${BUILD_DESC_3:-}" ]]; then
    echo "${BUILD_DESC_3}"
fi
if [[ "$MODE" == "minimal" ]]; then
    echo ""
    echo "  Using --packages-up-to to include dependencies for multirobot_map_merge"
fi
echo ""

if [ -z "$PARALLEL_JOBS" ]; then
    NUM_CORES=$(nproc)
    PARALLEL_JOBS=$((NUM_CORES / 2))
    if [ $PARALLEL_JOBS -lt 1 ]; then
        PARALLEL_JOBS=1
    elif [ $PARALLEL_JOBS -gt 4 ]; then
        PARALLEL_JOBS=4
    fi
fi

if command -v free >/dev/null 2>&1; then
    AVAIL_MEM_GB=$(free -g | awk '/^Mem:/ {print $7}')
    if [ "$AVAIL_MEM_GB" -lt 4 ]; then
        echo "  ⚠ Warning: Only ${AVAIL_MEM_GB}GB free memory detected"
        echo "  ⚠ Consider closing other applications or reducing parallel jobs"
        echo "  ⚠ You can set PARALLEL_JOBS=2 to use fewer workers"
        echo ""
    fi
fi

echo "  - Building with $PARALLEL_JOBS parallel workers (to prevent memory issues)"
echo "  - To override: PARALLEL_JOBS=N ${BUILD_CMD_HINT}"
echo ""

COLCON_ARGS=(build --symlink-install --parallel-workers "$PARALLEL_JOBS" --cmake-args -DBUILD_TESTING=OFF)
if [[ -n "$PACKAGES_UP_TO" ]]; then
    COLCON_ARGS+=(--packages-up-to "$PACKAGES_UP_TO")
fi
colcon "${COLCON_ARGS[@]}"

echo ""
echo "=========================================="
echo "✓ Build completed successfully!"
echo "=========================================="
echo ""
echo "Step 6: Sourcing workspace..."
WS_DIR="$WS_DIR" source "${WS_DIR}/scripts/env/ros_robot_env.bash"
echo "  ✓ Workspace sourced via ros_robot_env.bash"
echo ""
echo "Verifying key packages are available..."
PKG_LIST=$(ros2 pkg list 2>/dev/null || echo "")
if echo "$PKG_LIST" | grep -q "^multirobot_map_merge$"; then
    echo "  ✓ multirobot_map_merge is available"
else
    echo "  ✗ multirobot_map_merge not found"
fi
echo ""
if [[ "$MODE" == "minimal" ]]; then
    echo "Workspace is ready to use for central scripts!"
else
    echo "Workspace is ready to use!"
fi
echo ""
echo "You can now run:"
echo "  - ./scripts/core/start_central.sh"
echo "  - ./scripts/core/start_rviz_central.sh"
