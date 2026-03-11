#!/bin/bash
# Clean rebuild script for turtlebot3_ws
# This will remove build artifacts and rebuild everything from scratch

set -e  # Exit on error

echo "=========================================="
echo "Clean Rebuild Script for turtlebot3_ws"
echo "=========================================="
echo ""

# Resolve script/workspace directories and ensure we're in the workspace root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
cd "$WS_DIR"

# Check if we're in the right directory
if [ ! -d "src" ]; then
    echo "Error: Must run from workspace root (turtlebot3_ws/)"
    exit 1
fi

# Clean up any stale workspace paths from environment variables
# This prevents warnings about non-existent paths in AMENT_PREFIX_PATH and CMAKE_PREFIX_PATH
WORKSPACE_ROOT="$(pwd)"
if [ -n "$AMENT_PREFIX_PATH" ]; then
    # Remove workspace paths from AMENT_PREFIX_PATH
    export AMENT_PREFIX_PATH=$(echo "$AMENT_PREFIX_PATH" | tr ':' '\n' | grep -v "^${WORKSPACE_ROOT}/install" | tr '\n' ':' | sed 's/:$//')
fi
if [ -n "$CMAKE_PREFIX_PATH" ]; then
    # Remove workspace paths from CMAKE_PREFIX_PATH
    export CMAKE_PREFIX_PATH=$(echo "$CMAKE_PREFIX_PATH" | tr ':' '\n' | grep -v "^${WORKSPACE_ROOT}/install" | tr '\n' ':' | sed 's/:$//')
fi

# Source base ROS environment (overlay is sourced after a successful build)
echo "Step 1: Sourcing base ROS Humble environment..."
source /opt/ros/humble/setup.bash

# Clean build artifacts
echo ""
echo "Step 2: Cleaning build and install directories..."
echo "  - Removing build/ directory..."
rm -rf build/
echo "  - Removing install/ directory..."
rm -rf install/
echo "  - Removing log/ directory..."
rm -rf log/
echo "  ✓ Cleanup complete"

# Verify system dependencies (placeholder block for future apt checks)
echo ""
echo "Step 3: Checking system dependencies..."
MISSING_DEPS=()

REQUIRED_DEPS=(
    # Example format:
    # "ros-humble-navigation2:Navigation2 (Nav2) stack"
    # "ros-humble-slam-toolbox:SLAM Toolbox"
)

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

# Ensure Navigation2 system packages are installed (needed for nav2_msgs / multi_robot_explorer.py)
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
    echo "    ./scripts/clean_rebuild.sh"
    echo ""
    exit 1
else
    echo "  ✓ Navigation2 packages found (nav2_msgs present)"
fi

# Build the workspace
echo ""
echo "Step 5: Building workspace (this may take a while)..."
echo "  - Using colcon build with symlink-install"
echo "  - This will build all packages in dependency order"
echo ""

# Limit parallel jobs to prevent memory exhaustion
# Use half of available CPU cores, but at least 1 and at most 4
# User can override with: PARALLEL_JOBS=N ./scripts/clean_rebuild.sh
if [ -z "$PARALLEL_JOBS" ]; then
    NUM_CORES=$(nproc)
    PARALLEL_JOBS=$((NUM_CORES / 2))
    if [ $PARALLEL_JOBS -lt 1 ]; then
        PARALLEL_JOBS=1
    elif [ $PARALLEL_JOBS -gt 4 ]; then
        PARALLEL_JOBS=4
    fi
fi

# Check available memory (warn if less than 4GB free)
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
echo "  - To override: PARALLEL_JOBS=N ./scripts/clean_rebuild.sh"
echo ""

colcon build --symlink-install --parallel-workers $PARALLEL_JOBS \
    --cmake-args -DBUILD_TESTING=OFF

# Check build result
if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "✓ Build completed successfully!"
    echo "=========================================="
    echo ""
    echo "Step 6: Sourcing workspace..."
    # Re-source ROS and then the freshly built workspace so this shell is ready
    WS_DIR="$WS_DIR" source "${SCRIPT_DIR}/ros_robot_env.bash"
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
    echo "Workspace is ready to use!"
    echo ""
    echo "You can now run central scripts such as:"
    echo "  - ./scripts/start_central.sh"
    echo "  - ./scripts/start_rviz_central.sh"
    echo ""
else
    echo ""
    echo "=========================================="
    echo "✗ Build failed!"
    echo "=========================================="
    echo ""
    echo "Check the error messages above for details."
    echo "Common issues:"
    echo "  - Missing system dependencies (run ./install_dependencies.sh)"
    echo "  - CMake configuration errors"
    echo "  - Source code issues"
    exit 1
fi
