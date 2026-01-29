#!/bin/bash
# Clean rebuild script for turtlebot3_ws
# This will remove build artifacts and rebuild everything from scratch

set -e  # Exit on error

echo "=========================================="
echo "Clean Rebuild Script for turtlebot3_ws"
echo "=========================================="
echo ""

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

# Source ROS environment
echo "Step 1: Sourcing ROS Humble environment..."
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

# Verify system dependencies
echo ""
echo "Step 3: Checking system dependencies..."
MISSING_DEPS=()

# Check for required dependencies
# Note: Navigation2 packages are ignored (using system packages), so their dependencies are not needed
REQUIRED_DEPS=(
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
    echo "  Please run the dependency installer first:"
    echo "    ./install_dependencies.sh"
    echo ""
    echo "  Or install manually:"
    echo "    sudo apt install -y ${MISSING_DEPS[*]}"
    echo ""
    read -p "  Continue anyway? (build will likely fail) (y/n): " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "  Exiting. Please install dependencies first."
        exit 1
    fi
    echo "  ⚠ Continuing without dependencies (build may fail)"
else
    echo "  ✓ All dependencies found"
fi

# Build the workspace
echo ""
echo "Step 4: Building workspace (this may take a while)..."
echo "  - Using colcon build with symlink-install"
echo "  - This will build all packages in dependency order"
echo "  - turtlebot3_navigation2 and explore_lite will be built"
echo ""

# Limit parallel jobs to prevent memory exhaustion
# Use half of available CPU cores, but at least 1 and at most 4
# User can override with: PARALLEL_JOBS=N ./clean_rebuild.sh
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
echo "  - To override: PARALLEL_JOBS=N ./clean_rebuild.sh"
echo ""

# List of Navigation2 packages to ignore (use system packages instead)
# Only include packages that actually exist in the workspace to avoid warnings
NAV2_IGNORE_PACKAGES="nav2_common nav_2d_msgs dwb_msgs nav2_msgs nav2_voxel_grid nav2_simple_commander nav2_util nav2_amcl nav2_behavior_tree nav2_lifecycle_manager nav2_map_server nav2_velocity_smoother nav_2d_utils nav2_costmap_2d costmap_queue nav2_collision_monitor nav2_core dwb_core nav2_behaviors nav2_bt_navigator nav2_constrained_smoother nav2_controller nav2_mppi_controller nav2_navfn_planner nav2_planner nav2_regulated_pure_pursuit_controller nav2_route nav2_smac_planner nav2_smoother nav2_theta_star_planner nav2_waypoint_follower dwb_critics dwb_plugins nav2_rotation_shim_controller nav2_rviz_plugins nav2_dwb_controller navigation2 nav2_bringup nav2_ros_common opennav_docking_core opennav_docking_bt nav2_graceful_controller opennav_docking opennav_following"

# Check if Navigation2 system packages are installed
echo ""
echo "Step 3.5: Checking for Navigation2 system packages..."
if ! ros2 pkg list 2>/dev/null | grep -q "^nav2_msgs$"; then
    echo "  ✗ Navigation2 packages not found!"
    echo ""
    echo "  This workspace uses system Navigation2 packages (installed via apt)."
    echo "  Please install Navigation2 before building:"
    echo ""
    echo "    sudo apt update"
    echo "    sudo apt install ros-humble-navigation2"
    echo ""
    echo "  After installing, source ROS again and retry:"
    echo "    source /opt/ros/humble/setup.bash"
    echo "    ./clean_rebuild.sh"
    echo ""
    exit 1
else
    echo "  ✓ Navigation2 packages found"
fi

# Filter ignore list to only include packages that actually exist in workspace
# This prevents warnings about unknown packages
if [ -d "src/navigation2" ]; then
    # Get list of all packages that colcon can find, then filter to only Navigation2 packages
    ALL_PACKAGES=$(colcon list --names-only 2>/dev/null || echo "")
    if [ -n "$ALL_PACKAGES" ]; then
        # Filter to only packages that exist in workspace and are in our ignore list
        EXISTING_NAV2_PACKAGES=""
        for pkg in $NAV2_IGNORE_PACKAGES; do
            if echo "$ALL_PACKAGES" | grep -q "^${pkg}$"; then
                EXISTING_NAV2_PACKAGES="${EXISTING_NAV2_PACKAGES} ${pkg}"
            fi
        done
        NAV2_IGNORE_PACKAGES=$(echo $EXISTING_NAV2_PACKAGES | sed 's/^ *//;s/ *$//')
    else
        # If colcon can't list packages, set to empty to avoid warnings
        NAV2_IGNORE_PACKAGES=""
    fi
else
    # Navigation2 not in workspace, so nothing to ignore
    NAV2_IGNORE_PACKAGES=""
fi

# Build all packages, but ignore Navigation2 packages in workspace (use system packages)
if [ -n "$NAV2_IGNORE_PACKAGES" ]; then
    colcon build --symlink-install --parallel-workers $PARALLEL_JOBS \
        --cmake-args -DBUILD_TESTING=OFF \
        --packages-ignore $NAV2_IGNORE_PACKAGES
else
    # If no Navigation2 packages to ignore, just build normally
    # (Navigation2 directory might not exist or might be gitignored)
    colcon build --symlink-install --parallel-workers $PARALLEL_JOBS \
        --cmake-args -DBUILD_TESTING=OFF
fi

# Check build result
if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "✓ Build completed successfully!"
    echo "=========================================="
    echo ""
    echo "Step 5: Sourcing workspace..."
    source install/setup.bash
    echo "  ✓ Workspace sourced"
    echo ""
    echo "Verifying key packages are built..."
    PKG_LIST=$(ros2 pkg list 2>/dev/null)
    if echo "$PKG_LIST" | grep -q "^turtlebot3_navigation2$"; then
        echo "  ✓ turtlebot3_navigation2 is available"
    else
        echo "  ✗ turtlebot3_navigation2 not found"
    fi
    if echo "$PKG_LIST" | grep -q "^explore_lite$"; then
        echo "  ✓ explore_lite is available"
    else
        echo "  ✗ explore_lite not found"
    fi
    echo ""
    echo "Workspace is ready to use!"
    echo ""
    echo "You can now run:"
    echo "  - ros2 launch turtlebot3_navigation2 navigation2.launch.py"
    echo "  - ./start_explorer_simple.sh"
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
