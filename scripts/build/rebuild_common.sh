#!/bin/bash
# Shared implementation for central workspace rebuild scripts.

set -eo pipefail

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
    PACKAGES_UP_TO=""
else
    BUILD_LABEL="Minimal Rebuild"
    BUILD_CMD_HINT="./scripts/build/minimal_rebuild.sh"
    PACKAGES_UP_TO="multirobot_map_merge"
fi

REBUILD_LOG_PROFILE="${REBUILD_LOG_PROFILE:-clean}"
if [[ "$REBUILD_LOG_PROFILE" != "clean" && "$REBUILD_LOG_PROFILE" != "verbose" ]]; then
    echo "Invalid REBUILD_LOG_PROFILE='${REBUILD_LOG_PROFILE}'. Use clean or verbose."
    exit 1
fi

log_info() {
    echo "[INFO] $*"
}

log_warn() {
    echo "[WARN] $*"
}

log_error() {
    echo "[ERROR] $*" >&2
}

log_ok() {
    echo "[OK] $*"
}

phase_start() {
    local label="$1"
    CURRENT_PHASE="$label"
    CURRENT_PHASE_START="$(date +%s)"
    log_info "${label}..."
}

phase_done() {
    local now elapsed
    now="$(date +%s)"
    elapsed=$((now - CURRENT_PHASE_START))
    log_ok "${CURRENT_PHASE} (${elapsed}s)"
}

print_build_failure_tail() {
    local log_file="$1"
    local tail_lines="${2:-50}"
    log_error "Build failed. Showing last ${tail_lines} lines from ${log_file}:"
    if [[ -f "$log_file" ]]; then
        tail -n "$tail_lines" "$log_file"
    else
        log_error "Build log file not found."
    fi
}

START_TS="$(date +%s)"
CURRENT_PHASE=""
CURRENT_PHASE_START="$START_TS"

echo "=========================================="
echo "${BUILD_LABEL} (profile: ${REBUILD_LOG_PROFILE})"
echo "=========================================="

cd "$WS_DIR"
if [[ ! -d "src" ]]; then
    log_error "Must run from workspace root (e.g. central-computer/)"
    exit 1
fi

WORKSPACE_ROOT="$(pwd)"
if [[ -n "${AMENT_PREFIX_PATH:-}" ]]; then
    export AMENT_PREFIX_PATH
    AMENT_PREFIX_PATH="$(echo "$AMENT_PREFIX_PATH" | tr ':' '\n' | sed "/^${WORKSPACE_ROOT//\//\\/}\/install/d" | tr '\n' ':' | sed 's/:$//')"
fi
if [[ -n "${CMAKE_PREFIX_PATH:-}" ]]; then
    export CMAKE_PREFIX_PATH
    CMAKE_PREFIX_PATH="$(echo "$CMAKE_PREFIX_PATH" | tr ':' '\n' | sed "/^${WORKSPACE_ROOT//\//\\/}\/install/d" | tr '\n' ':' | sed 's/:$//')"
fi

phase_start "Source ROS base environment"
source /opt/ros/humble/setup.bash
phase_done

phase_start "Clean workspace artifacts"
rm -rf build/ install/ log/
mkdir -p log/
phase_done

phase_start "Check system dependencies"
MISSING_DEPS=()
REQUIRED_DEPS=()
for dep_check in "${REQUIRED_DEPS[@]}"; do
    IFS=':' read -r pkg_name display_name <<< "$dep_check"
    if ! dpkg -l | awk '{print $2}' | grep -x "$pkg_name" >/dev/null 2>&1; then
        MISSING_DEPS+=("$pkg_name")
        [[ "$REBUILD_LOG_PROFILE" == "verbose" ]] && log_warn "Missing: ${display_name} (${pkg_name})"
    elif [[ "$REBUILD_LOG_PROFILE" == "verbose" ]]; then
        log_ok "Found: ${display_name}"
    fi
done
if [[ ${#MISSING_DEPS[@]} -gt 0 ]]; then
    log_warn "Missing dependencies: ${MISSING_DEPS[*]}"
    echo "Install with: sudo apt install -y ${MISSING_DEPS[*]}"
    read -r -p "Continue anyway? (build may fail) (y/n): " -n 1
    echo
    if [[ ! "${REPLY:-}" =~ ^[Yy]$ ]]; then
        log_error "Exiting. Please install dependencies first."
        exit 1
    fi
fi
phase_done

phase_start "Verify Navigation2 packages"
PKG_LIST_NAV2="$(ros2 pkg list 2>/dev/null || echo "")"
if ! echo "$PKG_LIST_NAV2" | grep -x "nav2_msgs" >/dev/null 2>&1; then
    log_error "Navigation2 packages not found."
    echo "Install with:"
    echo "  sudo apt update"
    echo "  sudo apt install ros-humble-navigation2"
    echo "Then retry:"
    echo "  source /opt/ros/humble/setup.bash"
    echo "  ${BUILD_CMD_HINT}"
    exit 1
fi
phase_done

if [[ -z "${PARALLEL_JOBS:-}" ]]; then
    NUM_CORES="$(nproc)"
    PARALLEL_JOBS=$((NUM_CORES / 2))
    if [[ $PARALLEL_JOBS -lt 1 ]]; then
        PARALLEL_JOBS=1
    elif [[ $PARALLEL_JOBS -gt 4 ]]; then
        PARALLEL_JOBS=4
    fi
fi

if command -v free >/dev/null 2>&1; then
    AVAIL_MEM_GB="$(free -g | awk '/^Mem:/ {print $7}')"
    if [[ "$AVAIL_MEM_GB" -lt 4 ]]; then
        log_warn "Only ${AVAIL_MEM_GB}GB free memory detected. Consider PARALLEL_JOBS=2."
    fi
fi

COLCON_ARGS=(build --symlink-install --parallel-workers "$PARALLEL_JOBS" --cmake-args -DBUILD_TESTING=OFF)
if [[ -n "$PACKAGES_UP_TO" ]]; then
    COLCON_ARGS+=(--packages-up-to "$PACKAGES_UP_TO")
fi

BUILD_LOG_FILE="log/rebuild_colcon_$(date +%Y%m%d_%H%M%S).log"
phase_start "Build workspace (mode=${MODE}, workers=${PARALLEL_JOBS})"
if [[ "$REBUILD_LOG_PROFILE" == "verbose" ]]; then
    log_info "Streaming build output to terminal and ${BUILD_LOG_FILE}"
    if ! colcon "${COLCON_ARGS[@]}" 2>&1 | tee "$BUILD_LOG_FILE"; then
        print_build_failure_tail "$BUILD_LOG_FILE" 80
        exit 1
    fi
else
    log_info "Detailed build log: ${BUILD_LOG_FILE}"
    if ! colcon "${COLCON_ARGS[@]}" >"$BUILD_LOG_FILE" 2>&1; then
        print_build_failure_tail "$BUILD_LOG_FILE" 80
        exit 1
    fi
fi
phase_done

phase_start "Source workspace environment"
WS_DIR="$WS_DIR" source "${WS_DIR}/scripts/env/ros_robot_env.bash"
phase_done

phase_start "Verify key packages"
PKG_LIST="$(ros2 pkg list 2>/dev/null || echo "")"
if echo "$PKG_LIST" | grep -x "multirobot_map_merge" >/dev/null 2>&1; then
    log_ok "multirobot_map_merge is available"
else
    log_warn "multirobot_map_merge not found"
fi
phase_done

END_TS="$(date +%s)"
TOTAL_SEC=$((END_TS - START_TS))

echo
echo "=========================================="
log_ok "Rebuild complete (${TOTAL_SEC}s)"
echo "Mode: ${MODE}"
echo "Profile: ${REBUILD_LOG_PROFILE}"
echo "Colcon log: ${WORKSPACE_ROOT}/${BUILD_LOG_FILE}"
if [[ "$MODE" == "minimal" ]]; then
    echo "Workspace is ready for central scripts."
else
    echo "Workspace is ready."
fi
echo "Next commands:"
echo "  - ./scripts/core/start_central.sh"
echo "  - ./scripts/core/start_rviz_central.sh"
echo "=========================================="
