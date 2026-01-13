#!/bin/bash
# Safe Nav2 launcher that waits for TF tree to be ready

set -e

echo "=========================================="
echo "Nav2 Safe Launcher"
echo "=========================================="
echo ""

# Source ROS 2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "ERROR: ROS 2 Humble not found. Please install or source ROS 2."
    exit 1
fi

# Set TurtleBot3 model
if [ -z "$TURTLEBOT3_MODEL" ]; then
    export TURTLEBOT3_MODEL=burger
    echo "Setting TURTLEBOT3_MODEL=burger (default)"
fi

echo "TurtleBot3 Model: $TURTLEBOT3_MODEL"
echo ""

# Get workspace directory
WORKSPACE_DIR="$HOME/turtlebot3_ws"
WAIT_TF_SCRIPT="$WORKSPACE_DIR/wait_for_tf.py"

# Check if wait script exists
if [ ! -f "$WAIT_TF_SCRIPT" ]; then
    echo "WARNING: wait_for_tf.py not found at $WAIT_TF_SCRIPT"
    echo "Proceeding without TF check..."
    sleep 2
else
    echo "Waiting for TF tree to be ready..."
    echo ""
    python3 "$WAIT_TF_SCRIPT"
    
    if [ $? -ne 0 ]; then
        echo ""
        echo "ERROR: TF tree is not ready. Please check:"
        echo "  1. Robot launch is running: ros2 launch turtlebot3_bringup robot.launch.py"
        echo "  2. SLAM Toolbox is running: ros2 launch slam_toolbox online_async_launch.py"
        echo "  3. Wait 10-15 seconds after starting SLAM"
        exit 1
    fi
    
    echo ""
    echo "TF tree is ready! Starting Nav2..."
    echo ""
    sleep 1
fi

# Launch Nav2
ros2 launch turtlebot3_navigation2 navigation2.launch.py "$@"
