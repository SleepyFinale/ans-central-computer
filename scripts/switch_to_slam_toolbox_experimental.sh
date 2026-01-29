#!/bin/bash
# Script to switch m-explore-ros2 to the experimental slam_toolbox branch
# for multi-robot map merging support

set -e  # Exit on error

echo "=========================================="
echo "Switching to slam_toolbox Experimental Branch"
echo "=========================================="
echo ""
echo "This script will:"
echo "  1. Add upstream remote (robo-friends/m-explore-ros2)"
echo "  2. Fetch the experimental branch"
echo "  3. Switch to feature/slam_toolbox_compat branch"
echo "  4. Rebuild multirobot_map_merge package"
echo ""
read -p "Continue? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Aborted."
    exit 1
fi

cd ~/turtlebot3_ws/src/m-explore-ros2

# Check current branch
CURRENT_BRANCH=$(git branch --show-current)
echo "Current branch: $CURRENT_BRANCH"
echo ""

# Add upstream remote if it doesn't exist
if ! git remote | grep -q "^upstream$"; then
    echo "Adding upstream remote..."
    git remote add upstream https://github.com/robo-friends/m-explore-ros2.git
    echo "✓ Upstream remote added"
else
    echo "✓ Upstream remote already exists"
fi

# Fetch upstream branches
echo "Fetching upstream branches..."
git fetch upstream
echo "✓ Fetched upstream branches"
echo ""

# Check if we have uncommitted changes
if ! git diff-index --quiet HEAD --; then
    echo "⚠ WARNING: You have uncommitted changes!"
    echo "Please commit or stash them before switching branches."
    echo ""
    read -p "Stash changes and continue? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        git stash push -m "Stashed before switching to slam_toolbox_compat branch"
        echo "✓ Changes stashed"
    else
        echo "Aborted. Please handle your changes first."
        exit 1
    fi
fi

# Switch to experimental branch
echo "Switching to feature/slam_toolbox_compat branch..."
if git show-ref --verify --quiet refs/heads/feature/slam_toolbox_compat; then
    # Branch exists locally, just checkout
    git checkout feature/slam_toolbox_compat
    git pull upstream feature/slam_toolbox_compat || true
else
    # Create new branch tracking upstream
    git checkout -b feature/slam_toolbox_compat upstream/feature/slam_toolbox_compat
fi
echo "✓ Switched to feature/slam_toolbox_compat branch"
echo ""

# Show current status
echo "Current branch: $(git branch --show-current)"
echo "Latest commit:"
git log -1 --oneline
echo ""

# Rebuild the package
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash

echo "Rebuilding multirobot_map_merge package..."
colcon build --packages-select multirobot_map_merge
echo ""

if [ $? -eq 0 ]; then
    echo "=========================================="
    echo "✓ Successfully switched to experimental branch!"
    echo "=========================================="
    echo ""
    echo "You can now use slam_toolbox for multi-robot map merging."
    echo ""
    echo "To test multi-robot setup:"
    echo "  1. Launch multiple robots with SLAM:"
    echo "     ros2 launch multirobot_map_merge multi_tb3_simulation_launch.py slam_toolbox:=True"
    echo ""
    echo "  2. Launch map merging:"
    echo "     ros2 launch multirobot_map_merge map_merge.launch.py"
    echo ""
    echo "To switch back to main branch:"
    echo "  cd ~/turtlebot3_ws/src/m-explore-ros2"
    echo "  git checkout main"
    echo "  cd ~/turtlebot3_ws"
    echo "  colcon build --packages-select multirobot_map_merge"
    echo ""
else
    echo "=========================================="
    echo "⚠ Build failed!"
    echo "=========================================="
    echo ""
    echo "Please check the build errors above."
    echo "You may need to resolve compatibility issues."
    echo ""
    exit 1
fi
