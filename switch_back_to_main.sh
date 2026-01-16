#!/bin/bash
# Script to switch m-explore-ros2 back to main branch
# (for single-robot exploration)

set -e  # Exit on error

echo "=========================================="
echo "Switching back to main branch"
echo "=========================================="
echo ""

cd ~/turtlebot3_ws/src/m-explore-ros2

# Check current branch
CURRENT_BRANCH=$(git branch --show-current)
echo "Current branch: $CURRENT_BRANCH"
echo ""

if [ "$CURRENT_BRANCH" = "main" ]; then
    echo "Already on main branch. Nothing to do."
    exit 0
fi

# Check if we have uncommitted changes
if ! git diff-index --quiet HEAD --; then
    echo "⚠ WARNING: You have uncommitted changes!"
    echo "Please commit or stash them before switching branches."
    echo ""
    read -p "Stash changes and continue? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        git stash push -m "Stashed before switching back to main branch"
        echo "✓ Changes stashed"
    else
        echo "Aborted. Please handle your changes first."
        exit 1
    fi
fi

# Switch to main branch
echo "Switching to main branch..."
git checkout main
echo "✓ Switched to main branch"
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
colcon build --packages-select multirobot_map_merge explore_lite
echo ""

if [ $? -eq 0 ]; then
    echo "=========================================="
    echo "✓ Successfully switched back to main branch!"
    echo "=========================================="
    echo ""
    echo "You're now using the stable main branch for single-robot exploration."
    echo ""
else
    echo "=========================================="
    echo "⚠ Build failed!"
    echo "=========================================="
    echo ""
    echo "Please check the build errors above."
    echo ""
    exit 1
fi
