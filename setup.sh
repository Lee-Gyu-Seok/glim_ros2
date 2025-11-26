#!/bin/bash
# Setup script for glim_ros2 workspace
# This script initializes submodules and applies ROS2 compatibility patches

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "==========================================="
echo "Setting up glim_ros2 workspace..."
echo "==========================================="
echo ""

# Initialize submodules if not already done
echo "[1/2] Initializing git submodules..."
cd "$SCRIPT_DIR"
git submodule update --init --recursive

# Checkout specific versions for compatibility
echo ""
echo "[2/2] Checking out compatible versions..."
cd "$SCRIPT_DIR/src/gtsam_points"
git checkout v1.0.8 2>/dev/null || true

cd "$SCRIPT_DIR/src/iridescence"
git submodule update --init --recursive

echo ""
echo "==========================================="
echo "Setup complete!"
echo "==========================================="
echo ""
echo "Build with:"
echo "  source /opt/ros/humble/setup.bash"
echo "  colcon build"
echo ""
echo "After build, source the workspace:"
echo "  source install/setup.bash"
