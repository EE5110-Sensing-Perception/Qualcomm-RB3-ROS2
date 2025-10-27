#!/bin/bash

# QIRP ROS2 Workspace Bootstrap Script
# This script can be manually copied to a QIRP device to set up the workspace

set -e

echo "QIRP ROS2 Workspace Bootstrap"
echo "============================="

# Repository information
REPO_URL="https://github.com/your-username/ros2_ws.git"
WORKSPACE_DIR="/root/ros2_ws"

# Check if we're on QIRP device
if [ ! -f "/usr/share/qirp-setup.sh" ]; then
    echo "Warning: This doesn't appear to be a QIRP device"
    echo "Continuing anyway..."
fi

# Install pip if not available
if ! command -v pip3 &> /dev/null; then
    echo "Installing pip..."
    python3 -m ensurepip --upgrade
fi

# Install GitPython
echo "Installing GitPython..."
python3 -m pip install GitPython --break-system-packages

# Install other dependencies
echo "Installing dependencies..."
python3 -m pip install numpy scipy --break-system-packages

# Install colcon
echo "Installing colcon..."
python3 -m pip install colcon-common-extensions --break-system-packages

# Clone repository using GitPython
echo "Cloning repository..."
python3 -c "
import git
import os
import shutil

# Remove existing workspace if it exists
if os.path.exists('$WORKSPACE_DIR'):
    shutil.rmtree('$WORKSPACE_DIR')

# Clone repository
repo = git.Repo.clone_from('$REPO_URL', '$WORKSPACE_DIR')
print('Repository cloned successfully to $WORKSPACE_DIR')
"

# Navigate to workspace
cd "$WORKSPACE_DIR"

# Make scripts executable
chmod +x scripts/*.sh

# Initialize git repository structure
echo "Initializing git repository structure..."
./scripts/init_git_repo.sh

# Run setup
echo "Running workspace setup..."
source scripts/ros2_env_setup.sh

echo ""
echo "✅ QIRP ROS2 Workspace setup complete!"
echo ""
echo "Next steps:"
echo "1. cd $WORKSPACE_DIR"
echo "2. colcon build"
echo "3. source install/setup.bash"
echo "4. ros2 run odom_imu imu_ekf_node"
echo ""
echo "For development:"
echo "- Edit code in src/ directory"
echo "- Use colcon build to compile"
echo "- Use git on host machine for version control"
