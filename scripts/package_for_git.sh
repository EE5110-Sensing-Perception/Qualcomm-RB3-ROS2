#!/bin/bash

# Package ROS2 Workspace for Git Transfer
# This script prepares the workspace for transfer to a host machine with git CLI

echo "Packaging ROS2 workspace for git transfer..."

# Create a clean copy without build artifacts
TEMP_DIR="/tmp/ros2_ws_clean"
PACKAGE_NAME="ros2_ws_$(date +%Y%m%d_%H%M%S).tar.gz"

# Remove existing temp directory
rm -rf "$TEMP_DIR"

# Create clean workspace structure
mkdir -p "$TEMP_DIR"
cp -r src/ "$TEMP_DIR/"
cp -r scripts/ "$TEMP_DIR/"
cp -r dockerfile/ "$TEMP_DIR/"
cp .gitignore "$TEMP_DIR/"
cp README.md "$TEMP_DIR/"

# Copy git configuration
if [ -d ".git" ]; then
    cp -r .git/ "$TEMP_DIR/"
    echo "✓ Git configuration copied"
else
    echo "⚠ No .git directory found"
fi

# Create package
cd /tmp
tar -czf "$PACKAGE_NAME" ros2_ws_clean/
mv "$PACKAGE_NAME" /root/

echo ""
echo "✅ Workspace packaged successfully!"
echo "Package: /root/$PACKAGE_NAME"
echo ""
echo "Next steps:"
echo "1. Transfer package to host machine:"
echo "   scp /root/$PACKAGE_NAME user@host-machine:/path/to/destination/"
echo ""
echo "2. On host machine:"
echo "   tar -xzf $PACKAGE_NAME"
echo "   cd ros2_ws_clean"
echo "   git remote add origin git@github.com:EE5110-Sensing-Perception/Qualcomm-RB3-ROS2.git"
echo "   git branch -M main"
echo "   git add ."
echo "   git commit -m 'Initial commit: QIRP ROS2 workspace'"
echo "   git push -u origin main"
echo ""
echo "3. To sync changes back to QIRP:"
echo "   git pull origin main"
echo "   # Then transfer back to QIRP device"
