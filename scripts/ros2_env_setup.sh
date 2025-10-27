#!/bin/bash
# Set HOME variable
export HOME=/opt

# Remount the /usr directory with read-write permissions
mount -o remount,rw /usr

# Set up the runtime environment with local development support
source /root/ros2_ws/scripts/qirp-setup-local.sh --local

# Set the ROS_DOMAIN_ID
export ROS_DOMAIN_ID=123