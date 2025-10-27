# ROS2 Workspace for QIRP Device

This repository contains a ROS2 workspace configured for local development on Qualcomm Innovation Center's QIRP (Qualcomm Innovation Robotics Platform) devices. The workspace includes IMU-based odometry estimation and AI inference capabilities.

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Quick Start](#quick-start)
- [Workspace Structure](#workspace-structure)
- [Available Packages](#available-packages)
- [Development Setup](#development-setup)
- [Building and Running](#building-and-running)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)
- [Scripts Reference](#scripts-reference)

## Overview

This workspace is specifically configured for:
- **QIRP QCLinux** devices with Qualcomm Snapdragon processors
- **Local development** with colcon build system
- **IMU-based odometry** estimation using Extended Kalman Filter
- **AI inference** capabilities with TensorFlow Lite models
- **ROS2 Jazzy** distribution

## Prerequisites

- QIRP device with QCLinux OS
- ROS2 Jazzy installed
- Python 3.12
- Required build tools (automatically installed by setup scripts)

### Git Support

**Note**: Git CLI is not available via opkg on QIRP devices. This workspace includes:
- `.gitignore` file for version control
- Git repository structure initialization script
- GitPython for Python-based git operations (limited functionality)

**For full git functionality:**
- Use a host machine with git CLI for commits and pushes
- Sync files to a host machine for version control
- Use VS Code with remote development extensions

## Quick Start

### 🚨 **Important: Chicken and Egg Problem**

Git CLI is not available on QIRP devices, so you cannot clone this repository directly. Choose one of these options:

#### **Option A: Bootstrap Script (Recommended)**
1. **Copy the bootstrap script** to your QIRP device:
   ```bash
   # On host machine
   scp scripts/bootstrap_qirp.sh user@qirp-device:/root/
   
   # On QIRP device
   chmod +x bootstrap_qirp.sh
   ./bootstrap_qirp.sh
   ```

#### **Option B: Manual File Transfer**
1. **Clone on host machine**:
   ```bash
   git clone https://github.com/your-username/ros2_ws.git
   ```
2. **Transfer to QIRP device**:
   ```bash
   scp -r ros2_ws/ user@qirp-device:/root/
   ```

#### **Option C: Docker Development**
1. **Use Docker on host machine** for development
2. **Deploy to QIRP device** when ready

### 1. Setup Environment

```bash
# Navigate to workspace
cd ~/ros2_ws

# Setup local development environment (IMPORTANT: use 'source', not 'bash')
source scripts/ros2_env_setup.sh
```

### 2. Build Workspace

```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select odom_imu
```

### 3. Source Workspace

```bash
# Source the workspace
source install/setup.bash
```

### 4. Run Nodes

```bash
# Run IMU EKF node
ros2 run odom_imu imu_ekf_node

# Or use launch file
ros2 launch odom_imu imu_ekf_launch.py
```

## Workspace Structure

```
ros2_ws/
├── src/                          # Source packages
│   ├── odom_imu/                 # IMU odometry EKF package
│   │   ├── config/               # Configuration files
│   │   ├── launch/               # Launch files
│   │   ├── odom_imu/             # Python package
│   │   ├── package.xml           # Package manifest
│   │   └── setup.py              # Python setup
│   └── ai_inference/             # AI inference package
│       ├── ai_inference/         # Python package
│       ├── config/               # Configuration files
│       ├── launch/               # Launch files
│       ├── models/               # AI model files
│       │   └── yolov8_det_float.tflite
│       ├── package.xml           # Package manifest
│       └── setup.py              # Python setup
├── build/                        # Build artifacts
├── install/                      # Installed packages
├── log/                          # Build logs
├── scripts/                      # Setup and utility scripts
│   ├── qirp-setup-local.sh       # Main setup script
│   └── ros2_env_setup.sh         # Environment setup wrapper
└── README.md                     # This file
```

## Available Packages

### odom_imu
**IMU-based odometry estimation using Extended Kalman Filter**

- **Purpose**: Provides accurate odometry from IMU data with drift reduction
- **Features**: ZUPT detection, gravity alignment, adaptive bias estimation
- **Node**: `imu_ekf_node`
- **Launch**: `imu_ekf_launch.py`
- **Config**: `imu_ekf.yaml`

**Topics:**
- `/imu/data` (sensor_msgs/Imu) - Input IMU data
- `/odom` (nav_msgs/Odometry) - Output odometry
- `/tf` (tf2_msgs/TFMessage) - Transform data

### ai_inference
**AI inference capabilities with TensorFlow Lite models**

- **Purpose**: Real-time object detection and AI inference on QIRP device
- **Features**: YOLOv8 object detection, TensorFlow Lite optimization
- **Node**: `yolo_detector`
- **Launch**: `yolo_detector_launch.py`
- **Config**: `yolo_config.yaml`

**Topics:**
- `/camera/image_raw` (sensor_msgs/Image) - Input camera images
- `/ai_inference/detections` (vision_msgs/Detection2DArray) - Detection results


## Development Setup

### Environment Setup Scripts

The workspace includes several setup scripts for different use cases:

#### 1. Main Setup Script (`qirp-setup-local.sh`)
```bash
# Local development with colcon
source scripts/qirp-setup-local.sh --local

# Download AI models
source scripts/qirp-setup-local.sh --model

# Docker setup
source scripts/qirp-setup-local.sh --docker
```

#### 2. Environment Wrapper (`ros2_env_setup.sh`)
```bash
# Complete environment setup (recommended)
source scripts/ros2_env_setup.sh
```

**What it does:**
- Sets up QIRP environment
- Installs colcon and build tools
- Installs numpy and scipy
- Sets ROS_DOMAIN_ID=123
- Sources ROS2 environment

### Build Tools

The setup automatically installs and configures:
- ✅ **colcon** - ROS2 build system
- ✅ **cmake** - Build configuration
- ✅ **make** - Build automation
- ✅ **aarch64-qcom-linux-gcc** - QIRP compiler
- ✅ **numpy** - Scientific computing
- ✅ **scipy** - Advanced scientific functions
- ✅ **GitPython** - Python git operations (limited)

### Git Repository Setup

Since git CLI is not available on QIRP, use the provided script:

```bash
# Initialize git repository structure
./scripts/init_git_repo.sh

# This creates a basic .git structure for version control
# Use a host machine for full git operations
```

## Building and Running

### Build Commands

```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select odom_imu

# Build with verbose output
colcon build --event-handlers console_direct+

# Clean build
rm -rf build/ install/ log/
colcon build
```

### Run Commands

```bash
# Run IMU EKF node directly
ros2 run odom_imu imu_ekf_node

# Run with configuration file
ros2 run odom_imu imu_ekf_node --ros-args --params-file install/odom_imu/share/odom_imu/config/imu_ekf.yaml

# Run with launch file
ros2 launch odom_imu imu_ekf_launch.py

# List available nodes
ros2 pkg executables odom_imu
```

### Testing

```bash
# Run tests
colcon test

# Run tests for specific package
colcon test --packages-select odom_imu

# View test results
colcon test-result --all --verbose
```

## Configuration

### IMU EKF Configuration

Edit `src/odom_imu/config/imu_ekf.yaml` to configure:

```yaml
imu_ekf:
  ros__parameters:
    # IMU settings
    imu_topic: "/imu/data"
    odom_topic: "/odom"
    frame_id: "odom"
    child_frame_id: "base_link"
    
    # Filter parameters
    process_noise_pos: 0.01
    process_noise_vel: 0.1
    process_noise_att: 0.01
    process_noise_bias: 0.001
    
    # ZUPT settings
    zupt_enabled: true
    zupt_threshold: 0.1
    zupt_window_size: 10
```

### ROS2 Configuration

Set ROS_DOMAIN_ID to avoid conflicts:
```bash
export ROS_DOMAIN_ID=123
```

## Troubleshooting

### Common Issues

#### 1. Environment Variables Not Set
**Problem**: `ROS_DOMAIN_ID` or other variables not available after running script
**Solution**: Always use `source` instead of `bash`:
```bash
# ❌ Wrong
bash scripts/ros2_env_setup.sh

# ✅ Correct
source scripts/ros2_env_setup.sh
```

#### 2. Import Errors
**Problem**: `ModuleNotFoundError: No module named 'scipy'`
**Solution**: Run the setup script to install dependencies:
```bash
source scripts/qirp-setup-local.sh --local
```

#### 3. Build Failures
**Problem**: Colcon build fails
**Solution**: Check if environment is properly sourced:
```bash
# Verify colcon is available
which colcon

# Rebuild from scratch
rm -rf build/ install/ log/
source scripts/ros2_env_setup.sh
colcon build
```

#### 4. Node Crashes
**Problem**: `ExternalShutdownException` or similar errors
**Solution**: Ensure proper ROS2 environment:
```bash
# Source workspace
source install/setup.bash

# Check ROS2 environment
echo $ROS_DOMAIN_ID
ros2 node list
```

### Debug Commands

```bash
# Check ROS2 environment
ros2 doctor

# List available packages
ros2 pkg list

# Check node status
ros2 node list
ros2 topic list
ros2 service list

# Monitor topics
ros2 topic echo /odom
ros2 topic echo /imu/data
```

## Scripts Reference

### qirp-setup-local.sh

Main setup script with multiple options:

```bash
# Local development setup
source scripts/qirp-setup-local.sh --local

# Download AI models (requires network)
source scripts/qirp-setup-local.sh --model

# Docker setup
source scripts/qirp-setup-local.sh --docker

# Show help
source scripts/qirp-setup-local.sh --help
```

### ros2_env_setup.sh

Simple wrapper for local development:

```bash
# Complete environment setup
source scripts/ros2_env_setup.sh
```

**What it includes:**
- Calls `qirp-setup-local.sh --local`
- Sets `ROS_DOMAIN_ID=123`
- Remounts `/usr` with write permissions

### init_git_repo.sh

Git repository initialization for QIRP device:

```bash
# Initialize git repository structure
./scripts/init_git_repo.sh
```

**What it creates:**
- Basic `.git` directory structure
- Git configuration files
- Initial commit message
- Remote origin configuration

### bootstrap_qirp.sh

**Bootstrap script to solve the chicken-and-egg problem:**

```bash
# Copy to QIRP device and run
scp scripts/bootstrap_qirp.sh user@qirp-device:/root/
./bootstrap_qirp.sh
```

**What it does:**
- Installs pip, GitPython, numpy, scipy, colcon
- Clones the repository using GitPython
- Initializes git repository structure
- Runs complete workspace setup
- Provides next steps instructions

## Development Workflow

1. **Setup environment**:
   ```bash
   source scripts/ros2_env_setup.sh
   ```

2. **Make changes** to source code in `src/`

3. **Build workspace**:
   ```bash
   colcon build --packages-select your_package
   ```

4. **Source workspace**:
   ```bash
   source install/setup.bash
   ```

5. **Test changes**:
   ```bash
   ros2 run your_package your_node
   ```

## Support

For issues specific to:
- **QIRP device**: Check Qualcomm documentation
- **ROS2**: Refer to [ROS2 documentation](https://docs.ros.org/en/jazzy/)
- **This workspace**: Check package-specific README files in `src/`

## License

This workspace is configured for Qualcomm Innovation Center's QIRP platform. Please refer to individual package licenses for specific terms.
