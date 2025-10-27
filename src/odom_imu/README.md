# IMU Odometry EKF Node

A ROS2 node that provides odometry estimation using an Inertial Measurement Unit (IMU) through an Extended Kalman Filter (EKF). This implementation includes Zero Velocity Update (ZUPT) detection, gravity alignment, and adaptive bias estimation for drift reduction.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Configuration](#configuration)
- [IMU Axis Configuration](#imu-axis-configuration)
- [Topics](#topics)
- [Parameters](#parameters)
- [Algorithm Details](#algorithm-details)
- [Troubleshooting](#troubleshooting)
- [Performance Tuning](#performance-tuning)

## Overview

This node implements a 16-state Extended Kalman Filter for IMU-based odometry estimation:

**State Vector (16 elements):**
- Position (3): `[x, y, z]` in world frame
- Velocity (3): `[vx, vy, vz]` in world frame  
- Orientation (4): `[qx, qy, qz, qw]` quaternion
- Accelerometer bias (3): `[bax, bay, baz]`
- Gyroscope bias (3): `[bgx, bgy, bgz]`

The filter uses:
- **Prediction step**: IMU measurements to propagate state
- **ZUPT updates**: Zero velocity updates when stationary detected
- **Gravity alignment**: Orientation correction using gravity vector
- **Adaptive noise**: Motion-dependent process noise scaling
- **Bias estimation**: Online calibration of sensor biases

## Features

- ✅ **Drift reduction** through ZUPT and gravity alignment
- ✅ **Adaptive bias estimation** for long-term stability
- ✅ **Configurable axis remapping** for different IMU orientations
- ✅ **Dynamic stationary detection** using sliding window variance
- ✅ **TF broadcasting** for ROS navigation stack integration
- ✅ **Quaternion normalization** to prevent numerical drift
- ✅ **Adaptive process noise** based on motion intensity

## Installation

### Prerequisites

- ROS2 (Jazzy or later)
- Python 3.8+
- Required Python packages:
  ```bash
  pip3 install numpy scipy
  ```

### Build

1. Clone into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url> odom_imu
   ```

2. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select odom_imu
   source install/setup.bash
   ```

## Quick Start

### Basic Usage

1. **Ensure your IMU is publishing to `/imu` topic** with `sensor_msgs/Imu` messages

2. **Place IMU flat and stationary** on a level surface

3. **Launch the node:**
   ```bash
   ros2 run odom_imu imu_ekf_node
   ```

4. **Wait for initialization:**
   ```
   [INFO] [imu_ekf]: Bias initialized: ba=[...], bg=[...]
   [INFO] [imu_ekf]: Initial orientation set - Roll: 0.2°, Pitch: 0.1°
   ```

5. **View output:**
   ```bash
   # View odometry
   ros2 topic echo /odom
   
   # Visualize in RViz
   rviz2
   ```

### Using a Configuration File

1. Create `imu_ekf.yaml`:
   ```yaml
   imu_ekf:
     ros__parameters:
       window_size: 20
       accel_threshold: 0.1
       gyro_threshold: 0.01
       init_samples: 100
       imu_axis_remap: 'xyz'
       imu_axis_signs: '+++'
   ```

2. Launch with config:
   ```bash
   ros2 run odom_imu imu_ekf_node --ros-args --params-file /path/to/imu_ekf.yaml
   ```

## Configuration

### Parameters File Template

```yaml
imu_ekf:
  ros__parameters:
    # Stationary Detection
    window_size: 20              # Sliding window size for ZUPT detection
    accel_threshold: 0.1         # Acceleration variance threshold (m/s²)²
    gyro_threshold: 0.01         # Gyroscope variance threshold (rad/s)²
    
    # Kalman Filter Noise Parameters
    zupt_noise: 0.001            # Zero velocity update measurement noise
    process_noise_position: 0.01 # Position process noise
    process_noise_velocity: 0.05 # Velocity process noise
    process_noise_orientation: 0.001  # Orientation process noise
    process_noise_bias: 1e-6     # Bias process noise
    bias_random_walk: 1e-7       # Bias random walk noise
    
    # Initialization
    init_samples: 100            # Number of samples for bias initialization
    
    # IMU Axis Configuration
    imu_axis_remap: 'xyz'        # Axis mapping (see below)
    imu_axis_signs: '+++'        # Axis sign flipping (see below)
```

## IMU Axis Configuration

### Understanding ROS Conventions

ROS uses the **REP-103** convention:
- **X-axis**: Forward
- **Y-axis**: Left  
- **Z-axis**: Up

### Checking Your IMU Orientation

1. **Place IMU flat on a table and check raw data:**
   ```bash
   ros2 topic echo /imu --once
   ```

2. **Identify which axis reads ~9.8 m/s²** (that's pointing up)

3. **Common IMU configurations:**

   | IMU Orientation | `imu_axis_remap` | `imu_axis_signs` | Description |
   |----------------|------------------|------------------|-------------|
   | Standard ROS   | `xyz`            | `+++`            | Already aligned |
   | Rotated 90° CW | `yxz`            | `-++`            | Y forward, X right |
   | Upside down    | `xyz`            | `++-`            | Z pointing down |
   | Roll/Pitch swap| `yxz`            | `+++`            | X/Y axes swapped |

### Axis Remapping Explained

**`imu_axis_remap`**: Specifies which physical axis maps to each ROS axis
- First character: What maps to ROS X (forward)
- Second character: What maps to ROS Y (left)
- Third character: What maps to ROS Z (up)

**`imu_axis_signs`**: Specifies the sign of each remapped axis
- `+`: Keep original sign
- `-`: Flip sign

**Examples:**

```yaml
# IMU where Y-axis points forward, X points right, Z points up
imu_axis_remap: 'yxz'
imu_axis_signs: '-++'

# IMU mounted upside down
imu_axis_remap: 'xyz'
imu_axis_signs: '++-'

# IMU rotated 180° around Z-axis
imu_axis_remap: 'xyz'
imu_axis_signs: '--+'
```

### Testing Your Configuration

1. **Start node with IMU flat:**
   ```bash
   ros2 run odom_imu imu_ekf_node --ros-args -p imu_axis_remap:=xyz -p imu_axis_signs:=+++
   ```

2. **Open RViz and add:**
   - TF display
   - Odometry display (topic: `/odom`)
   - Set fixed frame to `odom`

3. **Verify movements:**
   - **Yaw** (rotate flat): Arrow sweeps horizontally
   - **Pitch** (tilt forward): Arrow tilts up/down
   - **Roll** (bank sideways): Arrow rotates around its length

4. **If wrong**, try different mappings until correct

## Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/imu` | `sensor_msgs/Imu` | Raw IMU measurements (required) |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | Estimated odometry (pose and velocity) |
| `/tf` | `tf2_msgs/TFMessage` | Transform from `odom` to `base_link` |

## Parameters

### Stationary Detection Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `window_size` | int | 20 | Number of samples for variance calculation |
| `accel_threshold` | double | 0.1 | Acceleration variance threshold for ZUPT |
| `gyro_threshold` | double | 0.01 | Gyroscope variance threshold for ZUPT |

**Tuning Tips:**
- Increase thresholds if ZUPT triggers during slow motion
- Decrease for more aggressive drift correction when stationary

### Process Noise Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `process_noise_position` | double | 0.01 | Position state uncertainty growth |
| `process_noise_velocity` | double | 0.05 | Velocity state uncertainty growth |
| `process_noise_orientation` | double | 0.001 | Orientation state uncertainty growth |
| `process_noise_bias` | double | 1e-6 | Bias state uncertainty growth |
| `bias_random_walk` | double | 1e-7 | Bias random walk noise |

**Tuning Tips:**
- Higher values = trust measurements more, faster response, more noise
- Lower values = trust model more, smoother output, slower response

### Measurement Noise Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `zupt_noise` | double | 0.001 | Zero velocity measurement noise |

### Initialization Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `init_samples` | int | 100 | Samples to collect before bias initialization |

**Note:** Keep IMU stationary during first few seconds for proper initialization

### IMU Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `imu_axis_remap` | string | 'xyz' | Axis remapping string (e.g., 'yxz', 'zxy') |
| `imu_axis_signs` | string | '+++' | Sign flipping (e.g., '+-+', '--+') |

## Algorithm Details

### State Prediction

The prediction step uses IMU measurements to propagate the state:

1. **Acceleration integration:**
   - Remove bias: `a_corrected = a_raw - bias_accel`
   - Rotate to world frame: `a_world = R * a_corrected`
   - Apply gravity: `a_world = a_world + [0, 0, -9.81]`
   - Update velocity: `v_new = v + a_world * dt`
   - Update position: `p_new = p + v * dt + 0.5 * a_world * dt²`

2. **Orientation integration:**
   - Remove bias: `ω_corrected = ω_raw - bias_gyro`
   - Integrate: `q_new = q * exp(ω_corrected * dt)`
   - Normalize quaternion

3. **Covariance prediction:**
   - Compute Jacobian matrix F
   - Update: `P = F * P * F^T + Q`

### Zero Velocity Update (ZUPT)

When stationary is detected:

1. **Stationary detection criteria:**
   - Acceleration variance < threshold
   - Gyroscope variance < threshold
   - Mean acceleration magnitude ≈ 9.81 m/s²

2. **Measurement update:**
   - Measurement: velocity = [0, 0, 0]
   - Innovation: `y = 0 - v_estimated`
   - Kalman gain: `K = P * H^T * (H * P * H^T + R)^-1`
   - State update: `x = x + K * y`
   - Covariance update: `P = (I - K * H) * P`

### Gravity Alignment

Uses gravity vector for orientation correction:

1. Measure gravity direction in body frame
2. Compare with expected direction based on current orientation
3. Compute cross product error
4. Apply correction through measurement update

### Adaptive Process Noise

Process noise scales with motion intensity:

```
motion_factor = 1.0 + 0.5 * max((accel_norm - 9.81) / 2.0, gyro_norm / 0.2, 0.0)
Q = Q_base * motion_factor
```

This allows:
- Lower noise when stationary (smoother, less drift)
- Higher noise during motion (faster response)

## Troubleshooting

### Common Issues

#### 1. High Drift Over Time

**Symptoms:** Position drifts significantly when stationary

**Solutions:**
- Ensure proper initialization (IMU must be flat and stationary)
- Increase `init_samples` for better bias estimation
- Decrease `accel_threshold` and `gyro_threshold` for more frequent ZUPT
- Lower process noise values

#### 2. Orientation is Wrong

**Symptoms:** Roll/pitch/yaw don't match physical movement

**Solutions:**
- Check IMU axis configuration (see [IMU Axis Configuration](#imu-axis-configuration))
- Verify with: `ros2 topic echo /imu`
- Try different `imu_axis_remap` and `imu_axis_signs` combinations
- Restart node with IMU completely flat

#### 3. "Unnormalized Quaternion" Error

**Symptoms:** Error about quaternion normalization

**Solutions:**
- Already fixed in current version (automatic normalization)
- If still occurs, check for numerical instabilities (very high IMU rates)

#### 4. Jerky or Noisy Output

**Symptoms:** Odometry jumps or jitters

**Solutions:**
- Increase process noise values for smoother filtering
- Increase `window_size` for ZUPT detection
- Check IMU data quality and sampling rate
- Verify IMU is properly mounted (vibration-isolated)

#### 5. Slow Response to Motion

**Symptoms:** Odometry lags behind actual movement

**Solutions:**
- Decrease process noise values
- Increase measurement noise (`zupt_noise`)
- Check IMU publishing rate (should be >50 Hz)

#### 6. Parameters Not Loading

**Symptoms:** Node uses default parameters despite config file

**Solutions:**
- Verify config file node name matches: `imu_ekf:`
- Use absolute path: `--params-file $(pwd)/config.yaml`
- Check YAML syntax (no tabs, proper indentation)
- Verify with: `ros2 param list /imu_ekf`

### Diagnostic Commands

```bash
# Check if IMU is publishing
ros2 topic hz /imu

# View raw IMU data
ros2 topic echo /imu

# Check node parameters
ros2 param list /imu_ekf

# Get specific parameter
ros2 param get /imu_ekf imu_axis_remap

# View odometry output
ros2 topic echo /odom

# Check TF tree
ros2 run tf2_tools view_frames
```

## Performance Tuning

### For High Accuracy (Slow Motion)

```yaml
process_noise_position: 0.001
process_noise_velocity: 0.01
process_noise_orientation: 0.0001
accel_threshold: 0.05
gyro_threshold: 0.005
```

### For Fast Response (Dynamic Motion)

```yaml
process_noise_position: 0.1
process_noise_velocity: 0.2
process_noise_orientation: 0.01
accel_threshold: 0.2
gyro_threshold: 0.05
```

### For Drift Minimization

```yaml
window_size: 30
accel_threshold: 0.08
gyro_threshold: 0.008
zupt_noise: 0.0001
init_samples: 200
```

### Recommended Settings by Application

| Application | Priority | Recommended Settings |
|-------------|----------|---------------------|
| Indoor Robot Navigation | Balance | Default settings |
| Handheld Device | Drift reduction | Drift minimization preset |
| Drone/Fast Motion | Responsiveness | Fast response preset |
| Precision Mapping | Accuracy | High accuracy preset |

## License

[Your License Here]

## Contributing

[Contribution Guidelines]

## Credits

Developed using:
- ROS2 (Robot Operating System)
- NumPy for numerical computations
- SciPy for spatial transformations
- EKF implementation based on standard IMU odometry techniques

## References

- [REP-103: Standard Units of Measure and Coordinate Conventions](https://www.ros.org/reps/rep-0103.html)
- [Quaternion Kinematics for Error-State Kalman Filter](https://arxiv.org/abs/1711.02508)
- [Zero Velocity Update (ZUPT) for Inertial Navigation](https://ieeexplore.ieee.org/document/4621073)