import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import numpy as np
from scipy.spatial.transform import Rotation as R
from collections import deque
import tf2_ros

def skew(vec):
    """Skew-symmetric matrix"""
    return np.array([
        [0, -vec[2], vec[1]],
        [vec[2], 0, -vec[0]],
        [-vec[1], vec[0], 0]
    ])

class IMUOdometryEKF(Node):
    def __init__(self):
        super().__init__('imu_ekf')
        
        # Parameters
        self.declare_parameter('window_size', 20)
        self.declare_parameter('accel_threshold', 0.15)
        self.declare_parameter('gyro_threshold', 0.05)
        self.declare_parameter('bias_random_walk', 1e-7)
        self.declare_parameter('zupt_noise', 0.0001)
        self.declare_parameter('process_noise_position', 0.001)
        self.declare_parameter('process_noise_velocity', 0.01)
        self.declare_parameter('process_noise_orientation', 0.0001)
        self.declare_parameter('process_noise_bias', 1e-7)
        self.declare_parameter('init_samples', 100)
        self.declare_parameter('imu_axis_remap', 'xyz')  # Options: 'xyz', 'xzy', 'yxz', 'yzx', 'zxy', 'zyx'
        self.declare_parameter('imu_axis_signs', '+++')   # + or - for each axis

        # Load parameters
        self.window_size = self.get_parameter('window_size').get_parameter_value().integer_value
        self.accel_threshold = self.get_parameter('accel_threshold').get_parameter_value().double_value
        self.gyro_threshold = self.get_parameter('gyro_threshold').get_parameter_value().double_value
        self.bias_rw = self.get_parameter('bias_random_walk').get_parameter_value().double_value
        self.zupt_noise = self.get_parameter('zupt_noise').get_parameter_value().double_value
        self.process_noise_position = self.get_parameter('process_noise_position').get_parameter_value().double_value
        self.process_noise_velocity = self.get_parameter('process_noise_velocity').get_parameter_value().double_value
        self.process_noise_orientation = self.get_parameter('process_noise_orientation').get_parameter_value().double_value
        self.process_noise_bias = self.get_parameter('process_noise_bias').get_parameter_value().double_value
        self.init_samples = self.get_parameter('init_samples').get_parameter_value().integer_value
        self.imu_axis_remap = self.get_parameter('imu_axis_remap').get_parameter_value().string_value
        self.imu_axis_signs = self.get_parameter('imu_axis_signs').get_parameter_value().string_value

        # State: p(3), v(3), q(4), ba(3), bg(3)
        self.state = np.zeros(16)
        self.state[6:10] = [0, 0, 0, 1]  # quaternion [x, y, z, w]

        # Covariance
        self.P = np.eye(16) * 0.01
        self.P[6:10, 6:10] = np.eye(4) * 0.001  # Lower initial orientation uncertainty
        self.P_max = np.eye(16) * 1e3

        # Process noise
        self.Q_base = np.diag([
            self.process_noise_position, self.process_noise_position, self.process_noise_position,
            self.process_noise_velocity, self.process_noise_velocity, self.process_noise_velocity,
            self.process_noise_orientation, self.process_noise_orientation, self.process_noise_orientation, self.process_noise_orientation,
            self.process_noise_bias, self.process_noise_bias, self.process_noise_bias,
            self.process_noise_bias, self.process_noise_bias, self.process_noise_bias
        ])

        # Gravity in world frame (ROS: z-up)
        self.gravity = np.array([0, 0, -9.81])

        # Sliding window for dynamic ZUPT
        self.accel_window = deque(maxlen=self.window_size)
        self.gyro_window = deque(maxlen=self.window_size)

        # Bias initialization buffers
        self.init_accel = []
        self.init_gyro = []
        self.bias_initialized = False

        # Subscribers / publishers
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.prev_time = None
        self.msg_count = 0

    def normalize_quaternion(self):
        """Ensure quaternion remains unit norm"""
        quat_norm = np.linalg.norm(self.state[6:10])
        if quat_norm > 0:
            self.state[6:10] = self.state[6:10] / quat_norm

    def remap_imu_axes(self, vec):
        """Remap IMU axes according to configuration"""
        # Parse axis mapping
        axis_map = {'x': 0, 'y': 1, 'z': 2}
        indices = [axis_map[c] for c in self.imu_axis_remap.lower()]
        
        # Parse signs
        signs = [1 if s == '+' else -1 for s in self.imu_axis_signs]
        
        # Apply remapping
        remapped = np.array([
            signs[0] * vec[indices[0]],
            signs[1] * vec[indices[1]],
            signs[2] * vec[indices[2]]
        ])
        
        return remapped

    def imu_callback(self, msg: Imu):
        # Standard IMU orientation: assume IMU reports data in its local frame
        # ROS convention: x-forward, y-left, z-up
        # Common IMU: x-forward, y-left, z-up (if already aligned) OR needs remapping
        # Check your IMU datasheet - this assumes standard ROS-aligned IMU
        accel_raw = self.remap_imu_axes(np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]))
        gyro_raw = self.remap_imu_axes(np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ]))

        # Initialize biases if not done
        if not self.bias_initialized:
            self.init_accel.append(accel_raw)
            self.init_gyro.append(gyro_raw)
            if len(self.init_accel) >= self.init_samples:
                # Accel bias: measured gravity in body frame minus expected
                accel_mean = np.mean(self.init_accel, axis=0)
                # Assuming stationary and level initially: accel should read [0, 0, 9.81]
                self.state[10:13] = accel_mean - np.array([0, 0, 9.81])
                self.state[13:16] = np.mean(self.init_gyro, axis=0)
                self.bias_initialized = True
                self.get_logger().info(f"Bias initialized: ba={self.state[10:13]}, bg={self.state[13:16]}")
                self.get_logger().info(f"Initial accel mean: {accel_mean}")
                self.get_logger().info(f"Initial orientation (quaternion): {self.state[6:10]}")
                
                # Set initial orientation based on gravity
                # If stationary, accelerometer reads gravity vector in body frame
                # For a level IMU: accel should read [0, 0, 9.81]
                # Calculate the rotation needed to align body z-axis with world z-axis
                g_body = accel_mean / np.linalg.norm(accel_mean)
                
                # The measured gravity vector tells us how the body frame is rotated
                # We want to find rotation that brings [0,0,1] to g_body direction
                # But gravity points down, so g_body ≈ [0, 0, 9.81] when level
                
                # Calculate roll and pitch from gravity (standard IMU alignment)
                roll_init = np.arctan2(-g_body[1], g_body[2])
                pitch_init = np.arctan2(g_body[0], np.sqrt(g_body[1]**2 + g_body[2]**2))
                
                # Create initial rotation (yaw = 0, we can't determine yaw from gravity alone)
                initial_rot = R.from_euler('XYZ', [roll_init, pitch_init, 0])
                self.state[6:10] = initial_rot.as_quat()
                
                self.get_logger().info(f"Initial orientation set - Roll: {np.degrees(roll_init):.2f}°, Pitch: {np.degrees(pitch_init):.2f}°")
                self.get_logger().info(f"Initial quaternion [x,y,z,w]: {self.state[6:10]}")
            return

        # Compute dt
        current_time = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_time is None:
            self.prev_time = current_time
            return
        dt = current_time - self.prev_time
        if dt <= 0 or dt > 1.0:  # Skip invalid dt
            self.prev_time = current_time
            return
        self.prev_time = current_time

        # Remove biases (now accel is true specific force in body frame)
        accel = accel_raw - self.state[10:13]
        gyro = gyro_raw - self.state[13:16]

        # EKF predict
        self.ekf_predict(accel, gyro, dt)

        # Update sliding windows
        self.accel_window.append(accel_raw)
        self.gyro_window.append(gyro_raw)

        # Dynamic ZUPT
        if self.detect_stationary():
            self.zupt_update()
            self.gravity_alignment_update(accel)

        # Periodic logging for debugging
        self.msg_count += 1
        if self.msg_count % 50 == 0:
            rot_current = R.from_quat(self.state[6:10])
            euler = rot_current.as_euler('XYZ', degrees=True)
            self.get_logger().info(f"Orient [R,P,Y]: [{euler[0]:.1f}°, {euler[1]:.1f}°, {euler[2]:.1f}°], Pos: [{self.state[0]:.2f}, {self.state[1]:.2f}, {self.state[2]:.2f}]")

        # Publish odometry & TF
        self.publish_odometry(msg.header.stamp)

    def ekf_predict(self, accel, gyro, dt):
        p = self.state[0:3]
        v = self.state[3:6]
        q = self.state[6:10]

        # Rotate acceleration from body frame to world frame
        rot = R.from_quat(q)
        accel_world = rot.apply(accel)
        
        # Apply gravity (gravity is already negative in z)
        v_new = v + (accel_world + self.gravity) * dt
        p_new = p + v * dt + 0.5 * (accel_world + self.gravity) * dt**2

        # Update orientation
        delta_q = R.from_rotvec(gyro * dt)
        rot_new = rot * delta_q
        q_new = rot_new.as_quat()
        
        # Normalize quaternion
        q_new = q_new / np.linalg.norm(q_new)

        self.state[0:3] = p_new
        self.state[3:6] = v_new
        self.state[6:10] = q_new

        # Process noise with adaptive scaling
        accel_norm = np.linalg.norm(accel)
        gyro_norm = np.linalg.norm(gyro)
        
        # Reduced motion scaling to prevent excessive noise growth
        motion_factor = 1.0 + 0.5 * max((accel_norm - 9.81) / 2.0, gyro_norm / 0.2, 0.0)
        Q = self.Q_base.copy() * motion_factor
        
        # Add bias random walk
        for i in range(10, 16):
            Q[i, i] += self.bias_rw

        # Jacobian for covariance propagation (simplified)
        F = np.eye(16)
        F[0:3, 3:6] = np.eye(3) * dt
        
        # Velocity affected by rotation and biases
        R_mat = rot.as_matrix()
        F[3:6, 10:13] = -R_mat * dt
        
        # Orientation affected by gyro biases (simplified)
        F[6:9, 13:16] = -np.eye(3) * dt
        
        self.P = F @ self.P @ F.T + Q
        self.P = np.minimum(self.P, self.P_max)

    def detect_stationary(self):
        if len(self.accel_window) < self.window_size:
            return False
        
        accel_array = np.array(self.accel_window)
        gyro_array = np.array(self.gyro_window)
        
        # Check variance of accelerometer (should be near [0,0,g] when stationary)
        accel_var = np.var(accel_array, axis=0).sum()
        gyro_var = np.var(gyro_array, axis=0).sum()
        
        # Check if magnitude is close to gravity
        accel_mag = np.linalg.norm(np.mean(accel_array, axis=0))
        gravity_check = abs(accel_mag - 9.81) < 0.5
        
        is_stationary = (accel_var < self.accel_threshold and 
                        gyro_var < self.gyro_threshold and 
                        gravity_check)
        
        return is_stationary

    def zupt_update(self):
        """Zero velocity update"""
        H = np.zeros((3, 16))
        H[0:3, 3:6] = np.eye(3)
        
        z = np.zeros(3)  # Expected velocity is zero
        y = z - self.state[3:6]  # Innovation
        
        S = H @ self.P @ H.T + np.eye(3) * self.zupt_noise
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.state = self.state + K @ y
        self.P = (np.eye(16) - K @ H) @ self.P
        
        # Normalize quaternion
        self.normalize_quaternion()

    def gravity_alignment_update(self, accel_body):
        """Update orientation using gravity vector alignment when stationary"""
        # Measured gravity direction in body frame (normalized)
        g_body_meas = accel_body / np.linalg.norm(accel_body)
        
        # Expected gravity direction in world frame
        g_world = np.array([0, 0, -1])  # Normalized
        
        # Transform expected gravity to body frame using current orientation
        rot = R.from_quat(self.state[6:10])
        g_body_expected = rot.inv().apply(g_world)
        
        # Error: difference between measured and expected (in body frame)
        error = np.cross(g_body_expected, g_body_meas)
        
        # Small angle approximation: convert to quaternion error
        # This assumes error is small, which is valid during correction
        error_quat = np.array([error[0], error[1], error[2], 0]) * 0.5
        
        # Observation matrix (maps quaternion to gravity error)
        H = np.zeros((3, 16))
        H[0:3, 6:9] = np.eye(3) * 2.0  # Simplified - maps to first 3 quat components
        
        # Measurement noise
        R_gravity = np.eye(3) * 0.01
        
        S = H @ self.P @ H.T + R_gravity
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.state = self.state + K @ error
        self.P = (np.eye(16) - K @ H) @ self.P
        
        # Normalize quaternion
        self.normalize_quaternion()

    def publish_odometry(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = float(self.state[0])
        odom.pose.pose.position.y = float(self.state[1])
        odom.pose.pose.position.z = float(self.state[2])
        
        odom.pose.pose.orientation.x = float(self.state[6])
        odom.pose.pose.orientation.y = float(self.state[7])
        odom.pose.pose.orientation.z = float(self.state[8])
        odom.pose.pose.orientation.w = float(self.state[9])
        
        odom.twist.twist.linear.x = float(self.state[3])
        odom.twist.twist.linear.y = float(self.state[4])
        odom.twist.twist.linear.z = float(self.state[5])
        
        self.odom_pub.publish(odom)
        self.publish_tf(stamp)

    def publish_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = float(self.state[0])
        t.transform.translation.y = float(self.state[1])
        t.transform.translation.z = float(self.state[2])
        
        t.transform.rotation.x = float(self.state[6])
        t.transform.rotation.y = float(self.state[7])
        t.transform.rotation.z = float(self.state[8])
        t.transform.rotation.w = float(self.state[9])
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = IMUOdometryEKF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
