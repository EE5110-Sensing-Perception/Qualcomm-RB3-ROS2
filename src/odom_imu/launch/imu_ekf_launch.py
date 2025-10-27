#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Locate package share directory
    pkg_share = FindPackageShare('odom_imu').find('odom_imu')
    config_file = os.path.join(pkg_share, 'config', 'imu_ekf.yaml')

    # IMU node from qrb_ros_imu package
    imu_node = Node(
        package='qrb_ros_imu',
        executable='imu_node',
        name='imu_node',
        output='screen'
    )

    # IMU EKF Python node
    imu_ekf_node = Node(
        package='odom_imu',
        executable='imu_ekf_node',  # matches setup.py console_scripts entry
        name='imu_ekf',
        output='screen',
        parameters=[config_file]
    )

    # Static transform: odom -> base_link
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_to_base',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    return LaunchDescription([
        imu_node,
        imu_ekf_node,
        static_tf_node
    ])

