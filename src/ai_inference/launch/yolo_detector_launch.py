#!/usr/bin/env python3

"""
Launch file for YOLO Object Detection Node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    package_dir = get_package_share_directory('ai_inference')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_dir, 'config', 'yolo_config.yaml'),
        description='Path to YOLO configuration file'
    )
    
    # YOLO Detector Node
    yolo_detector_node = Node(
        package='ai_inference',
        executable='yolo_detector',
        name='yolo_detector',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ('/camera/image_raw', '/camera/image_raw'),
            ('/ai_inference/detections', '/ai_inference/detections')
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        yolo_detector_node
    ])
