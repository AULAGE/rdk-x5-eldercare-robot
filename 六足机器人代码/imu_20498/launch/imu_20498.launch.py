import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'imu_20498'
    
    return LaunchDescription([
        # IMU Serial Driver - USB1
        Node(
            package=pkg_name,
            executable='imu_20498_driver',
            name='imu_20498_driver',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 115200,
                'frame_id': 'imu_link',
                'publish_rate': 100,
            }]
        ),
        
        # IMU Filter (Madgwick) - produce quaternion orientation
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
                'orientation_stddev': 0.05,
            }],
        ),
        
        # Publish base_footprint -> imu_link static transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_broadcaster',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.05',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'imu_link'
            ]
        ),
        
        # Publish base_footprint -> laser static transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_broadcaster',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.1',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'laser'
            ]
        ),
    ])
