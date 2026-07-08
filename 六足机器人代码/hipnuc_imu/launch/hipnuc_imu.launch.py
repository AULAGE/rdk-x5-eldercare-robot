import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'hipnuc_imu'
    
    config = os.path.join(
        get_package_share_directory('hipnuc_imu'),
        'config',
        'hipnuc_config.yaml',
    )
    
    return LaunchDescription([
        Node(
            package=pkg_name,
            executable='talker',
            name='IMU_publisher',
            parameters=[{'serial_port': '/dev/ttyUSB0'}, {'baud_rate': 115200}, {'frame_id': 'imu_link'}, {'imu_topic': '/imu/data_raw'}],
            output='screen',
        ),

        # static transform: base_footprint -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_link_broadcaster',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.05',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'imu_link',
            ],
        ),

        # static transform: base_footprint -> laser (雷达位置)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_link_broadcaster',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.1',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'laser',
            ],
        ),
    ])
