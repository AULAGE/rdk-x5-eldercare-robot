from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stm32_to_chassis',
            executable='stm32_to_chassis_bridge',
            name='stm32_to_chassis_bridge',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 115200,
            }],
        ),
    ])