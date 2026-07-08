# Save map to persistent directory (not in install folder)
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_ros.actions


def generate_launch_description():
    # 使用持久化路径，编译不会被覆盖
    map_dir = '/home/sunrise/robot_map'
    os.makedirs(map_dir, exist_ok=True)
    map_path = os.path.join(map_dir, 'WHEELTEC')

    map_saver = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        output='screen',
        arguments=['-f', map_path],
        parameters=[{'save_map_timeout': 20000.0},
                    {'free_thresh_default': 0.196}]
    )

    ld = LaunchDescription()
    ld.add_action(map_saver)
    return ld
 
