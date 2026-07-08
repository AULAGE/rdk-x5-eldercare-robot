import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', 
        default=os.path.join(get_package_share_directory('wheeltec_cartographer') , 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='cartographer.lua')
    
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='0.5')
    
    # 建图状态文件保存在家目录，编译不被重置
    cartographer_state_path = os.path.expanduser('~/.cartographer/cartographer_state.pbstream')

    hipnuc_launch_dir = os.path.join(get_package_share_directory('hipnuc_imu'), 'launch')
    hipnuc_launch_file = os.path.join(hipnuc_launch_dir, 'hipnuc_imu.launch.py')

    wheeltec_lidar_launch_dir = os.path.join(get_package_share_directory('turn_on_wheeltec_robot'), 'launch')
    wheeltec_lidar_launch_file = os.path.join(wheeltec_lidar_launch_dir, 'wheeltec_lidar.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hipnuc_launch_file),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(wheeltec_lidar_launch_file),
        ),

        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            parameters=[{
                'use_sim_time': use_sim_time,
                'write_state_filename': cartographer_state_path,
            }],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename],
            remappings=[
                ('odom', 'odom'),
                ('imu', '/imu/data_raw')
            ]),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]),
    ])
