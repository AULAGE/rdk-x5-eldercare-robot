import os
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(file_path: str) -> dict:
    with open(file_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    wheeltec_robot_dir = get_package_share_directory("turn_on_wheeltec_robot")
    wheeltec_launch_dir = os.path.join(wheeltec_robot_dir, "launch")

    wheeltec_nav_dir = get_package_share_directory("wheeltec_nav2")
    wheeltec_nav_launch_dir = os.path.join(wheeltec_nav_dir, "launch")

    cfg_params = load_yaml(
        os.path.join(
            get_package_share_directory("turn_on_wheeltec_robot"),
            "config",
            "wheeltec_param.yaml",
        )
    )
    car_mode = cfg_params["car_mode"]
    print(f"car_mode:{car_mode}")

    # 地图：优先使用持久化路径（如存在），否则使用包内默认地图
    persistent_map = Path("/home/sunrise/robot_map/WHEELTEC.yaml")
    default_map = str(persistent_map) if persistent_map.exists() else os.path.join(
        wheeltec_nav_dir, "map", "WHEELTEC.yaml"
    )
    map_file = LaunchConfiguration("map", default=default_map)

    param_dir = os.path.join(wheeltec_nav_dir, "param", "wheeltec_params")
    param_file = LaunchConfiguration(
        "params", default=os.path.join(param_dir, f"param_{car_mode}.yaml")
    )
    print(os.path.join(param_dir, f"param_{car_mode}.yaml"))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map", default_value=map_file, description="Full path to map file to load"
            ),
            DeclareLaunchArgument(
                "params",
                default_value=param_file,
                description="Full path to param file to load",
            ),
            Node(
                name="waypoint_cycle",
                package="nav2_waypoint_cycle",
                executable="nav2_waypoint_cycle",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [wheeltec_launch_dir, "/imu_processor.launch.py"]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [wheeltec_launch_dir, "/turn_on_wheeltec_robot.launch.py"]
                ),
                launch_arguments={"carto_slam": "false", "robot_nav": "false"}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [wheeltec_launch_dir, "/wheeltec_lidar.launch.py"]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [wheeltec_nav_launch_dir, "/bringup_launch.py"]
                ),
                launch_arguments={
                    "map": map_file,
                    "use_sim_time": use_sim_time,
                    "params_file": param_file,
                    "use_composition": "False",
                }.items(),
            ),
        ]
    )