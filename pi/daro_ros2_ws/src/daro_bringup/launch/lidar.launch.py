import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = get_package_share_directory("daro_bringup")

    default_lidar_params = os.path.join(bringup_share, "config", "lidar.yaml")

    lidar_params = LaunchConfiguration("lidar_params")

    # --- LiDAR driver ---
    lidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar",
        output="screen",
        parameters=[lidar_params],
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "lidar_params",
            default_value=default_lidar_params,
            description="Path to lidar YAML params file",
        ),

        lidar_node,
    ])
