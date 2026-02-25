#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_share = get_package_share_directory("daro_slam")

    lidar_params = LaunchConfiguration("lidar_params")

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
            default_value=os.path.join(slam_share, "config", "lidar.yaml"),
            description="Path to lidar YAML params file",
        ),
        lidar_node,
    ])
