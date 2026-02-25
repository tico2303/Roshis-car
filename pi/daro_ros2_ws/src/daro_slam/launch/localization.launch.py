#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_share = get_package_share_directory("daro_slam")

    ekf_params = LaunchConfiguration("ekf_params")
    log_level = LaunchConfiguration("log_level")

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        namespace="",
        output="screen",
        parameters=[ekf_params],
        arguments=["--ros-args", "--log-level", log_level],
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "ekf_params",
            default_value=os.path.join(slam_share, "config", "ekf.yaml"),
            description="Path to robot_localization EKF params yaml",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS log level",
        ),
        ekf_node,
    ])
