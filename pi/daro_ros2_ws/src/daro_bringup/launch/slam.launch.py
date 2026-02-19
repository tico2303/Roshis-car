#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import LifecycleNode
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():

    # -------------------------------------------------
    # Package paths
    # -------------------------------------------------
    bringup = get_package_share_directory("daro_bringup")

    lidar_launch = os.path.join(bringup, "launch", "lidar.launch.py")
    localization_launch = os.path.join(bringup, "launch", "localization.launch.py")
    default_slam_params = os.path.join(bringup, "config", "slam.yaml")

    # -------------------------------------------------
    # Launch arguments
    # -------------------------------------------------
    slam_params = LaunchConfiguration("slam_params")
    log_level = LaunchConfiguration("log_level")

    # -------------------------------------------------
    # Include LiDAR
    # -------------------------------------------------
    include_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch),
        launch_arguments={"log_level": log_level}.items(),
    )

    # -------------------------------------------------
    # Include EKF (robot_localization)
    # This will publish odom -> base_link
    # -------------------------------------------------
    include_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={"log_level": log_level}.items(),
    )

    # -------------------------------------------------
    # SLAM Toolbox Lifecycle Node
    # -------------------------------------------------
    slam_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[ParameterFile(slam_params, allow_substs=True)],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # -------------------------------------------------
    # Lifecycle transitions
    # -------------------------------------------------
    configure_slam = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "configure"],
        output="screen",
    )

    activate_slam = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "activate"],
        output="screen",
    )

    # -------------------------------------------------
    # Launch Description
    # -------------------------------------------------
    return LaunchDescription([

        DeclareLaunchArgument(
            "slam_params",
            default_value=default_slam_params,
            description="Path to slam_toolbox YAML params file",
        ),

        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS log level",
        ),

        # Start sensor + localization first
        include_lidar,
        include_localization,

        # Then SLAM
        slam_node,

        # Delay lifecycle transitions so EKF + TF are alive first
        TimerAction(period=3.0, actions=[configure_slam]),
        TimerAction(period=5.0, actions=[activate_slam]),
    ])