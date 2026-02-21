#!/usr/bin/env python3
"""
slam.launch.py — Real-robot SLAM stack.

Composes:
  - lidar.launch.py        (sllidar hardware driver → /scan)
  - slam_common.launch.py  (RSP, TF, SLAM Toolbox)
  - localization.launch.py (optional EKF for fused odometry)

Usage:
  ros2 launch daro_bringup slam.launch.py
  ros2 launch daro_bringup slam.launch.py use_localization:=true
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    bringup = get_package_share_directory("daro_bringup")

    lidar_launch        = os.path.join(bringup, "launch", "lidar.launch.py")
    slam_common_launch  = os.path.join(bringup, "launch", "slam_common.launch.py")
    localization_launch = os.path.join(bringup, "launch", "localization.launch.py")

    log_level        = LaunchConfiguration("log_level")
    ekf_params       = LaunchConfiguration("ekf_params")
    use_localization = LaunchConfiguration("use_localization")

    # Invert use_localization: when EKF runs, don't publish static odom TF
    not_use_localization = PythonExpression(["'", use_localization, "' != 'true'"])

    # -------------------------------------------------
    # LiDAR hardware driver (publishes /scan)
    # -------------------------------------------------
    include_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch),
    )

    # -------------------------------------------------
    # Shared SLAM stack (RSP, TF, SLAM Toolbox)
    # -------------------------------------------------
    include_slam_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_common_launch),
        launch_arguments={
            "use_sim_time":    "false",
            "publish_odom_tf": not_use_localization,
            "log_level":       log_level,
        }.items(),
    )

    # -------------------------------------------------
    # EKF odometry fusion (optional)
    # -------------------------------------------------
    include_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={
            "ekf_params": ekf_params,
            "log_level":  log_level,
        }.items(),
        condition=IfCondition(use_localization),
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS log level",
        ),
        DeclareLaunchArgument(
            "ekf_params",
            default_value=os.path.join(bringup, "config", "ekf.yaml"),
            description="Path to robot_localization EKF params YAML",
        ),
        DeclareLaunchArgument(
            "use_localization",
            default_value="false",
            description=(
                "true  = start EKF (needs /wheel/odom + /imu/data_raw); "
                "false = static identity odom→base_link"
            ),
        ),

        include_lidar,
        include_slam_common,
        include_localization,
    ])
