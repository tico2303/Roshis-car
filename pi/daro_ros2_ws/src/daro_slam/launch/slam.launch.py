#!/usr/bin/env python3
"""
slam.launch.py — Full DARO SLAM stack for real-robot map building.

Composes:
  daro_bringup/daro.launch.py   — ESP32, sensors, joystick, drive, wheel odom
  daro_slam/lidar.launch.py     — SLLIDAR hardware driver → /scan
  daro_slam/localization.launch.py — EKF: fuses /wheel/odom + /imu/data_raw
  daro_slam/slam_core.launch.py — RSP, static TF, SLAM Toolbox

Usage:
  ros2 launch daro_slam slam.launch.py

  # Override serial port or baud:
  ros2 launch daro_slam slam.launch.py esp_port:=/dev/ttyUSB0 baud:=460800

Save a map when done:
  ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from daro_bringup.defaults import ESP_PORT, BAUD


def generate_launch_description():

    bringup_share = get_package_share_directory("daro_bringup")
    slam_share    = get_package_share_directory("daro_slam")

    log_level   = LaunchConfiguration("log_level")
    ekf_params  = LaunchConfiguration("ekf_params")
    slam_params = LaunchConfiguration("slam_params")
    esp_port    = LaunchConfiguration("esp_port")
    baud        = LaunchConfiguration("baud")

    # ── Hardware: ESP32, sensors, joystick, drive ─────────────────────────────
    include_daro = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "daro.launch.py")
        ),
        launch_arguments={
            "esp_port":  esp_port,
            "baud":      baud,
            "log_level": log_level,
        }.items(),
    )

    # ── LiDAR hardware driver → /scan ─────────────────────────────────────────
    include_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, "launch", "lidar.launch.py")
        ),
    )

    # ── EKF: fuses wheel odom + IMU → odom→base_link TF ──────────────────────
    include_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, "launch", "localization.launch.py")
        ),
        launch_arguments={
            "ekf_params": ekf_params,
            "log_level":  log_level,
        }.items(),
    )

    # ── SLAM core: RSP, static TF, SLAM Toolbox ───────────────────────────────
    # EKF publishes odom→base_link, so suppress the static identity TF.
    include_slam_core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, "launch", "slam_core.launch.py")
        ),
        launch_arguments={
            "slam_params":    slam_params,
            "use_sim_time":   "false",
            "publish_odom_tf": "false",  # EKF handles odom→base_link
            "log_level":      log_level,
        }.items(),
    )

    return LaunchDescription([

        DeclareLaunchArgument("log_level",  default_value="info"),
        DeclareLaunchArgument(
            "esp_port",
            default_value=ESP_PORT,
            description="ESP32 serial device",
        ),
        DeclareLaunchArgument(
            "baud",
            default_value=BAUD,
            description="ESP32 serial baud rate",
        ),
        DeclareLaunchArgument(
            "ekf_params",
            default_value=os.path.join(slam_share, "config", "ekf.yaml"),
            description="Path to EKF params YAML",
        ),
        DeclareLaunchArgument(
            "slam_params",
            default_value=os.path.join(slam_share, "config", "slam.yaml"),
            description="Path to SLAM Toolbox params YAML",
        ),

        # Hardware first, then sensors, then SLAM
        include_daro,
        include_lidar,
        include_localization,
        include_slam_core,
    ])
