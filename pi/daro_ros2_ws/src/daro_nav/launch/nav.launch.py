#!/usr/bin/env python3
"""
nav.launch.py — Full DARO autonomous navigation stack.

Composes:
  daro_bringup/daro.launch.py       — ESP32, sensors, joystick, drive, wheel odom
  daro_slam/lidar.launch.py         — SLLIDAR hardware driver → /scan
  daro_slam/localization.launch.py  — EKF: fuses /wheel/odom + /imu/data_raw
  daro_slam/slam_core.launch.py     — RSP + static TF (no SLAM Toolbox)
  daro_nav/nav2_stack.launch.py     — AMCL + Nav2 planners + lifecycle manager

Prerequisites:
  1. A saved map — create one first with SLAM:
       ros2 launch daro_slam slam.launch.py
       # drive the robot around to build the map, then:
       ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

  2. Pass the map path at launch time (map:= argument).

Usage:
  ros2 launch daro_nav nav.launch.py map:=/home/pi/maps/my_map.yaml
  ros2 launch daro_nav nav.launch.py map:=/home/pi/maps/my_map.yaml esp_port:=/dev/ttyUSB0

RViz (on macOS Docker or Pi with display):
  rviz2 -d <daro_bringup>/rviz/slam.rviz
  # Then use "2D Pose Estimate" to initialize AMCL, and "2D Nav Goal" to send goals.

Tuning:
  Edit daro_nav/config/nav2_params.yaml for all Nav2 parameters.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from daro_bringup.defaults import ESP_PORT, BAUD


def generate_launch_description():

    bringup_share  = get_package_share_directory("daro_bringup")
    slam_share     = get_package_share_directory("daro_slam")
    daro_nav_share = get_package_share_directory("daro_nav")

    # ── Launch arguments ──────────────────────────────────────────────────────

    map_yaml     = LaunchConfiguration("map")
    nav2_params  = LaunchConfiguration("nav2_params")
    ekf_params   = LaunchConfiguration("ekf_params")
    slam_params  = LaunchConfiguration("slam_params")
    esp_port     = LaunchConfiguration("esp_port")
    baud         = LaunchConfiguration("baud")
    log_level    = LaunchConfiguration("log_level")
    use_sim_time = LaunchConfiguration("use_sim_time")

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

    # ── SLAM core: RSP + static TF only (no SLAM Toolbox) ────────────────────
    # AMCL (in nav2_stack) publishes map→odom — we don't need SLAM Toolbox.
    # EKF publishes odom→base_link, so suppress the static identity TF.
    include_slam_core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, "launch", "slam_core.launch.py")
        ),
        launch_arguments={
            "slam_params":     slam_params,
            "use_sim_time":    use_sim_time,
            "publish_odom_tf": "false",  # EKF handles odom→base_link
            "log_level":       log_level,
        }.items(),
    )

    # ── Nav2 stack (AMCL + map server + planners + lifecycle) ─────────────────
    include_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(daro_nav_share, "launch", "nav2_stack.launch.py")
        ),
        launch_arguments={
            "map":          map_yaml,
            "nav2_params":  nav2_params,
            "use_sim_time": use_sim_time,
            "log_level":    log_level,
        }.items(),
    )

    # ── Compose ───────────────────────────────────────────────────────────────
    return LaunchDescription([

        DeclareLaunchArgument(
            "map",
            description="Full path to the saved map .yaml file",
        ),
        DeclareLaunchArgument(
            "nav2_params",
            default_value=os.path.join(daro_nav_share, "config", "nav2_params.yaml"),
            description="Path to Nav2 parameters YAML — tune navigation here",
        ),
        DeclareLaunchArgument(
            "ekf_params",
            default_value=os.path.join(slam_share, "config", "ekf.yaml"),
            description="Path to EKF parameters YAML",
        ),
        DeclareLaunchArgument(
            "slam_params",
            default_value=os.path.join(slam_share, "config", "slam.yaml"),
            description="Path to SLAM core parameters YAML (RSP/TF only in nav mode)",
        ),
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
            "use_sim_time",
            default_value="false",
            description="Use simulation clock (true when running in Gazebo)",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS log level",
        ),

        # Hardware must come up before Nav2 so topics/TF are available
        include_daro,
        include_lidar,
        include_localization,
        include_slam_core,
        include_nav2,
    ])
