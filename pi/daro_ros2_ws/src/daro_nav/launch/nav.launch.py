#!/usr/bin/env python3
"""
nav.launch.py — Full DARO autonomous navigation stack.

Composes:
  robot.launch.py (daro_bringup) — hardware, EKF, LiDAR  [slam:=false]
  nav2_stack.launch.py (daro_nav) — AMCL + Nav2 planners + lifecycle manager

Prerequisites:
  1. A saved map — create one first with SLAM:
       ros2 launch daro_bringup robot.launch.py
       # drive the robot around to build the map, then:
       ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

  2. Pass the map path at launch time (map:= argument).

Usage:
  ros2 launch daro_nav nav.launch.py map:=/home/pi/maps/my_map.yaml
  ros2 launch daro_nav nav.launch.py map:=/home/pi/maps/my_map.yaml esp_port:=/dev/ttyUSB0

RViz (on macOS Docker or Pi with display):
  rviz2 -d <daro_bringup>/rviz/nav.rviz
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
    daro_nav_share = get_package_share_directory("daro_nav")

    # ── Launch arguments ──────────────────────────────────────────────────────

    map_yaml     = LaunchConfiguration("map")
    nav2_params  = LaunchConfiguration("nav2_params")
    ekf_params   = LaunchConfiguration("ekf_params")
    esp_port     = LaunchConfiguration("esp_port")
    baud         = LaunchConfiguration("baud")
    log_level    = LaunchConfiguration("log_level")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ── Hardware stack (no SLAM — AMCL handles localization) ──────────────────
    # robot.launch.py is called with slam:=false so SLAM Toolbox is not started.
    # The EKF still runs (via use_localization:=true inside slam.launch.py),
    # publishing the odom→base_link TF that Nav2 needs.

    include_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "robot.launch.py")
        ),
        launch_arguments={
            "esp_port":  esp_port,
            "baud":      baud,
            "log_level": log_level,
            "slam":      "false",   # AMCL replaces SLAM Toolbox for map→odom TF
            "rviz":      "false",   # launch RViz separately if needed
            "keyboard":  "false",   # Nav2 commands /cmd_vel; disable manual teleop
            "ekf_params": ekf_params,
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
            default_value=os.path.join(bringup_share, "config", "ekf.yaml"),
            description="Path to EKF parameters YAML",
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
        include_robot,
        include_nav2,
    ])
