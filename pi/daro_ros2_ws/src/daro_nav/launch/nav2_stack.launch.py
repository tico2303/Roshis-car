#!/usr/bin/env python3
"""
nav2_stack.launch.py — Nav2 nodes only (AMCL + map_server + planners + BT + lifecycle).

Designed to be included by nav.launch.py, or run standalone for development.
The hardware stack (EKF, LiDAR, etc.) must already be running.

Usage (standalone):
  ros2 launch daro_nav nav2_stack.launch.py map:=/home/pi/maps/my_map.yaml

Usage (via nav.launch.py — preferred):
  ros2 launch daro_nav nav.launch.py map:=/home/pi/maps/my_map.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    daro_nav_share  = get_package_share_directory("daro_nav")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    # ── Launch arguments ──────────────────────────────────────────────────────

    map_yaml   = LaunchConfiguration("map")
    params     = LaunchConfiguration("nav2_params")
    log_level  = LaunchConfiguration("log_level")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ── Include Nav2 bringup ──────────────────────────────────────────────────
    # nav2_bringup's bringup_launch.py launches all Nav2 lifecycle nodes and
    # the lifecycle manager in the correct order, using our params file.

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map":           map_yaml,
            "params_file":   params,
            "use_sim_time":  use_sim_time,
            "autostart":     "true",
            # AMCL handles localization — no SLAM Toolbox here
            "slam":          "false",
            "use_lifecycle_mgr": "true",
            "log_level":     log_level,
        }.items(),
    )

    # ── Compose ───────────────────────────────────────────────────────────────
    return LaunchDescription([

        DeclareLaunchArgument(
            "map",
            description="Full path to the map .yaml file (created with map_saver_cli)",
        ),
        DeclareLaunchArgument(
            "nav2_params",
            default_value=os.path.join(daro_nav_share, "config", "nav2_params.yaml"),
            description="Path to nav2_params.yaml — tune all Nav2 parameters here",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock (set true when running in Gazebo)",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS log level",
        ),

        nav2_bringup,
    ])
