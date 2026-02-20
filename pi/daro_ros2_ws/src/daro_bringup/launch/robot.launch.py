#!/usr/bin/env python3
"""
robot.launch.py — full DARO robot bringup with SLAM and optional RViz.

Composes:
  daro.launch.py   — ESP32 bridge, joystick, drive, sensors, wheel odometry
  slam.launch.py   — LiDAR, robot_state_publisher, EKF, SLAM Toolbox

Usage (on the Pi):
  # Full robot + SLAM (no RViz — Pi headless):
  ros2 launch daro_bringup robot.launch.py

  # With RViz on the Pi (if display is attached):
  ros2 launch daro_bringup robot.launch.py rviz:=true

  # Override serial port or baud:
  ros2 launch daro_bringup robot.launch.py esp_port:=/dev/ttyUSB0 baud:=460800

RViz on macOS (run this in the Docker container after sourcing ROS2):
  rviz2 -d <path-to>/daro_bringup/rviz/slam.rviz
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from daro_bringup.defaults import ESP_PORT, BAUD


def generate_launch_description():

    bringup = get_package_share_directory("daro_bringup")
    launch_dir = os.path.join(bringup, "launch")
    rviz_cfg = os.path.join(bringup, "rviz", "slam.rviz")

    # ── Launch arguments ──────────────────────────────────────────────────────
    log_level      = LaunchConfiguration("log_level")
    esp_port       = LaunchConfiguration("esp_port")
    baud           = LaunchConfiguration("baud")
    rviz           = LaunchConfiguration("rviz")
    slam_params    = LaunchConfiguration("slam_params")
    ekf_params     = LaunchConfiguration("ekf_params")

    # ── Sub-launch files ──────────────────────────────────────────────────────

    # Robot hardware: ESP32, joy, teleop, drive, IMU, encoders, wheel odom
    include_daro = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "daro.launch.py")),
        launch_arguments={
            "esp_port":  esp_port,
            "baud":      baud,
            "log_level": log_level,
        }.items(),
    )

    # SLAM stack: LiDAR + robot_state_publisher + EKF + SLAM Toolbox
    # use_localization:=true  →  EKF fuses /wheel/odom + /imu/data_raw
    #                             and publishes odom→base_link.
    #                             SLAM Toolbox then only publishes map→odom.
    include_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "slam.launch.py")),
        launch_arguments={
            "use_localization": "true",
            "slam_params":      slam_params,
            "ekf_params":       ekf_params,
            "log_level":        log_level,
        }.items(),
    )

    # Optional RViz (useful if Pi has a display; normally run on macOS instead)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_cfg],
        condition=IfCondition(rviz),
    )

    # ── Compose ───────────────────────────────────────────────────────────────
    return LaunchDescription([

        DeclareLaunchArgument("log_level",   default_value="info"),
        DeclareLaunchArgument("esp_port",    default_value=ESP_PORT,
                              description="ESP32 serial device"),
        DeclareLaunchArgument("baud",        default_value=BAUD,
                              description="ESP32 serial baud rate"),
        DeclareLaunchArgument("rviz",        default_value="false",
                              description="Launch RViz2 on this machine"),
        DeclareLaunchArgument("slam_params", default_value=os.path.join(
                                  bringup, "config", "slam.yaml"),
                              description="Path to slam_toolbox params YAML"),
        DeclareLaunchArgument("ekf_params",  default_value=os.path.join(
                                  bringup, "config", "ekf.yaml"),
                              description="Path to EKF params YAML"),

        # Hardware first, then SLAM (SLAM needs /wheel/odom + /imu/data_raw)
        include_daro,
        include_slam,

        rviz_node,
    ])
