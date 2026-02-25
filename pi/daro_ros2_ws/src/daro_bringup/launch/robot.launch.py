#!/usr/bin/env python3
"""
robot.launch.py — full DARO robot bringup with optional SLAM and RViz.

Composes:
  daro.launch.py   — ESP32 bridge, joystick, drive, sensors, wheel odometry
  slam.launch.py   — LiDAR, robot_state_publisher, EKF, SLAM Toolbox

Usage (on the Pi):
  # Full robot + SLAM + RViz + keyboard teleop:
  ros2 launch daro_bringup robot.launch.py rviz:=true

  # Without keyboard teleop:
  ros2 launch daro_bringup robot.launch.py rviz:=true keyboard:=false

  # Hardware + EKF + LiDAR only (no SLAM — used by daro_nav for AMCL navigation):
  ros2 launch daro_bringup robot.launch.py slam:=false

  # Override serial port or baud:
  ros2 launch daro_bringup robot.launch.py rviz:=true esp_port:=/dev/ttyUSB0 baud:=460800

RViz on macOS (run this in the Docker container after sourcing ROS2):
  rviz2 -d <path-to>/daro_bringup/rviz/slam.rviz

Keyboard controls (when keyboard:=true):
  i / , — forward / backward
  j / l — rotate left / right
  k     — stop
  q / z — increase / decrease max speed
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
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
    keyboard       = LaunchConfiguration("keyboard")
    slam           = LaunchConfiguration("slam")
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
    # When slam:=false (e.g. called from daro_nav nav.launch.py), SLAM Toolbox
    # is skipped — AMCL handles the map→odom TF instead.
    include_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "slam.launch.py")),
        launch_arguments={
            "use_localization": "true",
            "slam_params":      slam_params,
            "ekf_params":       ekf_params,
            "log_level":        log_level,
        }.items(),
        condition=LaunchConfigurationEquals("slam", "true"),
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

    # Optional keyboard teleoperation — runs in an xterm so it can read stdin.
    # Publishes to /cmd_vel, same topic as the joystick, so twist_to_drv
    # picks it up without any remapping.
    keyboard_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        prefix="xterm -e",
        condition=IfCondition(keyboard),
    )

    # ── Compose ───────────────────────────────────────────────────────────────
    return LaunchDescription([

        DeclareLaunchArgument("log_level",   default_value="info"),
        DeclareLaunchArgument("slam",        default_value="true",
                              description="Launch SLAM Toolbox for mapping (false = skip SLAM, e.g. when using AMCL nav)"),
        DeclareLaunchArgument("esp_port",    default_value=ESP_PORT,
                              description="ESP32 serial device"),
        DeclareLaunchArgument("baud",        default_value=BAUD,
                              description="ESP32 serial baud rate"),
        DeclareLaunchArgument("rviz",        default_value="false",
                              description="Launch RViz2 on this machine"),
        DeclareLaunchArgument("keyboard",    default_value="true",
                              description="Launch teleop_twist_keyboard in an xterm window"),
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
        keyboard_node,
    ])
