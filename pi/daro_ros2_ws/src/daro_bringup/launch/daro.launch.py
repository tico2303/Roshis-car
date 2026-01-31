#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    log_level = LaunchConfiguration("log_level")
    params_file = LaunchConfiguration("params_file")

    bringup_share = get_package_share_directory("daro_bringup")
    cfg_dir = os.path.join(bringup_share, "config")
    launch_dir = os.path.join(bringup_share, "launch")

    default_params = os.path.join(cfg_dir, "daro.yaml")
    teleop_joy_yaml = os.path.join(cfg_dir, "teleop_joy.yaml")

    esp32_bridge_launch = os.path.join(launch_dir, "esp32_bridge.launch.py")

    twist_to_drv_yaml = os.path.join(cfg_dir, "twist_to_drv.yaml")

    joy_buttons_params_yaml = os.path.join(cfg_dir, "joy_buttons_params.yaml")

    # --- Include ESP32 bridge launch ---
    # For now: keep esp_port/baud as launch args (runtime override friendly).
    esp_port = LaunchConfiguration("esp_port")
    baud = LaunchConfiguration("baud")

    esp32_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(esp32_bridge_launch),
        launch_arguments={
            "esp_port": esp_port,
            "baud": baud,
            "namespace": "esp32",
            "log_level": log_level,
        }.items(),
    )

    # --- Nodes ---
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    teleop_twist_joy = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy",
        output="screen",
        parameters=[teleop_joy_yaml],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Update these executables if your console_scripts names differ:
    joy_buttons_node = Node(
        package="daro_inputs",
        executable="joy_buttons",
        name="joy_buttons_node",
        output="screen",
        parameters=[joy_buttons_params_yaml],
        arguments=["--ros-args", "--log-level", log_level],
    )

    twist_to_drv_node = Node(
        package="daro_actuation",
        executable="twist_to_drv",
        name="twist_to_drv",
        output="screen",
        parameters=[twist_to_drv_yaml],
        arguments=["--ros-args", "--log-level", log_level],
    )

    tof_to_range_node = Node(
        package="daro_sensors",
        executable="tof_to_range",
        name="tof_to_range_node",
        output="screen",
        parameters=[params_file],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS log level",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to DARO params YAML",
        ),

        # Keep these as launch args because serial port changes frequently.
        DeclareLaunchArgument(
            "esp_port",
            default_value="/dev/ttyUSB0",
            description="ESP32 serial device",
        ),
        DeclareLaunchArgument(
            "baud",
            default_value="115200",
            description="ESP32 serial baud rate",
        ),

        esp32_bridge,
        joy_node,
        teleop_twist_joy,
        joy_buttons_node,
        twist_to_drv_node,
        tof_to_range_node,
    ])