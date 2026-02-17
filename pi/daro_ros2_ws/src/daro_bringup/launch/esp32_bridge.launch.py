#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from daro_bringup.defaults import ESP_PORT, BAUD


def generate_launch_description():
    esp_port = LaunchConfiguration("esp_port")
    baud = LaunchConfiguration("baud")
    namespace = LaunchConfiguration("namespace")
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription([
        DeclareLaunchArgument(
            "esp_port",
            default_value=ESP_PORT,
            description="Serial port for ESP32 (e.g. /dev/ttyUSB0, /dev/ttyACM0).",
        ),
        DeclareLaunchArgument(
            "baud",
            default_value=BAUD,
            description="ESP32 serial baud rate.",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="esp32",
            description="Namespace for ESP32 topics (e.g. /esp32/...).",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS log level (debug, info, warn, error, fatal).",
        ),

        Node(
            package="daro_ndjson_bridge",
            executable="ndjson_bridge",
            name="ndjson_bridge_node",
            namespace=namespace,
            output="screen",

            parameters=[{
                "port": esp_port,
                "baud": baud,
            }],

            arguments=["--ros-args", "--log-level", log_level],
        ),
    ])
