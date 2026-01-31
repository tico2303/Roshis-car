#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ---------- Launch args ----------
    esp_port   = LaunchConfiguration("esp_port")
    baud       = LaunchConfiguration("baud")
    log_level  = LaunchConfiguration("log_level")

    rx_topic   = LaunchConfiguration("rx_topic")
    range_topic = LaunchConfiguration("range_topic")

    # ---------- Params file ----------
    bringup_share = get_package_share_directory("daro_bringup")
    default_params = os.path.join(bringup_share, "config", "tof_test.yaml")

    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "esp_port",
            default_value="/dev/ttyUSB0",
            description="ESP32 serial device (e.g. /dev/ttyUSB0 or /dev/ttyACM0)",
        ),
        DeclareLaunchArgument(
            "baud",
            default_value="115200",
            description="ESP32 serial baud rate",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS log level: debug|info|warn|error|fatal",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="YAML params file for tof test stack",
        ),
        DeclareLaunchArgument(
            "rx_topic",
            default_value="/esp32/rx_json/tof",
            description="Input topic from NDJSON bridge",
        ),
        DeclareLaunchArgument(
            "range_topic",
            default_value="/tof/range",
            description="Output Range topic",
        ),

        # ---------- 1) NDJSON Bridge ----------
        Node(
            package="daro_ndjson_bridge",
            executable="ndjson_bridge",
            name="ndjson_bridge_node",
            output="screen",
            parameters=[{
                "port": esp_port,
                "baud": baud,
            }],
            arguments=["--ros-args", "--log-level", log_level],
        ),

        # ---------- 2) ToF JSON -> Range ----------
        Node(
            package="daro_sensors",
            executable="tof_to_range",
            name="tof_to_range_node",
            output="screen",
            # Load defaults from YAML, but allow topic overrides from launch args
            parameters=[
                params_file,
                {
                    "rx_topic": rx_topic,
                    "pub_topic": range_topic,
                }
            ],
            arguments=["--ros-args", "--log-level", log_level],
        ),
    ])