#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from daro_bringup.defaults import ESP_PORT, BAUD


def generate_launch_description():
    esp_port = LaunchConfiguration("esp_port")
    baud = LaunchConfiguration("baud")
    namespace = LaunchConfiguration("namespace")
    log_level = LaunchConfiguration("log_level")

    # Kill any stale ndjson_bridge process left over from a hard shutdown.
    # We use pkill by name rather than fuser on the symlink (/dev/esp32)
    # because the stale process holds the real device (/dev/ttyUSB0) and
    # fuser on a symlink won't find it. pkill exits 1 if nothing matched,
    # which is fine — the "|| true" swallows that so the launch continues.
    release_port = ExecuteProcess(
        cmd=["bash", "-c", "pkill -f ndjson_bridge || true"],
        output="screen",
    )

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

        # Kill any stale port holder first, then wait briefly for the OS to
        # release the file descriptor before the bridge node opens it.
        release_port,
        TimerAction(
            period=1.0,
            actions=[
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
            ],
        ),
    ])
