from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    use_esp32_bridge = LaunchConfiguration("use_esp32_bridge")
    esp32_port = LaunchConfiguration("esp32_port")
    esp32_baud = LaunchConfiguration("esp32_baud")

    bringup_share = get_package_share_directory("daro_bringup")

    default_teleop = os.path.join(bringup_share, "config", "teleop_joy.yaml")
    default_buttons_params = os.path.join(bringup_share, "config", "joy_buttons_params.yaml")
    default_buttons_map = os.path.join(bringup_share, "config", "joy_buttons_map.yaml")

    joy_dev = LaunchConfiguration("joy_dev")
    teleop_params = LaunchConfiguration("teleop_params")
    buttons_params = LaunchConfiguration("buttons_params")
    buttons_map = LaunchConfiguration("buttons_map")

    use_actuation = LaunchConfiguration("use_actuation")

    # Actuation tuning args (safe defaults)
    thr_max = LaunchConfiguration("thr_max")
    str_max = LaunchConfiguration("str_max")
    cmd_timeout_s = LaunchConfiguration("cmd_timeout_s")
    publish_hz = LaunchConfiguration("publish_hz")

    return LaunchDescription([
        # -------------------- Inputs --------------------
        DeclareLaunchArgument(
            "use_esp32_bridge",
            default_value="true",
            description="If true, start daro_ndjson_bridge to talk to ESP32 over serial",
        ),
        DeclareLaunchArgument(
            "esp32_port",
            default_value="/dev/ttyACM0",
            description="Serial port for ESP32 (e.g., /dev/ttyACM0 or /dev/ttyUSB0)",
        ),
        DeclareLaunchArgument(
            "esp32_baud",
            default_value="115200",
            description="ESP32 serial baud rate",
        ),
        DeclareLaunchArgument(
            "joy_dev",
            default_value="/dev/input/event5",
            description="evdev device for Xbox controller (e.g., /dev/input/event5)",
        ),
        DeclareLaunchArgument(
            "teleop_params",
            default_value=default_teleop,
            description="teleop_twist_joy params YAML",
        ),
        DeclareLaunchArgument(
            "buttons_params",
            default_value=default_buttons_params,
            description="daro_inputs joy_buttons params YAML",
        ),
        DeclareLaunchArgument(
            "buttons_map",
            default_value=default_buttons_map,
            description="Button mapping YAML (dict) for joy_buttons_node",
        ),

        # -------------------- Actuation --------------------
        DeclareLaunchArgument(
            "use_actuation",
            default_value="true",
            description="If true, start twist_to_drv to publish /esp32/tx_json",
        ),
        DeclareLaunchArgument("thr_max", default_value="100", description="Throttle max (int)"),
        DeclareLaunchArgument("str_max", default_value="100", description="Steering max (int)"),
        DeclareLaunchArgument("cmd_timeout_s", default_value="0.25", description="Stop if cmd_vel stale (seconds)"),
        DeclareLaunchArgument("publish_hz", default_value="20.0", description="Publish rate to ESP32 (Hz)"),

        # 1) joy_node: controller -> /joy
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[{
                "dev": joy_dev,
                "deadzone": 0.05,
                "autorepeat_rate": 0.0,
            }],
        ),

        # 2) teleop_twist_joy: /joy -> /cmd_vel
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy_node",
            output="screen",
            parameters=[teleop_params],
        ),

        # 3) joy_buttons: /joy -> /daro/events/*
        Node(
            package="daro_inputs",
            executable="joy_buttons",
            name="joy_buttons",
            output="screen",
            parameters=[buttons_params, {"mapping_file": buttons_map}],
        ),

        # 4) twist_to_drv: /cmd_vel -> /esp32/tx_json
        Node(
            condition=IfCondition(use_actuation),
            package="daro_actuation",
            executable="twist_to_drv",
            name="twist_to_drv",
            output="screen",
            parameters=[{
                "cmd_vel_topic": "/cmd_vel",
                "tx_topic": "/esp32/tx_json",
                "thr_max": thr_max,
                "str_max": str_max,
                "cmd_timeout_s": cmd_timeout_s,
                "publish_hz": publish_hz,
                # leave signs default; flip later if needed:
                # "thr_sign": 1,
                # "str_sign": 1,
            }],
        ),
        # bridge from ros to esp32 serial
         Node(
            condition=IfCondition(use_esp32_bridge),
            package="daro_ndjson_bridge",
            executable="ndjson_bridge",
            name="ndjson_bridge",
            output="screen",
            parameters=[{
                "port": esp32_port,
                "baud": esp32_baud,
            }],
        ),
    ])