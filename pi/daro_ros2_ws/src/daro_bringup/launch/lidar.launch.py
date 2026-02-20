import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = get_package_share_directory("daro_bringup")
    description_share = get_package_share_directory("daro_description")

    default_lidar_params = os.path.join(bringup_share, "config", "lidar.yaml")
    default_urdf = os.path.join(description_share, "urdf", "daro_min.urdf")

    # Use a specific arg name (lidar_params) to avoid colliding with the
    # generic "params_file" arg declared by daro.launch.py, which defaults
    # to daro.yaml and would otherwise be inherited when this launch file is
    # included transitively via robot.launch.py -> slam.launch.py -> here.
    lidar_params = LaunchConfiguration("lidar_params")
    log_level = LaunchConfiguration("log_level")

    # --- LiDAR driver ---
    lidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar",
        output="screen",
        parameters=[lidar_params],
        respawn=True,
        respawn_delay=2.0,
    )

    # --- TF from URDF ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": open(default_urdf, "r").read()
        }],
    )

    # --- Explicit static TF: base_link -> laser ---
    # robot_state_publisher publishes this from the URDF but uses a zero
    # timestamp, which SLAM Toolbox rejects during TF lookups.
    # A dedicated static_transform_publisher fixes this — it broadcasts
    # with wall-clock time and is always retained in the TF buffer.
    # Values must match the laser_joint origin in daro_min.urdf:
    #   xyz="0.08 0 0.10" rpy="0 0 0"
    base_link_to_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_laser",
        output="screen",
        arguments=["0.08", "0", "0.10", "0", "0", "0", "base_link", "laser"],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "lidar_params",
            default_value=default_lidar_params,
            description="Path to lidar YAML params file",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS log level",
        ),

        robot_state_publisher,
        base_link_to_laser,
        lidar_node,
    ])
