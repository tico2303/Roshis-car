import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = get_package_share_directory("daro_bringup")
    description_share = get_package_share_directory("daro_description")

    default_params = os.path.join(bringup_share, "config", "lidar.yaml")
    default_urdf = os.path.join(description_share, "urdf", "daro_min.urdf")

    params_file = LaunchConfiguration("params_file")
    urdf_file = LaunchConfiguration("urdf_file")

    # --- LiDAR driver ---
    lidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar",
        output="screen",
        parameters=[params_file],
        respawn=True,
        respawn_delay=2.0,
    )

    # --- TF from URDF --- (possibly remove later and just keep in daro.launch)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": open(default_urdf, "r").read()
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to lidar YAML params file",
        ),
        DeclareLaunchArgument(
            "urdf_file",
            default_value=default_urdf,
            description="Path to robot URDF used for TF via robot_state_publisher",
        ),

        # NOTE: We read default_urdf above. If you want urdf_file overrides to work,
        # replace open(default_urdf, ...) with open(urdf_file.perform(...)) pattern via OpaqueFunction.
        # For now: keep it simple and stable.

        robot_state_publisher,
        lidar_node,

    ])