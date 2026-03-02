#!/usr/bin/env python3
"""
nav2_stack.launch.py — Nav2 nodes only (AMCL + map_server + planners + BT + lifecycle).

Launches all Nav2 lifecycle nodes directly (no nav2_bringup dependency).
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    daro_nav_share = get_package_share_directory("daro_nav")

    # ── Launch arguments ──────────────────────────────────────────────────────

    map_yaml     = LaunchConfiguration("map")
    params       = LaunchConfiguration("nav2_params")
    log_level    = LaunchConfiguration("log_level")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ── Lifecycle node list (managed by lifecycle_manager) ────────────────────
    lifecycle_nodes = [
        "map_server",
        "amcl",
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "velocity_smoother",
    ]

    # Common kwargs shared by every lifecycle node
    def nav2_node(executable, name=None, **kwargs):
        return Node(
            package="nav2_" + executable.replace("_server", "").replace("_navigator", "").replace("bt_", "bt_"),
            executable=executable,
            name=name or executable,
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            parameters=[params, {"use_sim_time": use_sim_time}],
            arguments=["--ros-args", "--log-level", log_level],
            **kwargs,
        )

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[params, {"use_sim_time": use_sim_time, "yaml_filename": map_yaml}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[params, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[params, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    smoother_server_node = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[params, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    planner_server_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[params, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    behavior_server_node = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[params, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    bt_navigator_node = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[params, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    velocity_smoother_node = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[params, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # ── Lifecycle manager: activates all Nav2 nodes in order ──────────────────
    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": lifecycle_nodes,
        }],
        arguments=["--ros-args", "--log-level", log_level],
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

        map_server_node,
        amcl_node,
        controller_server_node,
        smoother_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        velocity_smoother_node,
        lifecycle_manager_node,
    ])
