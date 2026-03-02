#!/usr/bin/env python3
"""
nav2_stack.launch.py — Nav2 nodes only (AMCL + map_server + planners + BT + lifecycle).

Launches all Nav2 lifecycle nodes directly (no nav2_bringup dependency).
Designed to be included by nav.launch.py, or run standalone for development.
The hardware stack (EKF, LiDAR, etc.) must already be running.

Usage (standalone):
  ros2 launch daro_nav nav2_stack.launch.py map:=/home/pi/maps/my_map.yaml
  ros2 launch daro_nav nav2_stack.launch.py map:=~/maps/my_map.yaml   # ~ supported

Usage (via nav.launch.py — preferred):
  ros2 launch daro_nav nav.launch.py map:=/home/pi/maps/my_map.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_nodes(context, *args, **kwargs):
    """
    OpaqueFunction callback — runs at launch time so LaunchConfiguration
    values are fully resolved and we can call os.path.expanduser on the
    map path.
    """
    map_yaml     = os.path.expanduser(LaunchConfiguration("map").perform(context))
    params       = LaunchConfiguration("nav2_params").perform(context)
    log_level    = LaunchConfiguration("log_level").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    sim_time_bool = use_sim_time.lower() in ("true", "1", "yes")

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

    common = dict(
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
    )

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[params, {"use_sim_time": sim_time_bool, "yaml_filename": map_yaml}],
        **common,
    )

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        parameters=[params, {"use_sim_time": sim_time_bool}],
        **common,
    )

    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        parameters=[params, {"use_sim_time": sim_time_bool}],
        **common,
    )

    smoother_server_node = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        parameters=[params, {"use_sim_time": sim_time_bool}],
        **common,
    )

    planner_server_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        parameters=[params, {"use_sim_time": sim_time_bool}],
        **common,
    )

    behavior_server_node = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        parameters=[params, {"use_sim_time": sim_time_bool}],
        **common,
    )

    bt_navigator_node = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        parameters=[params, {"use_sim_time": sim_time_bool}],
        **common,
    )

    velocity_smoother_node = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        parameters=[params, {"use_sim_time": sim_time_bool}],
        **common,
    )

    # ── Lifecycle manager: activates all Nav2 nodes in order ──────────────────
    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": sim_time_bool,
            "autostart": True,
            "node_names": lifecycle_nodes,
        }],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return [
        map_server_node,
        amcl_node,
        controller_server_node,
        smoother_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        velocity_smoother_node,
        lifecycle_manager_node,
    ]


def generate_launch_description():

    daro_nav_share = get_package_share_directory("daro_nav")

    return LaunchDescription([

        DeclareLaunchArgument(
            "map",
            description="Full path to the map .yaml file — ~ is expanded automatically",
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

        OpaqueFunction(function=_make_nodes),
    ])
