#!/usr/bin/env python3
"""
sim.launch.py — Gazebo simulation for DARO (no physical hardware needed).

Replaces the ESP32 bridge + LiDAR driver with simulated equivalents.
SLAM and EKF still run exactly as in robot.launch.py so you can test
navigation without the physical robot.

Usage:
  ros2 launch daro_bringup sim.launch.py

Then open Gazebo (it launches automatically) and RViz on your Mac:
  rviz2 -d <path>/daro_bringup/rviz/slam.rviz
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():

    bringup = get_package_share_directory("daro_bringup")
    launch_dir = os.path.join(bringup, "launch")
    world_file = os.path.join(bringup, "worlds", "test_world.sdf")
    slam_params = os.path.join(bringup, "config", "slam.yaml")
    ekf_params  = os.path.join(bringup, "config", "ekf.yaml")
    rviz_cfg    = os.path.join(bringup, "rviz", "slam.rviz")

    use_rviz  = LaunchConfiguration("rviz")
    log_level = LaunchConfiguration("log_level")

    # ── Gazebo ────────────────────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", world_file, "-r"],   # -r = start running immediately
        output="screen",
    )

    # ── ROS2 <-> Gazebo bridge ────────────────────────────────────────────────
    # Bridges the simulated LiDAR scan from Gazebo to /scan in ROS2.
    # Extend this list if you add more sensors to the SDF.
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )

    # ── Static TF: base_link -> laser (matches URDF / test_world laser pose) ─
    base_link_to_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_laser",
        arguments=["0.08", "0", "0.10", "0", "0", "0", "base_link", "laser"],
    )

    # ── Static TF: odom -> base_link (sim has no wheel encoders yet) ──────────
    # Replace with a proper odometry source once the robot SDF has drive plugins.
    static_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_odom_base_link",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
    )

    # ── robot_state_publisher (publishes /robot_description for RViz) ─────────
    description_share = get_package_share_directory("daro_description")
    urdf_file = os.path.join(description_share, "urdf", "daro_min.urdf")
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": open(urdf_file).read()}],
    )

    # ── SLAM Toolbox ──────────────────────────────────────────────────────────
    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        emulate_tty=True,
        parameters=[
            ParameterFile(slam_params, allow_substs=True),
            {"use_sim_time": True},   # use Gazebo clock
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # ── Optional RViz ─────────────────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_cfg],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([

        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("rviz",      default_value="false",
                              description="Launch RViz2 on this machine"),

        # Gazebo first
        gazebo,

        # Give Gazebo 2s to start before bringing up ROS2 nodes
        TimerAction(period=2.0, actions=[
            gz_bridge,
            robot_state_publisher,
            base_link_to_laser,
            static_odom_tf,
        ]),

        # SLAM needs TF to be ready — start after another 2s
        TimerAction(period=4.0, actions=[slam_node]),

        rviz_node,
    ])
