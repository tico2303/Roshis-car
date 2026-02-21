#!/usr/bin/env python3
"""
sim.launch.py — Gazebo Harmonic simulation for DARO (no physical hardware needed).

What runs:
  - Gazebo with test_world.sdf (boxes, walls, pillar)
  - daro.sdf robot spawned at origin (blue box, two wheels, simulated LiDAR)
  - gz_bridge: /scan, /odom, /cmd_vel, /tf, /clock, /joint_states
  - slam_common: robot_state_publisher, base_link→laser TF, SLAM Toolbox
  - Optional RViz

Usage:
  ros2 launch daro_bringup sim.launch.py
  ros2 launch daro_bringup sim.launch.py rviz:=true

Drive with Xbox controller (same as real robot):
  Left stick Y = forward/back, Right stick X = turn

View on macOS Docker:
  rviz2 -d <path>/daro_bringup/rviz/slam.rviz
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    bringup     = get_package_share_directory("daro_bringup")
    description = get_package_share_directory("daro_description")

    world_file       = os.path.join(bringup,     "worlds", "test_world.sdf")
    robot_sdf        = os.path.join(description, "urdf",   "daro.sdf")
    bridge_yaml      = os.path.join(bringup,     "config", "gz_bridge.yaml")
    slam_common_launch = os.path.join(bringup,   "launch", "slam_common.launch.py")
    rviz_cfg         = os.path.join(bringup,     "rviz",   "slam.rviz")

    use_rviz  = LaunchConfiguration("rviz")
    log_level = LaunchConfiguration("log_level")

    # ── 1. Gazebo world ───────────────────────────────────────────────────────
    # -s = server only (no GUI), --headless-rendering = EGL software rendering
    # DISPLAY= forces EGL path so ogre2 works without X/GPU for sensor rendering
    # Run GUI separately with: gz sim -g
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", world_file, "-r", "-s", "--headless-rendering", "-v", "4"],
        output="screen",
        additional_env={"DISPLAY": ""},
    )

    # ── 2. Spawn DARO into the world ──────────────────────────────────────────
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_daro",
        output="screen",
        arguments=[
            "-file", robot_sdf,
            "-name", "daro",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.001",
        ],
    )

    # ── 3. Gazebo <-> ROS2 topic bridge ───────────────────────────────────────
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        parameters=[{
            "config_file": bridge_yaml,
        }],
    )

    # ── 4. Shared SLAM stack (RSP, TF, SLAM Toolbox) ─────────────────────────
    # Gazebo DiffDrive publishes odom→base_link via gz_bridge, so we don't
    # need the static identity TF (publish_odom_tf:=false).
    include_slam_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_common_launch),
        launch_arguments={
            "use_sim_time":    "true",
            "publish_odom_tf": "false",
            "log_level":       log_level,
        }.items(),
    )

    # ── 5. Optional RViz ──────────────────────────────────────────────────────
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
                              description="Launch RViz2 locally"),

        # Gazebo world first
        gazebo,

        # Spawn robot once Gazebo is up
        TimerAction(period=3.0, actions=[spawn_robot]),

        # Bridge + SLAM stack after robot is spawned (needs Gazebo topics)
        TimerAction(period=5.0, actions=[
            gz_bridge,
            include_slam_common,
            rviz_node,
        ]),
    ])
