#!/usr/bin/env python3
"""
sim.launch.py — Gazebo Harmonic simulation for DARO (no physical hardware needed).

What runs:
  - Gazebo with test_world.sdf (boxes, walls, pillar)
  - daro.sdf robot spawned at origin (blue box, two wheels, simulated LiDAR)
  - gz_bridge: /scan, /odom, /cmd_vel, /tf, /clock, /joint_states
  - robot_state_publisher + static base_link->laser TF
  - SLAM Toolbox (builds map from simulated LiDAR)
  - joy + teleop_twist_joy (drive with Xbox controller, same as real robot)
  - Optional RViz

Usage:
  ros2 launch daro_bringup sim.launch.py

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
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, LifecycleNode
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():

    bringup     = get_package_share_directory("daro_bringup")
    description = get_package_share_directory("daro_description")

    world_file  = os.path.join(bringup,     "worlds", "test_world.sdf")
    robot_sdf   = os.path.join(description, "urdf",   "daro.sdf")
    urdf_file   = os.path.join(description, "urdf",   "daro_min.urdf")
    slam_params = os.path.join(bringup,     "config", "slam.yaml")
    bridge_yaml = os.path.join(bringup,     "config", "gz_bridge.yaml")
    rviz_cfg    = os.path.join(bringup,     "rviz",   "slam.rviz")

    use_rviz  = LaunchConfiguration("rviz")
    log_level = LaunchConfiguration("log_level")

    # ── 1. Gazebo world ───────────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", world_file, "-r"],   # -r = start running immediately
        output="screen",
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
    # Uses a YAML config for unambiguous direction control.
    # See config/gz_bridge.yaml for the full mapping.
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_yaml}],
    )

    # ── 4. robot_state_publisher ──────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": open(urdf_file).read(),
            "use_sim_time": True,
        }],
    )

    # Explicit static TF for base_link->laser (same fix as real robot —
    # robot_state_publisher broadcasts with zero timestamp which SLAM rejects)
    base_link_to_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_laser",
        output="screen",
        arguments=["0.08", "0", "0.10", "0", "0", "0", "base_link", "laser"],
    )

    # ── 5. SLAM Toolbox ───────────────────────────────────────────────────────
    slam_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[
            ParameterFile(slam_params, allow_substs=True),
            {"use_sim_time": True},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    configure_slam = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "configure"],
        output="screen",
    )
    activate_slam = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "activate"],
        output="screen",
    )

    # NOTE: joy_node + teleop_node run on the Pi (where the controller is connected).
    # They publish /cmd_vel over DDS which the bridge forwards to Gazebo.

    # ── 6. Optional RViz ──────────────────────────────────────────────────────
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

        # Spawn robot + start all ROS2 nodes once Gazebo is up
        TimerAction(period=2.0, actions=[
            spawn_robot,
            gz_bridge,
            robot_state_publisher,
            base_link_to_laser,
            rviz_node,
        ]),

        # SLAM needs TF from the diff-drive bridge to be flowing first
        TimerAction(period=5.0,  actions=[slam_node]),
        TimerAction(period=7.0,  actions=[configure_slam]),
        TimerAction(period=9.0,  actions=[activate_slam]),
    ])
