#!/usr/bin/env python3
"""
slam_common.launch.py — Shared SLAM stack used by both sim and real robot.

Provides:
  - robot_state_publisher (URDF TF)
  - base_link→laser static TF
  - SLAM Toolbox lifecycle node (configure + activate)
  - Optional static odom→base_link identity (when no EKF or Gazebo)

Does NOT include a lidar driver — the caller is responsible for /scan:
  - Real robot: lidar.launch.py starts sllidar_node
  - Simulation: Gazebo publishes /scan via gz_bridge
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

from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():

    bringup     = get_package_share_directory("daro_bringup")
    description = get_package_share_directory("daro_description")

    default_slam_params = os.path.join(bringup,     "config", "slam.yaml")
    default_urdf        = os.path.join(description, "urdf",   "daro_min.urdf")

    slam_params      = LaunchConfiguration("slam_params")
    log_level        = LaunchConfiguration("log_level")
    use_sim_time     = LaunchConfiguration("use_sim_time")
    publish_odom_tf  = LaunchConfiguration("publish_odom_tf")

    # -------------------------------------------------
    # robot_state_publisher (URDF → TF)
    # -------------------------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": open(default_urdf, "r").read(),
            "use_sim_time": use_sim_time,
        }],
    )

    # -------------------------------------------------
    # Static TF: base_link → laser
    # robot_state_publisher publishes this from the URDF but uses a zero
    # timestamp which SLAM Toolbox rejects.  A dedicated publisher fixes it.
    # Values match laser_joint origin in daro_min.urdf: xyz="0.08 0 0.10"
    # -------------------------------------------------
    base_link_to_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_laser",
        output="screen",
        arguments=["0.08", "0", "0.10", "0", "0", "0", "base_link", "laser"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # -------------------------------------------------
    # Static odom → base_link identity (when no EKF / no Gazebo DiffDrive)
    # Callers that get odom→base_link from elsewhere (EKF, Gazebo) should
    # set publish_odom_tf:=false.
    # -------------------------------------------------
    static_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_odom_base_link",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(publish_odom_tf),
    )

    # -------------------------------------------------
    # SLAM Toolbox Lifecycle Node
    # -------------------------------------------------
    slam_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[
            ParameterFile(slam_params, allow_substs=True),
            {"use_sim_time": use_sim_time},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # -------------------------------------------------
    # Lifecycle transitions — delay for TF tree to settle
    # -------------------------------------------------
    configure_slam = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "configure"],
        output="screen",
    )
    activate_slam = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "activate"],
        output="screen",
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "slam_params",
            default_value=default_slam_params,
            description="Path to slam_toolbox YAML params file",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS log level",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock (/clock topic)",
        ),
        DeclareLaunchArgument(
            "publish_odom_tf",
            default_value="true",
            description=(
                "Publish static identity odom→base_link. Set false when "
                "EKF or Gazebo DiffDrive provides this transform."
            ),
        ),

        robot_state_publisher,
        base_link_to_laser,
        static_odom_tf,

        slam_node,

        TimerAction(period=3.0, actions=[configure_slam]),
        TimerAction(period=5.0, actions=[activate_slam]),
    ])
