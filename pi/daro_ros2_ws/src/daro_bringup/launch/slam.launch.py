#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():

    # -------------------------------------------------
    # Package paths
    # -------------------------------------------------
    bringup = get_package_share_directory("daro_bringup")

    lidar_launch = os.path.join(bringup, "launch", "lidar.launch.py")
    localization_launch = os.path.join(bringup, "launch", "localization.launch.py")
    default_slam_params = os.path.join(bringup, "config", "slam.yaml")

    # -------------------------------------------------
    # Launch arguments
    # -------------------------------------------------
    slam_params = LaunchConfiguration("slam_params")
    ekf_params  = LaunchConfiguration("ekf_params")
    log_level = LaunchConfiguration("log_level")
    use_localization = LaunchConfiguration("use_localization")

    # -------------------------------------------------
    # Include LiDAR + robot_state_publisher (always)
    # -------------------------------------------------
    include_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch),
        launch_arguments={"log_level": log_level}.items(),
    )

    # -------------------------------------------------
    # Include EKF (robot_localization) — only when
    # odometry sources (wheel odom + IMU) are available,
    # i.e. when daro.launch.py is also running.
    # use_localization:=true  -> EKF publishes odom->base_link
    # use_localization:=false -> static identity odom->base_link
    # -------------------------------------------------
    include_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={
            "ekf_params": ekf_params,
            "log_level":  log_level,
        }.items(),
        condition=IfCondition(use_localization),
    )

    # Static odom -> base_link identity transform used when EKF is not running.
    # SLAM Toolbox will correct map->odom; odom->base_link stays identity until
    # real odometry is wired up.
    static_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_odom_base_link",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
        condition=UnlessCondition(use_localization),
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
        parameters=[ParameterFile(slam_params, allow_substs=True)],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # -------------------------------------------------
    # Lifecycle transitions — delay long enough for TF
    # tree to be established before SLAM starts consuming
    # scans.  With static TF this is fast; with EKF we
    # give it an extra couple of seconds.
    # -------------------------------------------------
    configure_slam = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "configure"],
        output="screen",
    )

    activate_slam = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "activate"],
        output="screen",
    )

    # -------------------------------------------------
    # Launch Description
    # -------------------------------------------------
    return LaunchDescription([

        DeclareLaunchArgument(
            "slam_params",
            default_value=default_slam_params,
            description="Path to slam_toolbox YAML params file",
        ),

        DeclareLaunchArgument(
            "ekf_params",
            default_value=os.path.join(bringup, "config", "ekf.yaml"),
            description="Path to robot_localization EKF params YAML",
        ),

        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS log level",
        ),

        DeclareLaunchArgument(
            "use_localization",
            default_value="false",
            description=(
                "true  = start EKF node (needs /wheel/odom + /imu/data_raw from daro.launch.py); "
                "false = use a static identity odom->base_link so SLAM can run standalone"
            ),
        ),

        # Sensors + (conditionally) EKF or static TF
        include_lidar,
        include_localization,
        static_odom_tf,

        # SLAM Toolbox
        slam_node,

        # Give TF tree ~3 s to settle before configure, then 2 more before activate
        TimerAction(period=3.0, actions=[configure_slam]),
        TimerAction(period=5.0, actions=[activate_slam]),
    ])
