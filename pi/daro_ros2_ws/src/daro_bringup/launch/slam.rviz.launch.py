import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory("daro_bringup")

    lidar_launch       = os.path.join(bringup, "launch", "lidar.launch.py")
    slam_common_launch = os.path.join(bringup, "launch", "slam_common.launch.py")
    default_rviz_config = os.path.join(bringup, "rviz", "slam.rviz")

    rviz_config = LaunchConfiguration("rviz_config")

    # LiDAR hardware driver
    include_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch),
    )

    # Shared SLAM stack (RSP, TF, SLAM Toolbox with lifecycle)
    include_slam_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_common_launch),
        launch_arguments={
            "use_sim_time":    "false",
            "publish_odom_tf": "true",
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace="",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="Path to RViz config file",
        ),

        include_lidar,
        include_slam_common,
        rviz_node,
    ])
