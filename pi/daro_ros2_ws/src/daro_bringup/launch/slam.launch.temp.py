import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = get_package_share_directory("daro_bringup")

    lidar_launch       = os.path.join(bringup, "launch", "lidar.launch.py")
    slam_common_launch = os.path.join(bringup, "launch", "slam_common.launch.py")

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

    return LaunchDescription([
        include_lidar,
        include_slam_common,
    ])
