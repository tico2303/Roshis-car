import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    bringup = get_package_share_directory("daro_bringup")

    lidar_launch = os.path.join(bringup, "launch", "lidar.launch.py")
    default_slam_params = os.path.join(bringup, "config", "slam.yaml")
    default_rviz_config = os.path.join(bringup, "rviz", "slam.rviz")

    slam_params = LaunchConfiguration("slam_params")
    rviz_config = LaunchConfiguration("rviz_config")

    # LiDAR + base_link->laser TF comes from lidar.launch.py (robot_state_publisher)
    include_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch),
    )

    # Temp: fake odom->base_link so SLAM has an odom chain even without encoders
    static_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_odom_to_base_link",
        namespace="",
        output="screen",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "odom", "--child-frame-id", "base_link"],
    )

    # slam_toolbox is LifecycleNode -> it will NOT publish /map until Activated
    slam_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="", 
        output="screen",
        emulate_tty=True,
        parameters=[ParameterFile(slam_params, allow_substs=True)],
    )

    # lifecycle manager configures + activates slam_toolbox
    lifecycle_mgr = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "autostart": True,
            # IMPORTANT: nav2 lifecycle manager expects node names (usually without leading '/')
            "node_names": ["slam_toolbox"],
            # Give it time 
            "bond_timeout": 20.0,
            "attempt_respawn_reconnection": True,
        }],
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
            "slam_params",
            default_value=default_slam_params,
            description="Path to slam_toolbox YAML params file",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="Path to RViz config file",
        ),

        include_lidar,
        static_odom_tf,

        slam_node,
        lifecycle_mgr,

        rviz_node,
    ])