import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    wego_share = get_package_share_directory("wego")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wego_share, "launch", "teleop_launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("include_bringup")),
        launch_arguments={
            "interface": LaunchConfiguration("interface"),
            "gui": LaunchConfiguration("gui"),
            "publish_odom_tf": "true",
            "cloud_topic": LaunchConfiguration("cloud_topic"),
            "start_scan": "true",
        }.items(),
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[os.path.join(wego_share, "config", "slam_offline.yaml")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "include_bringup",
                default_value="true",
                choices=["true", "false"],
                description="Start the current Go2 + Hesai bringup before SLAM.",
            ),
            DeclareLaunchArgument(
                "interface",
                default_value="eth0",
                description="Network interface used by the Unitree SDK.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                choices=["true", "false"],
                description="Start RViz from teleop_launch.py.",
            ),
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/lidar_points",
                description="Hesai point cloud topic converted to LaserScan.",
            ),
            bringup,
            slam_toolbox,
        ]
    )
