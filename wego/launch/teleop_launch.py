from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    wego_share = get_package_share_path("wego")
    go2_base_share = get_package_share_path("go2_base")
    hesai_share = get_package_share_path("hesai_ros_driver")
    default_hesai_config = hesai_share / "config" / "config.yaml"

    interface_arg = DeclareLaunchArgument(
        "interface",
        default_value="eth0",
        description="Network interface used by the Unitree SDK.",
    )
    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="false",
        choices=["true", "false"],
        description="Start RViz with the Go2 and Hesai sensor configuration.",
    )
    start_go2_arg = DeclareLaunchArgument(
        "start_go2",
        default_value="true",
        choices=["true", "false"],
        description="Start the Go2 driver, camera, and robot-state publisher.",
    )
    start_hesai_arg = DeclareLaunchArgument(
        "start_hesai",
        default_value="true",
        choices=["true", "false"],
        description="Start the Hesai LiDAR driver.",
    )
    start_merger_arg = DeclareLaunchArgument(
        "start_merger",
        default_value="false",
        choices=["true", "false"],
        description="Optional: publish /merged/points from Go2 and Hesai point clouds.",
    )
    start_scan_arg = DeclareLaunchArgument(
        "start_scan",
        default_value="true",
        choices=["true", "false"],
        description="Convert the point cloud to /scan and publish /scan_filtered.",
    )
    cloud_topic_arg = DeclareLaunchArgument(
        "cloud_topic",
        default_value="/lidar_points",
        description="Point cloud topic converted to /scan.",
    )
    publish_odom_tf_arg = DeclareLaunchArgument(
        "publish_odom_tf",
        default_value="true",
        choices=["true", "false"],
        description="Publish odom -> base_footprint TF from go2_driver.",
    )
    hesai_config_arg = DeclareLaunchArgument(
        "hesai_config",
        default_value=str(default_hesai_config),
        description="Absolute path to Hesai driver config.yaml.",
    )
    hesai_frame_arg = DeclareLaunchArgument(
        "hesai_frame",
        default_value="hesai_lidar",
        description="Frame ID configured as ros_frame_id in Hesai config.yaml.",
    )
    hesai_x_arg = DeclareLaunchArgument(
        "hesai_x",
        default_value="0.13",
        description="Hesai origin X in base_link (metres).",
    )
    hesai_y_arg = DeclareLaunchArgument(
        "hesai_y",
        default_value="0.0",
        description="Hesai origin Y in base_link (metres).",
    )
    hesai_z_arg = DeclareLaunchArgument(
        "hesai_z",
        default_value="0.10",
        description="Hesai origin Z in base_link (metres); initial value from the former LiDAR mount.",
    )
    hesai_roll_arg = DeclareLaunchArgument(
        "hesai_roll",
        default_value="0.0",
        description="Hesai roll in base_link (radians).",
    )
    hesai_pitch_arg = DeclareLaunchArgument(
        "hesai_pitch",
        default_value="0.0",
        description="Hesai pitch in base_link (radians).",
    )
    hesai_yaw_arg = DeclareLaunchArgument(
        "hesai_yaw",
        default_value="1.57",
        description="Hesai yaw in base_link (radians).",
    )
    rviz_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=str(wego_share / "rviz" / "teleop.rviz"),
        description="Absolute path to the RViz configuration.",
    )

    go2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(go2_base_share / "launch" / "go2_bringup_launch.py")
        ),
        launch_arguments={
            "interface": LaunchConfiguration("interface"),
            "gui": "false",
            "publish_odom_tf": LaunchConfiguration("publish_odom_tf"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_go2")),
    )

    hesai_driver = Node(
        package="hesai_ros_driver",
        executable="hesai_ros_driver_node",
        name="hesai_ros_driver_node",
        output="screen",
        parameters=[{"config_path": LaunchConfiguration("hesai_config")}],
        condition=IfCondition(LaunchConfiguration("start_hesai")),
    )

    base_to_hesai = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_hesai_lidar",
        arguments=[
            LaunchConfiguration("hesai_x"),
            LaunchConfiguration("hesai_y"),
            LaunchConfiguration("hesai_z"),
            LaunchConfiguration("hesai_yaw"),
            LaunchConfiguration("hesai_pitch"),
            LaunchConfiguration("hesai_roll"),
            "base_link",
            LaunchConfiguration("hesai_frame"),
        ],
    )

    pointcloud_merger = Node(
        package="wego",
        executable="cloud_merger_node",
        name="pointcloud_merger",
        output="screen",
        parameters=[
            {
                "target_frame": "base_link",
                "go2_lidar_topic": "/go2/lidar_points",
                "secondary_lidar_topic": "/lidar_points",
            }
        ],
        condition=IfCondition(LaunchConfiguration("start_merger")),
    )

    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[
            str(wego_share / "config" / "pointcloud_to_laserscan.yaml")
        ],
        remappings=[
            ("/cloud_in", LaunchConfiguration("cloud_topic")),
            ("/scan", "/scan"),
        ],
        condition=IfCondition(LaunchConfiguration("start_scan")),
    )

    scan_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="scan_to_scan_filter_chain",
        output="screen",
        parameters=[str(wego_share / "config" / "laser_filter.yaml")],
        remappings=[
            ("scan", "/scan"),
            ("scan_filtered", "/scan_filtered"),
        ],
        condition=IfCondition(LaunchConfiguration("start_scan")),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    return LaunchDescription(
        [
            interface_arg,
            gui_arg,
            start_go2_arg,
            start_hesai_arg,
            start_merger_arg,
            start_scan_arg,
            cloud_topic_arg,
            publish_odom_tf_arg,
            hesai_config_arg,
            hesai_frame_arg,
            hesai_x_arg,
            hesai_y_arg,
            hesai_z_arg,
            hesai_roll_arg,
            hesai_pitch_arg,
            hesai_yaw_arg,
            rviz_arg,
            # go2_bringup receives gui:=false to prevent a duplicate RViz.
            # In Foxy that included argument can overwrite the same launch
            # configuration in the parent scope, so evaluate this node first.
            rviz_node,
            go2_bringup,
            base_to_hesai,
            hesai_driver,
            pointcloud_merger,
            pointcloud_to_laserscan,
            scan_filter,
        ]
    )
