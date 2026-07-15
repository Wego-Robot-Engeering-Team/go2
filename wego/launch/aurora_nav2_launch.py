import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    home_dir = os.environ.get("HOME", "/tmp")
    wego_share = get_package_share_directory("wego")
    wego_nav_share = get_package_share_directory("wego_2d_nav")
    go2_base_share = get_package_share_directory("go2_base")
    slamware_share = get_package_share_directory("slamware_ros_sdk")

    default_map = os.path.join(
        home_dir,
        "wego_ws",
        "src",
        "aurora",
        "aurora_test",
        "maps",
        "nav2",
        "aurora_pointcloud_map.yaml",
    )
    default_stcm = os.path.join(
        home_dir,
        "wego_ws",
        "src",
        "aurora",
        "aurora_test",
        "maps",
        "aurora_map.stcm",
    )
    default_aurora_root = os.path.join(home_dir, "wego_ws", "src", "aurora")

    ip_address = LaunchConfiguration("ip_address")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")
    gui_nav = LaunchConfiguration("gui_nav")
    use_native_scan = LaunchConfiguration("use_native_scan")
    load_stcm = LaunchConfiguration("load_stcm")
    include_go2_driver = LaunchConfiguration("include_go2_driver")
    go2_interface = LaunchConfiguration("go2_interface")
    aurora_root = LaunchConfiguration("aurora_root")
    stcm_file = LaunchConfiguration("stcm_file")
    relocalize_after_load = LaunchConfiguration("relocalize_after_load")

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            "odom_topic": "/slamware_ros_sdk_server_node/odom",
        },
        convert_types=True,
    )

    aurora_server = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(slamware_share, "launch", "slamware_ros_sdk_server_node.xml")
        ),
        launch_arguments={"ip_address": ip_address}.items(),
    )

    aurora_map_manager = Node(
        package="aurora_test",
        executable="vslam_map_manager",
        name="aurora_vslam_map_manager",
        output="screen",
        condition=IfCondition(load_stcm),
        parameters=[
            {
                "device_ip": ip_address,
                "aurora_root": aurora_root,
                "map_file": stcm_file,
                "load_on_startup": True,
                "save_on_shutdown": False,
                "relocalize_after_load": ParameterValue(relocalize_after_load, value_type=bool),
                "session_timeout_sec": 300.0,
                "relocalization_timeout_ms": 15000,
            }
        ],
    )

    go2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(go2_base_share, "launch", "go2_bringup_launch.py")),
        condition=IfCondition(include_go2_driver),
        launch_arguments={
            "interface": go2_interface,
            "gui": "false",
        }.items(),
    )

    aurora_base_to_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aurora_base_to_base_link",
        arguments=[
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "aurora_base",
            "base_link",
        ],
    )

    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        condition=UnlessCondition(use_native_scan),
        parameters=[os.path.join(wego_share, "config", "pointcloud_to_laserscan.yaml")],
        remappings=[
            ("/cloud_in", "/slamware_ros_sdk_server_node/point_cloud"),
            ("/scan", "/scan"),
        ],
    )

    native_scan_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="scan_to_scan_filter_chain",
        output="screen",
        condition=IfCondition(use_native_scan),
        parameters=[os.path.join(wego_share, "config", "laser_filter.yaml")],
        remappings=[
            ("scan", "/slamware_ros_sdk_server_node/scan"),
            ("scan_filtered", "/scan_filtered"),
        ],
    )

    generated_scan_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="scan_to_scan_filter_chain",
        output="screen",
        condition=UnlessCondition(use_native_scan),
        parameters=[os.path.join(wego_share, "config", "laser_filter.yaml")],
        remappings=[
            ("scan", "/scan"),
            ("scan_filtered", "/scan_filtered"),
        ],
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(wego_share, "launch", "nav2_bringup_launch.py")),
        launch_arguments={
            "map": map_yaml,
            "params_file": configured_params,
            "autostart": autostart,
            "log_level": log_level,
            "gui_nav": gui_nav,
        }.items(),
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument("ip_address", default_value="192.168.11.1"),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(wego_nav_share, "config", "nav2_params.yaml"),
            ),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="fatal"),
            DeclareLaunchArgument("gui_nav", default_value="true"),
            DeclareLaunchArgument("use_native_scan", default_value="false"),
            DeclareLaunchArgument("load_stcm", default_value="true"),
            DeclareLaunchArgument("include_go2_driver", default_value="false"),
            DeclareLaunchArgument("go2_interface", default_value="enp88s0"),
            DeclareLaunchArgument("aurora_root", default_value=default_aurora_root),
            DeclareLaunchArgument("stcm_file", default_value=default_stcm),
            DeclareLaunchArgument("relocalize_after_load", default_value="true"),
            aurora_server,
            aurora_map_manager,
            go2_bringup,
            aurora_base_to_base_link,
            pointcloud_to_laserscan,
            native_scan_filter,
            generated_scan_filter,
            nav2_bringup,
        ]
    )
