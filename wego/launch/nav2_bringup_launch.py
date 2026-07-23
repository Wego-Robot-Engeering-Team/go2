import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, LogInfo,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    wego_base_path = get_package_share_directory('wego')
    bringup_dir = get_package_share_directory('wego_2d_nav')
    launch_dir = os.path.join(bringup_dir, 'launch')

    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    interface = LaunchConfiguration('interface')
    start_teleop = LaunchConfiguration('start_teleop')
    cloud_topic = LaunchConfiguration('cloud_topic')

    gui_arg = DeclareLaunchArgument(
        'gui_nav',
        default_value='true',
        description='Flag to enable RViz and joint_state_publisher_gui'
    )

    declare_rviz_file_cmd = DeclareLaunchArgument(
        'navigation_rviz_config_file',
        default_value=os.path.join(wego_base_path, 'rviz', 'navigation.rviz'),
        description='Full path to the rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('navigation_rviz_config_file'),
                   '--ros-args', '--log-level', log_level],
        condition=IfCondition(LaunchConfiguration('gui_nav')),
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'map1.yaml'),
        description='Full path to map yaml file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='Log level for RViz and all Nav2 nodes')

    declare_interface_cmd = DeclareLaunchArgument(
        'interface',
        default_value='eth0',
        description='Network interface used by the Unitree SDK')

    declare_start_teleop_cmd = DeclareLaunchArgument(
        'start_teleop',
        default_value='true',
        choices=['true', 'false'],
        description='Start Go2/Hesai teleop bringup before Nav2')

    declare_cloud_topic_cmd = DeclareLaunchArgument(
        'cloud_topic',
        default_value='/lidar_points',
        description='Point cloud topic converted to /scan when start_teleop is true')

    launch_config_log = LogInfo(msg=[
        '[wego nav2] map=', map_yaml_file,
        ', params_file=', params_file,
        ', autostart=', autostart,
        ', log_level=', log_level,
        ', start_teleop=', start_teleop,
        ', cloud_topic=', cloud_topic,
        ', interface=', interface,
    ])

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wego_base_path, 'launch', 'teleop_launch.py')),
        launch_arguments={
            'interface': interface,
            'gui': 'false',
            'publish_odom_tf': 'true',
            'cloud_topic': cloud_topic,
            'start_scan': 'true',
        }.items(),
        condition=IfCondition(start_teleop),
    )

    # Foxy Nav2 launches standalone nodes.  Later Nav2 releases use the
    # composable-node container that this package previously started here.
    bringup_cmd_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir,
                                                       'localization_launch.py')),
            launch_arguments={'map': map_yaml_file,
                              'autostart': autostart,
                              'params_file': params_file,
                              'log_level': log_level}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'autostart': autostart,
                              'params_file': params_file,
                              'log_level': log_level}.items()),
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(gui_arg)
    ld.add_action(declare_rviz_file_cmd)
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_interface_cmd)
    ld.add_action(declare_start_teleop_cmd)
    ld.add_action(declare_cloud_topic_cmd)

    delayed_nav2_bringup = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='[wego nav2] Starting Nav2 after waiting for Go2 TF'),
            bringup_cmd_group,
        ],
    )

    delayed_rviz = TimerAction(
        period=9.0,
        actions=[
            LogInfo(msg='[wego nav2] Starting RViz after Nav2 bringup'),
            rviz_node,
        ],
    )

    # Add the actions to launch all of the navigation nodes
    ld.add_action(launch_config_log)
    ld.add_action(teleop_launch)
    ld.add_action(delayed_nav2_bringup)
    ld.add_action(delayed_rviz)

    return ld
