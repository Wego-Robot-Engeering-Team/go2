import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    wego_base_path = get_package_share_directory('wego')
    bringup_dir = get_package_share_directory('wego_2d_nav')
    launch_dir = os.path.join(bringup_dir, 'launch')

    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    map_mask_yaml_file = LaunchConfiguration('map_mask')
    mask_params_file = LaunchConfiguration('mask_params_file')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    
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
                   '--ros-args', '--log-level', 'fatal'],
        condition=IfCondition(LaunchConfiguration('gui_nav')),
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'map.yaml'),
        description='Full path to map yaml file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_map_mask_yaml_cmd = DeclareLaunchArgument(
        'map_mask',
        default_value=os.path.join(bringup_dir, 'maps', 'map_speed_mask.yaml'),
        description='Full path to map yaml file to load')

    declare_mask_param_file_cmd = DeclareLaunchArgument(
        'mask_params_file',
        default_value=os.path.join(bringup_dir, 'config', 'speed_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='fatal',
        description='log level')

    # Specify the actions
    bringup_cmd_group = GroupAction([
        Node(
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[params_file, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir,
                                                       'localization_launch.py')),
            launch_arguments={'map': map_yaml_file,
                              'map_mask': map_mask_yaml_file,
                              'mask_params_file':  mask_params_file,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'autostart': autostart,
                              'params_file': params_file,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(gui_arg)
    ld.add_action(declare_rviz_file_cmd)
    ld.add_action(rviz_node)
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_mask_param_file_cmd)
    ld.add_action(declare_map_mask_yaml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld