import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    bringup_dir = get_package_share_directory('wego_2d_nav')
    map_yaml_file = LaunchConfiguration('map') 
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    mask_params_file = LaunchConfiguration('mask_params_file')
    mask_yaml_file = LaunchConfiguration('map_mask')
    container_name = LaunchConfiguration('container_name')
    # lifecycle_nodes = ['map_server', 'amcl', 'speed_filter_mask_server', 'speed_costmap_filter_info_server']
    lifecycle_nodes = ['map_server', 'amcl']

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        description='Full path to map yaml file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_mask_params_file_cmd = DeclareLaunchArgument(
        'mask_params_file',
        description='Full path to the ROS2 parameters file to use')
    
    declare_mask_yaml_file_cmd = DeclareLaunchArgument(
        'map_mask',
        description='Full path to filter mask yaml file to load')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    # Make re-written yaml
    param_substitutions = {
        'yaml_filename': mask_yaml_file}

    mask_configured_params = RewrittenYaml(
        source_file=mask_params_file,
        param_rewrites=param_substitutions,
        convert_types=True)
    
    load_composable_nodes = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[{'yaml_filename': map_yaml_file}],
                remappings=remappings),
            ComposableNode(
                package='nav2_amcl',
                plugin='nav2_amcl::AmclNode',
                name='amcl',
                parameters=[params_file],
                remappings=remappings),
            # ComposableNode(
            #             package='nav2_map_server',
            #             plugin='nav2_map_server::MapServer',
            #             name='speed_filter_mask_server',
            #             parameters=[
            #                 mask_configured_params,
            #                 {'yaml_filename': mask_yaml_file}
            #             ],
            #             remappings=[
            #                 *remappings,
            #                 ('map', '/speed_filter_mask'),
            #                 ('map_metadata', '/speed_filter_mask_metadata')
            #             ],
            #         ),
            # ComposableNode(
            #     package='nav2_map_server',
            #     plugin='nav2_map_server::CostmapFilterInfoServer',
            #     name='speed_costmap_filter_info_server',
            #     parameters=[mask_configured_params, {'mask_topic': '/speed_filter_mask'}]),
           ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_localization',
                parameters=[{'autostart': autostart,
                             'node_names': lifecycle_nodes}]),
        ],
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    # ld.add_action(declare_mask_params_file_cmd)
    # ld.add_action(declare_mask_yaml_file_cmd)

    ld.add_action(load_composable_nodes)

    return ld