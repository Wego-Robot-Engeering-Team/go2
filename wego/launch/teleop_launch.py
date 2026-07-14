from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    gui_arg = DeclareLaunchArgument(
        'gui_teleop',
        default_value='false',
        description='Flag to enable RViz and joint_state_publisher_gui'
    )

    # Get paths
    wego_base_path = get_package_share_path('wego')
    go2_base_path = get_package_share_path('go2_base')
    default_rviz_config_path = wego_base_path / 'rviz/display.rviz'
    wego_parameters_file_dir = os.path.join(wego_base_path, 'config')
    apriltag_parameters_file_path = os.path.join(wego_parameters_file_dir, 'apriltag.yaml')
    livox_lidar_path = get_package_share_path('livox_ros_driver2')
    point_to_laser_parameters_file_path = os.path.join(wego_parameters_file_dir, 'pointcloud_to_laserscan.yaml')

    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    # go2 bringup
    go2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(go2_base_path / 'launch/go2_bringup_launch.py'))
    )

    apriltag= Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            parameters=[apriltag_parameters_file_path],
            remappings=[
                ('/camera_info', '/go2/camera/camera_info'),
                ('/image_rect', '/go2/camera/image_raw'),
            ],
        )
    
    # aurora_to_base_link= Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         arguments=[
    #             '--x', '0.23', 
    #             '--y', '0',
    #             '--z', '0.1',
    #             '--yaw', '0', 
    #             '--pitch', '0.0', 
    #             '--roll',  '0.0', 
    #             '--frame-id', 'base_link', 
    #             '--child-frame-id', 'aurora_base']
    #     )

    aurora_to_base_link= Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '-0.23', 
                '--y', '0',
                '--z', '-0.1',
                '--yaw', '0', 
                '--pitch', '0.0', 
                '--roll',  '0.0', 
                '--frame-id', 'aurora_base', 
                '--child-frame-id', 'base_footprint']
        )
    
    lidar_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(livox_lidar_path / 'launch_ROS2/msg_MID360_launch.py'))
    )

    point_to_laser= Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[point_to_laser_parameters_file_path],
            arguments=['--ros-args', '--log-level', 'fatal'],
            remappings=[
                ('/cloud_in', '/merged/points'),
            ],
        )

    laser_filter =IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(wego_base_path / 'launch/laser_filter_launch.py'))
        )

    lidar_msg_convert= Node(
            package='livox_to_pointcloud2',
            executable='livox_to_pointcloud2_node',
            arguments=['--ros-args', '--log-level', 'fatal'],
            name='livox_to_pointcloud',
        )
    
    lidar_merger= Node(
            package='wego',
            executable='cloud_merger_node',
            arguments=['--ros-args', '--log-level', 'fatal'],
            name='pointcloud_merger'
        )

    lidar_to_base_link= Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0.0', 
                '--y', '0',
                '--z', '0.2',
                '--yaw', '0', 
                '--pitch', '0.0', 
                '--roll',  '0.0', 
                '--frame-id', 'base_link', 
                '--child-frame-id', 'livox_frame']
        )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig'),
                   '--ros-args', '--log-level', 'fatal'
                   ],
        condition=IfCondition(LaunchConfiguration('gui_teleop')),
    )

    return LaunchDescription([
        rviz_arg,
        gui_arg,
        go2_bringup,
        apriltag,
        aurora_to_base_link,
        lidar_to_base_link,
        lidar_bringup,
        lidar_msg_convert,
        lidar_merger,
        point_to_laser,
        laser_filter,
        # robot_localization_bringup,
        rviz_node
    ])
