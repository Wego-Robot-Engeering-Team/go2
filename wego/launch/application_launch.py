from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    wego_base_path = get_package_share_path('wego')
    wego_nav_base_path = get_package_share_path('wego_2d_nav')
    april_app_base_path = get_package_share_path('apriltag_application')

    navi_app = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(wego_base_path / 'launch/nav2_bringup_launch.py'))
    )

    apriltag_app = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(april_app_base_path / 'launch/marker_application_launch.py'))
    )

    waypoint_follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(wego_nav_base_path / 'launch/waypoint_follower_launch.py'))
    )

    state_machine = Node(
        package='wego_state_machine',
        executable='state_machine_node',
        arguments=['--ros-args', '--log-level', 'fatal'],
        output='screen'
    )

    alarm = Node(
    package='scenario_alarm',
    executable='state_sound_node',
    name='state_sound',
    arguments=['--ros-args', '--log-level', 'fatal'],
    output='screen'
    )

    head_n_wait = Node(
        package="head_n_wait",
        executable="head_n_wait",
        name="head_n_wait"
    )

    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{'port': 9090}],   # 필요시 포트 변경
        arguments=['--ros-args', '--log-level', 'fatal'],
        output='screen'
    )

    return LaunchDescription([
        navi_app,
        apriltag_app,
        waypoint_follower,
        state_machine,
        alarm,
        head_n_wait,
        rosbridge
    ])