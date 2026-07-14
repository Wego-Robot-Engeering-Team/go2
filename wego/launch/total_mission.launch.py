import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Waypoint Follower Launch 포함하기
    # (ros2 launch wego_2d_nav waypoint_follower_launch.py)
    # 주의: wego_2d_nav 패키지 안에 launch 폴더가 있고, 그 안에 파일이 있어야 합니다.
    waypoint_follower_dir = get_package_share_directory('wego_2d_nav')
    waypoint_follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(waypoint_follower_dir, 'launch', 'waypoint_follower_launch.py')
        )
    )

    # 2. Head n Wait 노드
    # (ros2 run head_n_wait head_n_wait)
    head_n_wait_node = Node(
        package='head_n_wait',
        executable='head_n_wait',
        name='head_n_wait',
        output='screen'
    )

    # 3. State Machine 노드
    # (ros2 run wego_state_machine state_machine_node2)
    state_machine_node = Node(
        package='wego_state_machine',
        executable='state_machine_node2',
        name='state_machine_node',
        output='screen'
    )

    # 4. Sound Alarm 노드
    # (ros2 run scenario_alarm state_sound_node)
    sound_node = Node(
        package='scenario_alarm',
        executable='state_sound_node',
        name='state_sound_node',
        output='screen'
    )

    return LaunchDescription([
        waypoint_follower_launch,
        head_n_wait_node,
        state_machine_node,
        sound_node
    ])