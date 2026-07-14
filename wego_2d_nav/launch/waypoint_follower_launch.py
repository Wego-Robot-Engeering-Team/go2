import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    wego_2d_nav_base_dir = get_package_share_directory('wego_2d_nav')
    wego_2d_nav_config_dir = os.path.join(wego_2d_nav_base_dir, 'config')
    param_file = os.path.join(wego_2d_nav_config_dir, 'waypoint.yaml')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='wego_2d_nav',
            executable='waypoint_save',
            parameters=[param_file],
            ),
        launch_ros.actions.Node(
            package='wego_2d_nav',
            executable='waypoint_runner',
            parameters=[param_file],
            )
    ])