import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *_args, **_kwargs):
    network_interface = (
        LaunchConfiguration("network_interface")
        .perform(context)
        .strip()
    )

    params_file = Path(
        LaunchConfiguration("params_file").perform(context)
    )

    if not params_file.exists():
        raise FileNotFoundError(params_file)

    actions = []

    # Cyclone DDS 네트워크 인터페이스 설정
    if network_interface:
        actions.extend(
            [
                SetEnvironmentVariable(
                    "RMW_IMPLEMENTATION",
                    "rmw_cyclonedds_cpp",
                ),
                SetEnvironmentVariable(
                    "CYCLONEDDS_URI",
                    "<CycloneDDS>"
                    "<Domain>"
                    "<General>"
                    "<Interfaces>"
                    f'<NetworkInterface name="{network_interface}" '
                    'priority="default" multicast="default" />'
                    "</Interfaces>"
                    "</General>"
                    "</Domain>"
                    "</CycloneDDS>",
                ),
            ]
        )

    # Go2 URDF / robot_state_publisher / RViz
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("go2_description"),
                        "launch",
                        "go2_description.launch.py",
                    ]
                )
            ),
            launch_arguments={
                "description_file": LaunchConfiguration(
                    "description_file"
                ),
                "rviz_config": LaunchConfiguration("rviz_config"),
                "network_interface": LaunchConfiguration(
                    "network_interface"
                ),
                "rviz": LaunchConfiguration("rviz"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
            condition=IfCondition(
                LaunchConfiguration("enable_description")
            ),
        )
    )

    # cmd_vel → Unitree SDK 제어 명령
    actions.append(
        Node(
            package="go2_base",
            executable="go2_cmd_vel_bridge",
            name="go2_cmd_vel_bridge",
            output="screen",
            parameters=[
                str(params_file),
                {
                    "use_sim_time": LaunchConfiguration(
                        "use_sim_time"
                    )
                },
            ],
            condition=IfCondition(
                LaunchConfiguration("enable_control")
            ),
        )
    )

    # Unitree SDK 상태 → ROS 2 토픽
    actions.append(
        Node(
            package="go2_base",
            executable="go2_state_bridge",
            name="go2_state_bridge",
            output="screen",
            parameters=[
                str(params_file),
                {
                    "use_sim_time": LaunchConfiguration(
                        "use_sim_time"
                    ),
                    "rebase_odom_on_start": LaunchConfiguration(
                        "rebase_odom_on_start"
                    ),
                },
            ],
            condition=IfCondition(
                LaunchConfiguration("enable_bridge")
            ),
        )
    )

    # Receive XT16 UDP packets directly and publish the point cloud.
    actions.append(
        Node(
            package="hesai_ros_driver",
            executable="hesai_ros_driver_node",
            name="hesai_ros_driver_node",
            output="screen",
            parameters=[
                {
                    "config_path": LaunchConfiguration(
                        "hesai_config_file"
                    )
                }
            ],
            condition=IfCondition(
                LaunchConfiguration("enable_hesai")
            ),
        )
    )

    # Intel RealSense D435i camera and depth/colour/IMU topics.
    if LaunchConfiguration("enable_realsense").perform(context).lower() in (
        "true",
        "1",
        "yes",
    ):
        realsense_prefix = Path(
            LaunchConfiguration("realsense_prefix").perform(context)
        )
        realsense_node = (
            realsense_prefix
            / "lib"
            / "realsense2_camera"
            / "realsense2_camera_node"
        )

        if not realsense_node.exists():
            raise FileNotFoundError(
                "RealSense ROS node not found: "
                f"{realsense_node}. "
                "Set realsense_prefix:=... or disable it with "
                "enable_realsense:=false."
            )

        # The RealSense packages are installed in a user-local overlay on
        # this machine rather than /opt/ros/humble. Make them discoverable
        # by the included launch file and by the camera node's shared libs.
        current_ament = os.environ.get("AMENT_PREFIX_PATH", "")
        current_ld = os.environ.get("LD_LIBRARY_PATH", "")
        current_python = os.environ.get("PYTHONPATH", "")
        realsense_lib = str(realsense_prefix / "lib")
        realsense_arch_lib = str(realsense_prefix / "lib" / "aarch64-linux-gnu")
        realsense_python = str(
            realsense_prefix / "local" / "lib" / "python3.10" / "dist-packages"
        )

        actions.extend(
            [
                SetEnvironmentVariable(
                    "AMENT_PREFIX_PATH",
                    ":".join(filter(None, [str(realsense_prefix), current_ament])),
                ),
                SetEnvironmentVariable(
                    "LD_LIBRARY_PATH",
                    ":".join(
                        filter(None, [realsense_arch_lib, realsense_lib, current_ld])
                    ),
                ),
                SetEnvironmentVariable(
                    "PYTHONPATH",
                    ":".join(filter(None, [realsense_python, current_python])),
                ),
                Node(
                    package="realsense2_camera",
                    executable="realsense2_camera_node",
                    namespace="camera",
                    name=LaunchConfiguration("realsense_camera_name"),
                    output="screen",
                    parameters=[
                        {
                            "serial_no": LaunchConfiguration("realsense_serial_no"),
                            "enable_color": True,
                            "enable_depth": True,
                            # South Korea uses 60 Hz mains frequency.
                            "rgb_camera.power_line_frequency": 2,
                            "enable_infra": False,
                            "enable_infra1": False,
                            "enable_infra2": False,
                            "enable_gyro": LaunchConfiguration(
                                "realsense_enable_gyro"
                            ),
                            "enable_accel": LaunchConfiguration(
                                "realsense_enable_accel"
                            ),
                            "enable_sync": True,
                            "enable_rgbd": True,
                            "align_depth.enable": True,
                        }
                    ],
                    arguments=["--ros-args", "--log-level", "info"],
                ),
            ]
        )

    return actions


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [
            FindPackageShare("go2_base"),
            "config",
            "go2_driver_params.yaml",
        ]
    )

    default_description = PathJoinSubstitution(
        [
            FindPackageShare("go2_description"),
            "urdf",
            "go2_description.urdf",
        ]
    )

    default_rviz = PathJoinSubstitution(
        [
            FindPackageShare("go2_description"),
            "rviz",
            "go2.rviz",
        ]
    )

    default_hesai_config = PathJoinSubstitution(
        [
            FindPackageShare("hesai_ros_driver"),
            "config",
            "config.yaml",
        ]
    )

    default_realsense_prefix = os.environ.get(
        "REALSENSE_PREFIX",
        "/home/ktl/ktl_ws/realsense_overlay/opt/ros/humble",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
            ),
            DeclareLaunchArgument(
                "description_file",
                default_value=default_description,
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
            ),
            DeclareLaunchArgument(
                "network_interface",
                default_value="eno1",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "enable_control",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "enable_bridge",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "enable_description",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "rebase_odom_on_start",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "enable_hesai",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "hesai_config_file",
                default_value=default_hesai_config,
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "enable_realsense",
                default_value="true",
                description="Start the Intel RealSense D435i camera node.",
            ),
            DeclareLaunchArgument(
                "realsense_prefix",
                default_value=default_realsense_prefix,
                description="ROS prefix containing realsense2_camera.",
            ),
            DeclareLaunchArgument(
                "realsense_camera_name",
                default_value="d435i",
            ),
            DeclareLaunchArgument(
                "realsense_serial_no",
                default_value="_238222076093",
            ),
            DeclareLaunchArgument(
                "realsense_enable_gyro",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "realsense_enable_accel",
                default_value="true",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
