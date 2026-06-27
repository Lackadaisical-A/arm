import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("so101_description")
    default_rviz = PathJoinSubstitution([pkg_share, "rviz", "so101.rviz"])
    pkg_path = get_package_share_directory("so101_description")
    urdf_path = os.path.join(pkg_path, "urdf", "so101_6dof_rviz.urdf")

    bind_ip_arg = DeclareLaunchArgument("bind_ip", default_value="0.0.0.0")
    bind_port_arg = DeclareLaunchArgument("bind_port", default_value="50102")
    unwrap_wraparound_arg = DeclareLaunchArgument("unwrap_wraparound", default_value="true")
    wrap_jump_ratio_arg = DeclareLaunchArgument("wrap_jump_ratio", default_value="0.75")
    clip_to_urdf_limits_arg = DeclareLaunchArgument("clip_to_urdf_limits", default_value="false")

    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = {"robot_description": f.read()}

    return LaunchDescription(
        [
            bind_ip_arg,
            bind_port_arg,
            unwrap_wraparound_arg,
            wrap_jump_ratio_arg,
            clip_to_urdf_limits_arg,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="so101_description",
                executable="real_state_udp_receiver.py",
                parameters=[
                    {
                        "bind_ip": LaunchConfiguration("bind_ip"),
                        "bind_port": LaunchConfiguration("bind_port"),
                        "unwrap_wraparound": ParameterValue(
                            LaunchConfiguration("unwrap_wraparound"), value_type=bool
                        ),
                        "wrap_jump_ratio": ParameterValue(
                            LaunchConfiguration("wrap_jump_ratio"), value_type=float
                        ),
                        "clip_to_urdf_limits": ParameterValue(
                            LaunchConfiguration("clip_to_urdf_limits"), value_type=bool
                        ),
                    }
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", default_rviz],
                output="screen",
            ),
        ]
    )
