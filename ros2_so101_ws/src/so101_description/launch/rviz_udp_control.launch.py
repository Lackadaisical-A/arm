from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = FindPackageShare("so101_description")
    default_rviz = PathJoinSubstitution([pkg_share, "rviz", "so101.rviz"])
    pkg_path = get_package_share_directory("so101_description")
    urdf_path = os.path.join(pkg_path, "urdf", "so101_6dof_rviz.urdf")

    target_ip_arg = DeclareLaunchArgument(
        "target_ip",
        default_value="",
        description="Windows host IP. Empty means auto-detect from /etc/resolv.conf.",
    )
    target_port_arg = DeclareLaunchArgument("target_port", default_value="50101")
    bind_ip_arg = DeclareLaunchArgument("bind_ip", default_value="0.0.0.0")
    bind_port_arg = DeclareLaunchArgument("bind_port", default_value="50102")
    gui_arg = DeclareLaunchArgument("gui", default_value="true")

    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = {"robot_description": f.read()}

    return LaunchDescription(
        [
            target_ip_arg,
            target_port_arg,
            bind_ip_arg,
            bind_port_arg,
            gui_arg,
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
                    }
                ],
                output="screen",
            ),
            Node(
                package="so101_description",
                executable="calibrated_joint_slider_gui.py",
                condition=IfCondition(LaunchConfiguration("gui")),
                parameters=[
                    {
                        "state_topic": "/so101_lerobot_state",
                        "limits_topic": "/so101_lerobot_limits",
                        "command_topic": "/lerobot_slider_positions",
                    }
                ],
                output="screen",
            ),
            Node(
                package="so101_description",
                executable="joint_states_udp_bridge.py",
                parameters=[
                    {
                        "target_ip": LaunchConfiguration("target_ip"),
                        "target_port": LaunchConfiguration("target_port"),
                        "source_topic": "/lerobot_slider_positions",
                        "input_mode": "lerobot",
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
