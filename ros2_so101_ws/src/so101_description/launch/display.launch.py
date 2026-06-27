import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("so101_description")
    default_rviz = PathJoinSubstitution([pkg_share, "rviz", "so101.rviz"])
    pkg_path = get_package_share_directory("so101_description")
    urdf_path = os.path.join(pkg_path, "urdf", "so101_6dof_rviz.urdf")

    gui_arg = DeclareLaunchArgument("gui", default_value="true")

    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = {"robot_description": f.read()}

    return LaunchDescription(
        [
            gui_arg,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                condition=IfCondition(LaunchConfiguration("gui")),
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
