from __future__ import annotations

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_file(package_name: str, relative_path: str) -> str:
    package_path = get_package_share_directory(package_name)
    path = os.path.join(package_path, relative_path)
    with open(path, "r", encoding="utf-8-sig") as file:
        return file.read()


def load_yaml(package_name: str, relative_path: str) -> dict:
    package_path = get_package_share_directory(package_name)
    path = os.path.join(package_path, relative_path)
    with open(path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file) or {}


def generate_launch_description():
    moveit_pkg = "so101_moveit_config"
    rviz_config = PathJoinSubstitution([FindPackageShare(moveit_pkg), "rviz", "moveit.rviz"])

    robot_description = {
        "robot_description": load_file("so101_description", "urdf/so101_6dof_rviz.urdf")
    }
    robot_description_semantic = {
        "robot_description_semantic": load_file(moveit_pkg, "config/so101.srdf")
    }
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(moveit_pkg, "config/kinematics.yaml")
    }
    robot_description_planning = {
        "robot_description_planning": load_yaml(moveit_pkg, "config/joint_limits.yaml")
    }

    planning_pipelines = load_yaml(moveit_pkg, "config/ompl_planning.yaml")
    trajectory_execution = load_yaml(moveit_pkg, "config/trajectory_execution.yaml")
    moveit_controllers = load_yaml(moveit_pkg, "config/moveit_controllers.yaml")
    planning_scene_monitor = load_yaml(moveit_pkg, "config/planning_scene_monitor.yaml")

    moveit_params = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        planning_pipelines,
        trajectory_execution,
        moveit_controllers,
        planning_scene_monitor,
    ]

    real_state_arg = DeclareLaunchArgument(
        "real_state",
        default_value="false",
        description="Use live Windows bridge state on /joint_states instead of fake replay state.",
    )
    stream_plan_udp_arg = DeclareLaunchArgument(
        "stream_plan_udp",
        default_value="false",
        description="Send replayed MoveIt plans to the Windows UDP bridge. Requires --accept-urdf-targets there.",
    )
    target_ip_arg = DeclareLaunchArgument(
        "target_ip",
        default_value="",
        description="Windows host IP for optional plan streaming. Empty means auto-detect from WSL route.",
    )
    target_port_arg = DeclareLaunchArgument("target_port", default_value="50101")
    bind_ip_arg = DeclareLaunchArgument("bind_ip", default_value="0.0.0.0")
    bind_port_arg = DeclareLaunchArgument("bind_port", default_value="50102")
    rviz_arg = DeclareLaunchArgument("rviz", default_value="true")
    time_scale_arg = DeclareLaunchArgument(
        "time_scale",
        default_value="1.0",
        description="Trajectory playback speed multiplier. 2.0 executes in half the planned time.",
    )
    controller_rate_arg = DeclareLaunchArgument(
        "controller_rate_hz",
        default_value="50.0",
        description="SO-101 trajectory controller update/UDP stream rate.",
    )

    return LaunchDescription(
        [
            real_state_arg,
            stream_plan_udp_arg,
            target_ip_arg,
            target_port_arg,
            bind_ip_arg,
            bind_port_arg,
            rviz_arg,
            time_scale_arg,
            controller_rate_arg,
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                    "--frame-id",
                    "world",
                    "--child-frame-id",
                    "base_link",
                ],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="so101_description",
                executable="real_state_udp_receiver.py",
                condition=IfCondition(LaunchConfiguration("real_state")),
                parameters=[
                    {
                        "bind_ip": LaunchConfiguration("bind_ip"),
                        "bind_port": LaunchConfiguration("bind_port"),
                    }
                ],
                output="screen",
            ),
            Node(
                package=moveit_pkg,
                executable="so101_trajectory_controller.py",
                condition=UnlessCondition(LaunchConfiguration("real_state")),
                parameters=[
                    {
                        "publish_joint_states": True,
                        "preview_plans": True,
                        "stream_udp": LaunchConfiguration("stream_plan_udp"),
                        "target_ip": LaunchConfiguration("target_ip"),
                        "target_port": LaunchConfiguration("target_port"),
                        "rate_hz": LaunchConfiguration("controller_rate_hz"),
                        "time_scale": LaunchConfiguration("time_scale"),
                    }
                ],
                output="screen",
            ),
            Node(
                package=moveit_pkg,
                executable="so101_trajectory_controller.py",
                condition=IfCondition(LaunchConfiguration("real_state")),
                parameters=[
                    {
                        "publish_joint_states": False,
                        "preview_plans": False,
                        "stream_udp": LaunchConfiguration("stream_plan_udp"),
                        "target_ip": LaunchConfiguration("target_ip"),
                        "target_port": LaunchConfiguration("target_port"),
                        "rate_hz": LaunchConfiguration("controller_rate_hz"),
                        "time_scale": LaunchConfiguration("time_scale"),
                    }
                ],
                output="screen",
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=moveit_params,
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                condition=IfCondition(LaunchConfiguration("rviz")),
                arguments=["-d", rviz_config],
                parameters=moveit_params,
                output="screen",
            ),
        ]
    )
