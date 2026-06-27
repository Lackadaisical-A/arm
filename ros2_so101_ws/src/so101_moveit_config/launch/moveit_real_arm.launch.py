from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("so101_moveit_config"), "launch", "moveit_demo.launch.py"]
                    )
                ),
                launch_arguments={
                    "real_state": "true",
                    "stream_plan_udp": "true",
                    "rviz": LaunchConfiguration("rviz"),
                }.items(),
            )
        ]
    )
