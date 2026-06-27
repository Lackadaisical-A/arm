from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "time_scale",
                default_value="1.0",
                description="Trajectory playback speed multiplier. 2.0 executes in half the planned time.",
            ),
            DeclareLaunchArgument("controller_rate_hz", default_value="50.0"),
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
                    "time_scale": LaunchConfiguration("time_scale"),
                    "controller_rate_hz": LaunchConfiguration("controller_rate_hz"),
                }.items(),
            )
        ]
    )
