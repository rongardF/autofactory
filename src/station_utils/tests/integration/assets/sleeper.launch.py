"""Test launch file: one process that records its PID and sleeps."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    pid_file = LaunchConfiguration('pid_file')
    duration = LaunchConfiguration('duration')
    return LaunchDescription([
        DeclareLaunchArgument('pid_file'),
        DeclareLaunchArgument('duration', default_value='60'),
        ExecuteProcess(
            cmd=['bash', '-c', ['echo $$ > ', pid_file, '; exec sleep ', duration]],
            output='screen',
        ),
    ])
