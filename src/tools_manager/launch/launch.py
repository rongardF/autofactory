# launch ros_gz_bridge with bridged Gazebo services and then launch 'gazebo_client.py' node to handle mating/detaching of parts in Gazebo

from os import environ

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import IfElseSubstitution, LaunchConfiguration
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, IfElseSubstitution
)


def generate_launch_description():
    gazebo_gui = LaunchConfiguration("gazebo_gui", default="true")
    simulated = LaunchConfiguration("simulated", default="true")
    world_name = LaunchConfiguration("world_name", default="station")
    world_file = LaunchConfiguration("world_file", default=FindPackageShare("station") + "/config/world/station.sdf")

    # TODO: remove this
    # update GZ_SIM_RESOURCE_PATH env variable to include the path to the Gazebo models in this package
    environ['GZ_SIM_RESOURCE_PATH'] = f"/workspaces/autofactory/gazebo_sandbox/models:{environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    
    # spawn Gazebo sim, transport bridge and Gazebo client
    actions =[
        Node(
            package="tools_manager",
            executable="tool_mount",
            output="screen",
            parameters=[
                {
                    "simulated": simulated,
                    "use_sim_time": simulated,
                },
            ],
        ),
        Node(
            package="tools_manager",
            executable="tool_rack",
            output="screen",
            parameters=[
                {
                    "simulated": simulated,
                    "use_sim_time": simulated,
                    "config_file": FindPackageShare("tools_manager") + "/config/tool_rack_config.yaml",
                },
            ],
        )
    ]

    if simulated:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("tools_manager"), "/launch/endtools_launch.py"]
                ),
                launch_arguments={
                    "simulated": simulated,
                    "config_file": FindPackageShare("tools_manager") + "/config/tool_rack_config.yaml",
                    "world_name": world_name,
                    "world_file": world_file,
                    "gazebo_gui": gazebo_gui,
                }.items(),
            )
        )

    return LaunchDescription(actions)