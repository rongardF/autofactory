# launch ros_gz_bridge with bridged Gazebo services and then launch 'gazebo_client.py' node to handle mating/detaching of parts in Gazebo

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from tools_manager.utils.config_reader import read_config_file


def _bridge_setup(context, *args, **kwargs):
    """Bridge each slot's Gazebo RFID scanner tag data 
    topic into ROS when simulating.
    """
    if LaunchConfiguration("simulated").perform(context).lower() != 'true':
        return []

    config_file = LaunchConfiguration("tools_manager_config_file").perform(context)
    config = read_config_file(config_file)

    # bridge tool rack slots RFID scanner tag data topics into ROS
    bridge_arguments = [
        f'/slot{slot.index}_rfid_scanner/tag_data@ros_gz_interfaces/msg/Dataframe[gz.msgs.Dataframe'
        for slot in config.slots
    ]

    # bridge tool mount RFID scanner tag data topic into ROS
    bridge_arguments += [
        f"tool_mount_rfid_scanner/tag_data@ros_gz_interfaces/msg/Dataframe[gz.msgs.Dataframe"
    ]

    # bridge Gazebo services for spawning, setting pose, and deleting entities
    world_name = LaunchConfiguration("world_name").perform(context)
    bridge_arguments += [
        f'/world/{world_name}/create@ros_gz_interfaces/srv/SpawnEntity',
        f'/world/{world_name}/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        f'/world/{world_name}/remove@ros_gz_interfaces/srv/DeleteEntity',
    ]

    if not bridge_arguments:
        return []

    return [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=bridge_arguments,
            output='screen',
        )
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'simulated',
            default_value='true',
            choices=['true', 'false'],
            description='Simulation mode enabled or not.',
        ),
        DeclareLaunchArgument(
            'station_model_name',
            default_value='station',
            description='Station model name.',
        ),
        DeclareLaunchArgument(
            'movement_controller_node_name',
            default_value='movement_controller',
            description='Name of the movement controller node.',
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='world',
            description='Name of the Gazebo world.',
        ),
        DeclareLaunchArgument(
            'tools_manager_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare("tools_manager"),
                "config",
                "tools_manager_config.yaml",
            ]),
            description='Path to the tool rack config YAML file.',
        ),
        DeclareLaunchArgument(
            'tool_mount_parent_frame_id',
            default_value='tool0',
            description='Parent frame ID for the tool mount.',
        ),
        DeclareLaunchArgument(
            'tool_mounted_update_rate',
            default_value='10.0',
            description='Update rate for the tool mount slots.',
        ),
        DeclareLaunchArgument(
            'tool_rack_parent_frame_id',
            default_value='station',
            description='Parent frame ID for the tool rack.',
        ),
        DeclareLaunchArgument(
            'tool_rack_slots_update_rate',
            default_value='10.0',
            description='Update rate for the tool rack slots.',
        ),
    ]

    simulated = LaunchConfiguration("simulated")

    actions = []

    # if simulated, bridge each slot's Gazebo RFID scanner detection topic into ROS
    actions.append(OpaqueFunction(function=_bridge_setup))

    actions.append(
        Node(
            package="tools_manager",
            executable="tool_mount",
            name="tool_mount",
            output="screen",
            parameters=[
                {
                    "simulated": simulated,
                    "use_sim_time": simulated,
                    "parent_frame_id": LaunchConfiguration("tool_mount_parent_frame_id"),
                    "mounted_publish_rate": LaunchConfiguration("tool_mounted_update_rate"),
                },
            ],
        )
    )
    actions.append(
        Node(
            package="tools_manager",
            executable="tool_rack",
            name="tool_rack",
            output="screen",
            parameters=[
                {
                    "simulated": simulated,
                    "use_sim_time": simulated,
                    "tools_manager_config_file": LaunchConfiguration("tools_manager_config_file"),
                    "parent_frame_id": LaunchConfiguration("tool_rack_parent_frame_id"),
                    "slots_update_rate": LaunchConfiguration("tool_rack_slots_update_rate"),
                },
            ],
        )
    )
    actions.append(
        Node(
            package="tools_manager",
            executable="tools_manager",
            name="tools_manager",
            output="screen",
            parameters=[
                {
                    "simulated": simulated,
                    "use_sim_time": simulated,
                    "world_name": LaunchConfiguration("world_name"),
                    "station_model_name": LaunchConfiguration("station_model_name"),
                    "tool_mount_node_name": "tool_mount",
                    "tool_rack_node_name": "tool_rack",
                    "movement_controller_node_name": LaunchConfiguration("movement_controller_node_name"),
                    "tools_manager_config_file": LaunchConfiguration("tools_manager_config_file"),
                },
            ],
        )
    )

    return LaunchDescription(declared_arguments + actions)