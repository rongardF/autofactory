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
from launch_ros.parameter_descriptions import ParameterValue


def endtool_description(
    xacro_file: str,
    model_name: str = "endtool",
    tool_mount_model: str = "tool_mount",
    tool_mount_child_link: str = "mount_link",
    tool_mount_topic_base: str = "/tool_mount",
    tool_rack_model: str = "tool_rack",
    tool_rack_child_link: str = "rack_link",
    tool_rack_topic_base: str = "/tool_rack",
    pp_publish_link_pose: bool = True,
    pp_publish_collision_pose: bool = False,
    pp_publish_visual_pose: bool = False,
    pp_publish_nested_model_pose: bool = False,
    pp_publish_model_pose: bool = True,
    pp_use_pose_vector_msg: bool = True,
    pp_update_frequency: int = 125,
) -> Command:
    """Return the expanded endtool SDF (from xacro) as a string substitution."""
    endtool_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
            " ",
            "model_name:=",
            model_name,
            " ",
            "tool_mount_model:=",
            tool_mount_model,
            " ",
            "tool_mount_child_link:=",
            tool_mount_child_link,
            " ",
            "tool_mount_topic_base:=",
            tool_mount_topic_base,
            " ",
            "tool_rack_model:=",
            tool_rack_model,
            " ",
            "tool_rack_child_link:=",
            tool_rack_child_link,
            " ",
            "tool_rack_topic_base:=",
            tool_rack_topic_base,
            " ",
            "pp_publish_link_pose:=",
            str(pp_publish_link_pose).lower(),
            " ",
            "pp_publish_collision_pose:=",
            str(pp_publish_collision_pose).lower(),
            " ",
            "pp_publish_visual_pose:=",
            str(pp_publish_visual_pose).lower(),
            " ",
            "pp_publish_nested_model_pose:=",
            str(pp_publish_nested_model_pose).lower(),
            " ",
            "pp_publish_model_pose:=",
            str(pp_publish_model_pose).lower(),
            " ",
            "pp_use_pose_vector_msg:=",
            str(pp_use_pose_vector_msg).lower(),
            " ",
            "pp_update_frequency:=",
            str(pp_update_frequency),
        ]
    )

    return endtool_description_content

def generate_launch_description():
    gazebo_gui = LaunchConfiguration("gazebo_gui", default="true")
    # update GZ_SIM_RESOURCE_PATH env variable to include the path to the Gazebo models in this package
    environ['GZ_SIM_RESOURCE_PATH'] = f"/workspaces/autofactory/gazebo_sandbox/models:{environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    
    # spawn Gazebo sim, transport bridge and Gazebo client
    return LaunchDescription([
        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-string",
                endtool_description(
                    xacro_file="/workspaces/autofactory/gazebo_sandbox/models/tool_side/endtool.xacro",
                    tool_mount_model="robot_side",
                    tool_mount_child_link="tool_mount",
                    tool_mount_topic_base="/tool_mount",
                    tool_rack_model="tool_rack",
                    tool_rack_child_link="rack",
                    tool_rack_topic_base="/tool_rack"
                ),
                "-name",
                "endtool",
                "-x", "0",
                "-y", "-0.164",
                "-z", "0.285",
                "-R", "0",
                "-P", "0",
                "-Y", "0",
            ],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/world/spawn/create@ros_gz_interfaces/srv/SpawnEntity',  # NOTE: endtools are known in 'tool_mount' launch and in simulation mode all tools always start on a rack so actually we can spawn all endtool entities already in launch file and in client we simply detach upon bootup (they are attached by default)
                '/world/spawn/set_pose@ros_gz_interfaces/srv/SetEntityPose',
                '/world/spawn/remove@ros_gz_interfaces/srv/DeleteEntity',
                '/model/endtool/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/mating/attach@std_msgs/msg/Empty]gz.msgs.Empty',
                '/mating/detach@std_msgs/msg/Empty]gz.msgs.Empty',
                '/mating/state@std_msgs/msg/String[gz.msgs.StringMsg'
            ],
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
            ),
            launch_arguments={
                "gz_args": IfElseSubstitution(
                    gazebo_gui,
                    if_value=[" -r -v 4 ", "/workspaces/autofactory/gazebo_sandbox/spawn.sdf"],
                    else_value=[" --headless-rendering -s -r -v 4 ", "/workspaces/autofactory/gazebo_sandbox/spawn.sdf"],
                )
            }.items(),
        ),
        # Node(
        #     executable='/workspaces/autofactory/gazebo_sandbox/gazebo_client.py',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}],
        # )
    ])