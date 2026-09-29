# launch ros_gz_bridge with bridged Gazebo services and then launch 'gazebo_client.py' node to handle mating/detaching of parts in Gazebo

from pathlib import Path
from yaml import safe_load, YAMLError

from pydantic import BaseModel, Field, ValidationError, model_validator

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import IfElseSubstitution
from launch.substitutions import (
    Command, FindExecutable, PathJoinSubstitution, IfElseSubstitution, LaunchConfiguration
)


Pose6 = tuple[float, float, float, float, float, float]


class ToolSlotDTO(BaseModel):
    """A single tool slot defined on the rack."""

    name: str = Field(description='Rack slot identifier.')
    tool_sn: str = Field(description='Serial number of the tool assigned to the slot.')
    tool_lifted_pose: Pose6 = Field(description='Pose (x, y, z, R, P, Y) of the tool lifted off the rack.')
    tool_attached_pose: Pose6 = Field(description='Pose (x, y, z, R, P, Y) where tool-mount is fully inserted into the tool.')
    tool_slide_in_pose: Pose6 = Field(description='Pose (x, y, z, R, P, Y) of from where the tool-mount is slid into the tool.')


class SimBootupDTO(BaseModel):
    """A tool spawned into simulation at bootup."""

    name: str = Field(description='Rack slot identifier.')
    tool_sn: str = Field(description='Serial number of the tool to spawn.')
    xacro_file: str = Field(description='Xacro description file for the tool.')
    executable: str = Field(description='Executable/node name that drives the tool.')


class ToolRackConfigDTO(BaseModel):
    """Tool rack configuration: slot definitions and simulation bootup tools."""

    slots: list[ToolSlotDTO] = Field(description='Tool slots defined on the rack.')
    sim_bootup: list[SimBootupDTO] = Field(description='Tools spawned into simulation at bootup.')

    @model_validator(mode='after')
    def _no_duplicate_serial_numbers(self) -> 'ToolRackConfigDTO':
        """Reject duplicate tool serial numbers within slots and within sim_bootup."""
        for section, entries in (('slots', self.slots), ('sim_bootup', self.sim_bootup)):
            seen: set[str] = set()
            duplicates = {e.tool_sn for e in entries if e.tool_sn in seen or seen.add(e.tool_sn)}
            if duplicates:
                raise ValueError(f'duplicate tool_sn in {section}: {sorted(duplicates)}')
        return self

    @model_validator(mode='after')
    def _sim_bootup_matches_slots(self) -> 'ToolRackConfigDTO':
        """Every slot must have exactly one matching sim_bootup entry (by slot name)."""
        slot_names = {slot.name for slot in self.slots}
        bootup_names = {bootup.name for bootup in self.sim_bootup}

        missing = slot_names - bootup_names
        if missing:
            raise ValueError(f'missing sim_bootup entries for slots: {sorted(missing)}')

        extra = bootup_names - slot_names
        if extra:
            raise ValueError(f'sim_bootup entries with no matching slot: {sorted(extra)}')

        return self

    def get_matched_tools(self) -> tuple[list[ToolSlotDTO], list[SimBootupDTO]]:
        """Return slots and sim_bootup entries aligned pairwise by slot name.

        Element ``i`` of both returned lists refers to the same slot ``name``.
        """
        bootup_by_name = {bootup.name: bootup for bootup in self.sim_bootup}
        matched_slots = list(self.slots)
        matched_bootups = [bootup_by_name[slot.name] for slot in matched_slots]
        return matched_slots, matched_bootups


def read_config_file(file_path: str) -> ToolRackConfigDTO:
    """Read, parse, and validate the tool-rack config YAML file."""
    if not file_path:
        raise ValueError('config file path is empty')

    path = Path(file_path)

    try:
        data = safe_load(path.read_text())
    except OSError as exc:
        raise ValueError(f'could not read config file {path}: {exc}') from exc
    except YAMLError as exc:
        raise ValueError(f'malformed YAML in {path}: {exc}') from exc

    try:
        return ToolRackConfigDTO.model_validate(data)
    except ValidationError as exc:
        raise ValueError(f'invalid config in {path}: {exc}') from exc
    

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
    config_file = LaunchConfiguration("config_file")
    world_name = LaunchConfiguration("world_name")
    world_file = LaunchConfiguration("world_file")
    gazebo_gui = LaunchConfiguration("gazebo_gui", default="true")
    simulated = LaunchConfiguration("simulated", default="true")

    declared_arguments = [
        DeclareLaunchArgument(
            'config_file',
            default_value='/workspaces/autofactory/gazebo_sandbox/config/tool_rack.yaml',
            description='Path to the tool rack config YAML file.',
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='station',
            description='Name of the Gazebo world.',
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='/workspaces/autofactory/gazebo_sandbox/worlds/station.sdf',
            description='Path to the Gazebo world SDF file.',
        ),
        DeclareLaunchArgument(
            'gazebo_gui',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to launch Gazebo with GUI or headless.',
        ),
        DeclareLaunchArgument(
            'simulated',
            default_value='true',
            choices=['true', 'false'],
            description='Simulation mode enabled or not.',
        ),
    ]

    config = read_config_file(config_file)
    actions = []

    if simulated:
        bridge_arguments = [
            f'/world/{world_name}/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            f'/world/{world_name}/create@ros_gz_interfaces/srv/SpawnEntity',
            f'/world/{world_name}/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            f'/world/{world_name}/remove@ros_gz_interfaces/srv/DeleteEntity',
        ]

        for slot_tools, sim_tool in config.get_matched_tools():
            actions.append(
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    output="screen",
                    arguments=[
                        "-string",
                        endtool_description(
                            xacro_file=sim_tool.xacro_file,
                            tool_mount_model="robot_side",
                            tool_mount_child_link="tool_mount",
                            tool_mount_topic_base=f"/tool_mount/{sim_tool.tool_sn}",
                            tool_rack_model="tool_rack",
                            tool_rack_child_link="rack",
                            tool_rack_topic_base=f"/tool_rack/{sim_tool.tool_sn}"
                        ),
                        "-name",
                        sim_tool.tool_sn,
                        "-x", str(slot_tools.tool_lifted_pose[0]),
                        "-y", str(slot_tools.tool_lifted_pose[1]),
                        "-z", str(slot_tools.tool_lifted_pose[2]),
                        "-R", str(slot_tools.tool_lifted_pose[3]),
                        "-P", str(slot_tools.tool_lifted_pose[4]),
                        "-Y", str(slot_tools.tool_lifted_pose[5]),
                    ],
                )
            )

            new_args = [
                f'/tool_mount/{sim_tool.tool_sn}/attach@std_msgs/msg/Empty]gz.msgs.Empty',
                f'/tool_mount/{sim_tool.tool_sn}/detach@std_msgs/msg/Empty]gz.msgs.Empty',
                f'/tool_mount/{sim_tool.tool_sn}/state@std_msgs/msg/String[gz.msgs.StringMsg',
                f'/tool_rack/{sim_tool.tool_sn}/attach@std_msgs/msg/Empty]gz.msgs.Empty',
                f'/tool_rack/{sim_tool.tool_sn}/detach@std_msgs/msg/Empty]gz.msgs.Empty',
                f'/tool_rack/{sim_tool.tool_sn}/state@std_msgs/msg/String[gz.msgs.StringMsg'
            ]
            bridge_arguments += new_args

        actions.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=bridge_arguments,
                output='screen'
            )
        )

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
                ),
                launch_arguments={
                    "gz_args": IfElseSubstitution(
                        gazebo_gui,
                        if_value=[" -r -v 4 ", world_file],
                        else_value=[" --headless-rendering -s -r -v 4 ", world_file],
                    )
                }.items(),
            )
        )

    for slot_tool in config.slots:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("endtools"), f"/launch/{slot_tool.launch_file}"]
                ),
                launch_arguments={
                    "simulated": simulated,
                    "tool_sn": slot_tool.tool_sn,
                }.items(),
            )
        )

    return LaunchDescription(declared_arguments + actions)