# Copyright (c) 2026, Autofactory
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""Launch the tool rack, tool mount and tools manager together with the endtools of the rack.

The endtools and their pattern executors are composed statically below. The
tools manager only configures the endtool nodes started here; it never starts
or stops their processes.
"""

from typing import NamedTuple

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from tools_manager.model.tools_manager_config import ToolsManagerConfigDTO

# Internal knowledge: the optimizer shared by all dispensing pattern executors started here.
_DISPENSING_OPTIMIZER_NODE_NAME = 'dispensing_optimizer'


class _Endtool(NamedTuple):
    """An endtool instance together with the pattern executor that drives it."""

    tool_sn: str
    node_name: str
    pattern_executor_argument: str
    pattern_executor_default: str


# Every physical rack slot with the endtool that lives in it; ``None`` marks a slot
# without an assigned tool, which gets neither an endtool nor an executor.
_RACK_SLOTS: tuple[tuple[int, _Endtool | None], ...] = (
    (
        1,
        _Endtool(
            tool_sn='abc123',
            node_name='dispenser_abc123',
            pattern_executor_argument='pattern_executor_abc123',
            pattern_executor_default='dispensing_executor_abc123',
        ),
    ),
    (
        2,
        _Endtool(
            tool_sn='def123',
            node_name='dispenser_def123',
            pattern_executor_argument='pattern_executor_def123',
            pattern_executor_default='dispensing_executor_def123',
        ),
    ),
)


def _tools_parameters() -> dict[str, list]:
    """Build the expected tool assignment shared by the tool rack and the tools manager."""
    index = [slot_index for slot_index, _ in _RACK_SLOTS]
    tool_sn = [endtool.tool_sn if endtool else '' for _, endtool in _RACK_SLOTS]
    node_name = [endtool.node_name if endtool else '' for _, endtool in _RACK_SLOTS]

    # fail while generating the launch description, not at node configuration
    ToolsManagerConfigDTO.from_parameters(index=index, tool_sn=tool_sn, node_name=node_name)

    return {
        'tools.index': index,
        'tools.tool_sn': tool_sn,
        'tools.node_name': node_name,
    }


def _endtool_actions(endtool: _Endtool):
    """Compose an endtool launch with its pattern executor launch."""
    simulated = LaunchConfiguration('simulated')
    pattern_executor = LaunchConfiguration(endtool.pattern_executor_argument)

    endtool_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('endtools'),
                'launch',
                'volumetric_dispenser_launch.py',
            ])
        ),
        launch_arguments={
            'simulated': simulated,
            'use_sim_time': simulated,
            'world_name': LaunchConfiguration('world_name'),
            'tool_sn': endtool.tool_sn,
            'node_name': endtool.node_name,
            'pattern_executor': pattern_executor,
            'station_workplace_node_name': LaunchConfiguration('station_workplace_node_name'),
        }.items(),
    )
    executor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('pattern_executors'),
                'launch',
                'dispensing_pattern_executor.launch.py',
            ])
        ),
        launch_arguments={
            'simulated': simulated,
            'node_name': pattern_executor,
            'tool_sn': endtool.tool_sn,
            'endtool_node_name': endtool.node_name,
            'execution_optimizer': _DISPENSING_OPTIMIZER_NODE_NAME,
            'movement_controller_node_name': [
                '/',
                LaunchConfiguration('movement_controller_node_name'),
            ],
        }.items(),
    )
    # scope each include so the arguments of one launch do not leak into the other
    return [
        GroupAction([endtool_launch], scoped=True),
        GroupAction([executor_launch], scoped=True),
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
            'station_workplace_node_name',
            default_value='workplace_rfid_scanner',
            description=(
                'Simulation only: node name of the station workplace whose spawned product '
                'the endtools weld their simulated models to.'
            ),
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='world',
            description='Name of the Gazebo world.',
        ),
        DeclareLaunchArgument(
            'start_world_manager',
            default_value='false',
            choices=['true', 'false'],
            description='Start the shared world manager for this launch.',
        ),
        DeclareLaunchArgument(
            'tools_manager_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare("tools_manager"),
                "config",
                "tools_manager_config.yaml",
            ]),
            description='ROS parameter file with the initial simulated tool inventory.',
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
    # one pattern executor node name per endtool instance
    declared_arguments += [
        DeclareLaunchArgument(
            endtool.pattern_executor_argument,
            default_value=endtool.pattern_executor_default,
            description=(
                f"Node name of the pattern executor of endtool '{endtool.node_name}'; "
                'it names the executor node and is given to the endtool as its pattern_executor.'
            ),
        )
        for _, endtool in _RACK_SLOTS
        if endtool is not None
    ]

    simulated = LaunchConfiguration("simulated")
    tools_parameters = _tools_parameters()

    actions = []

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('world_manager'),
                    'launch',
                    'world_manager.launch.py',
                ])
            ),
            launch_arguments={
                'simulated': simulated,
                'world_name': LaunchConfiguration('world_name'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('start_world_manager')),
        )
    )

    # Bridge the RFID scanner tag data of every rack slot (assigned or not) and of the
    # tool mount into ROS when simulating.
    bridge_arguments = [
        f'/slot{slot_index}_rfid_scanner/tag_data@ros_gz_interfaces/msg/'
        'Dataframe[gz.msgs.Dataframe'
        for slot_index, _ in _RACK_SLOTS
    ]
    bridge_arguments.append(
        'tool_mount_rfid_scanner/tag_data@ros_gz_interfaces/msg/'
        'Dataframe[gz.msgs.Dataframe'
    )
    actions.append(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_ros_bridge_tools_manager',
            arguments=bridge_arguments,
            output='screen',
            condition=IfCondition(simulated),
        )
    )

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
                    "parent_frame_id": LaunchConfiguration(
                        "tool_mount_parent_frame_id"
                    ),
                    "mounted_publish_rate": LaunchConfiguration(
                        "tool_mounted_update_rate"
                    ),
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
                    "parent_frame_id": LaunchConfiguration(
                        "tool_rack_parent_frame_id"
                    ),
                    "slots_update_rate": LaunchConfiguration(
                        "tool_rack_slots_update_rate"
                    ),
                    **tools_parameters,
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
                LaunchConfiguration("tools_manager_config_file"),
                {
                    "simulated": simulated,
                    "use_sim_time": simulated,
                    "world_name": LaunchConfiguration("world_name"),
                    "station_model_name": LaunchConfiguration(
                        "station_model_name"
                    ),
                    "tool_mount_node_name": "/tool_mount",
                    "tool_rack_node_name": "/tool_rack",
                    "movement_controller_node_name": LaunchConfiguration(
                        "movement_controller_node_name"
                    ),
                    **tools_parameters,
                },
            ],
        )
    )

    optimizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('execution_optimizer'),
                'launch',
                'dispensing_optimizer.launch.py',
            ])
        ),
        launch_arguments={
            'simulated': simulated,
            'node_name': _DISPENSING_OPTIMIZER_NODE_NAME,
        }.items(),
    )
    actions.append(GroupAction([optimizer_launch], scoped=True))

    for _, endtool in _RACK_SLOTS:
        if endtool is not None:
            actions += _endtool_actions(endtool)

    return LaunchDescription(declared_arguments + actions)
