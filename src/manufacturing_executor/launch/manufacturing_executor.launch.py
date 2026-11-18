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
"""Launch the ``manufacturing_executor`` lifecycle node and the localizer it drives.

The node is started *unconfigured*; an external lifecycle manager drives the
transitions. The node configures and activates the included localizer itself.
Optimizers, repositories, product carrier, production manager, station workplace,
camera, endtools and tools manager are not launched here.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare

# (dependency key, launch argument, default, description)
_DEPENDENCIES = [
    ('movement_controller', 'movement_controller_node_name', 'movement_controller',
     'Movement controller node name.'),
    ('station_workplace', 'station_workplace_node_name', 'workplace_rfid_scanner',
     'Station workplace node name.'),
    ('camera', 'camera_node_name', 'cameras/basler_camera',
     'Camera node (with its namespace) serving the get_image action.'),
    ('localizer', 'localizer_node_name', 'mock_localizer', 'Localizer node name.'),
]


def _launch_node(context: LaunchContext) -> list:
    """Return the lifecycle node, resolved from the launch configuration."""
    def arg(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    # Only the dependencies that are launch arguments are overridden; the rest keep the node defaults.
    overrides = {key: arg(argument) for key, argument, _, _ in _DEPENDENCIES}
    parameters: dict = {
        'use_sim_time': arg('use_sim_time').lower() == 'true',
        'camera_tcp': arg('camera_tcp'),
        'dependency_nodes.names': list(overrides),
        'dependency_nodes.values': list(overrides.values()),
    }
    return [
        LifecycleNode(
            package='manufacturing_executor',
            executable='manufacturing_executor',
            name=arg('node_name'),
            namespace='',
            output='screen',
            parameters=[parameters],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Declare the launch arguments, include the localizer and start the node."""
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'node_name', default_value='manufacturing_executor', description='Name of the node.'))
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use the simulation clock.'))
    ld.add_action(DeclareLaunchArgument(
        'camera_tcp', default_value='camera_tcp_calibrated',
        description='TF frame used as tool_frame for the image capture moves.'))
    for _, argument, default, description in _DEPENDENCIES:
        ld.add_action(DeclareLaunchArgument(argument, default_value=default, description=description))
    # Scoped so the arguments of the localizer launch do not collide with ours.
    ld.add_action(GroupAction(
        [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare('localization'), '/launch/mock_localizer.launch.py']),
            launch_arguments={
                'node_name': LaunchConfiguration('localizer_node_name'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'station_workplace_node_name': LaunchConfiguration('station_workplace_node_name'),
                'camera_node_name': LaunchConfiguration('camera_node_name'),
                'movement_controller_node_name': LaunchConfiguration('movement_controller_node_name'),
            }.items(),
        )],
        scoped=True,
    ))
    ld.add_action(OpaqueFunction(function=_launch_node))
    return ld
