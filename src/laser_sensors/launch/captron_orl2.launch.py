# Copyright (c) 2026, Movement Controller Contributors
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
"""Launch the Captron ORL2 laser cross sensor node.

Selects between a simulated sensor (Gazebo single-ray beams bridged into ROS)
and real hardware via the ``simulated`` launch argument. The real-hardware path
is not implemented yet, so requesting it emits a log message and starts nothing.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_node(context: LaunchContext):
    """Return the actions to launch, resolved from the launch configuration."""
    simulated = LaunchConfiguration('simulated').perform(context)
    simulated_bool = simulated.lower() == 'true'

    namespace = LaunchConfiguration('namespace').perform(context)
    node_name = LaunchConfiguration('node_name').perform(context)
    detection_distance = LaunchConfiguration('detection_distance').perform(context)

    respawn = LaunchConfiguration('respawn').perform(context)
    respawn_bool = respawn.lower() == 'true'

    nodes = []
    if simulated_bool:
        LogInfo(msg="Launching Captron ORL2 node in simulated mode.").execute(context)
        # gz -> ROS bridge for the two simulated laser cross beams.
        nodes.append(
            Node(
                package='ros_gz_bridge',
                namespace=namespace,
                executable='parameter_bridge',
                name='captron_orl2_gz_bridge',
                output='screen',
                respawn=respawn_bool,
                arguments=[
                    '/simulated_laser_cross/beam_x_axis'
                    '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    '/simulated_laser_cross/beam_y_axis'
                    '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                ],
            )
        )

    nodes.append(
        Node(
            package='laser_sensors',
            namespace=namespace,
            executable='captron_orl2',
            name=node_name,
            output='screen',
            respawn=respawn_bool,
            parameters=[{
                'simulated': simulated_bool,
                "use_sim_time": True if simulated_bool else False,
                'frame_id': 'captron_orl2_intersection',
                'x_axis_beam_input': 'cross_laser_x_triggered',
                'y_axis_beam_input': 'cross_laser_y_triggered',
                "sensor_diameter": float(detection_distance),
            }],
            remappings=[
                ('simulated_laser_cross/beam_x_axis', '/simulated_laser_cross/beam_x_axis'),
                ('simulated_laser_cross/beam_y_axis', '/simulated_laser_cross/beam_y_axis'),
            ],
        )
    )

    return nodes

def generate_launch_description():
    """Declare launch arguments and wire up the node selection."""
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            'simulated',
            default_value='true',
            description='Use a simulated Gazebo sensor (true) or real hardware '
            '(false). Real hardware is not implemented yet.',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'namespace',
            default_value='laser_sensors',
            description='Namespace applied to the launched nodes.',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'node_name',
            default_value='captron_orl2',
            description='Name of the Captron ORL2 node.',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'detection_distance',
            default_value='0.04',
            description='Beam range, in metres, at or below which a beam counts '
            'as interrupted. Defaults to 0.04 m (the 40 mm window).',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'respawn',
            default_value='false',
            description='Enable respawn mode for the launched nodes.',
        )
    )

    ld.add_action(OpaqueFunction(function=_launch_node))

    return ld
