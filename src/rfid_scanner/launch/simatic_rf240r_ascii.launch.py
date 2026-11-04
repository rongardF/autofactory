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
"""Launch the SIMATIC RF240R (RS-232 ASCII) RFID scanner lifecycle node.

The node is started *unconfigured*; an external manager drives the lifecycle
transitions. In simulation a GZ->ROS bridge for the device's Gazebo RFID
reader topics is started as well.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def _launch_nodes(context: LaunchContext) -> list:
    """Return the actions to launch, resolved from the launch configuration."""
    def arg(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    simulated = arg('simulated').lower() == 'true'
    device_name = arg('device_name')
    node_name = arg('node_name')

    actions: list[Node] = []
    actions = [
        LifecycleNode(
            package='rfid_scanner',
            executable='simatic_rf240r_ascii',
            name=node_name,
            namespace='',
            output='screen',
            parameters=[{
                'simulated': simulated,
                'device_name': device_name,
                'serial_port': arg('serial_port'),
                'baud_rate': int(arg('baud_rate')),
                'parity': arg('parity'),
                'stop_bits': int(arg('stop_bits')),
                'response_timeout_sec': float(arg('response_timeout_sec')),
                'heartbeat_period_sec': float(arg('heartbeat_period_sec')),
                'use_sim_time': simulated,
            }],
        ),
    ]

    if simulated:
        actions.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=f'{node_name}_gz_bridge',
                output='screen',
                arguments=[
                    f'/{device_name}/detected@std_msgs/msg/Bool[gz.msgs.Boolean',
                    f'/{device_name}/tag_data'
                    '@ros_gz_interfaces/msg/Dataframe[gz.msgs.Dataframe',
                ],
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    """Declare launch arguments and start the node (and bridge in simulation)."""
    arguments = [
        ('simulated', 'true',
         'Use the Gazebo-simulated reader (true) or real hardware (false).'),
        ('device_name', 'simatic_rf240r',
         'Device name; namespaces the Gazebo topics/service. Must equal the xacro '
         'device_name.'),
        ('node_name', 'simatic_rf240r_ascii', 'Name of the lifecycle node.'),
        ('serial_port', '/dev/ttyUSB0', 'Serial device of the reader (real hardware).'),
        ('baud_rate', '19200', 'Serial baud rate: 19200, 57600 or 115200.'),
        ('parity', 'odd', 'Serial parity: odd, even or none.'),
        ('stop_bits', '1', 'Serial stop bits: 1 or 2.'),
        ('response_timeout_sec', '1.0',
         'Maximum wait for a command acknowledgement, in seconds.'),
        ('heartbeat_period_sec', '1.0',
         'SLG-STATUS heartbeat period, in seconds (real hardware).'),
    ]
    ld = LaunchDescription()
    for name, default, description in arguments:
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=description))
    ld.add_action(OpaqueFunction(function=_launch_nodes))
    return ld
