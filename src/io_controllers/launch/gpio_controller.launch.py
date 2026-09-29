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
"""Launch the device-agnostic GPIO controller lifecycle node.

Brings up ``gpio_controller``. Simulated vs. real hardware is selected here at
the launch layer via the ``simulated`` argument (repo convention); the node
code itself never branches on it.

The node is a managed lifecycle node and starts ``unconfigured``. Drive it
through its transitions with the lifecycle CLI once launched::

    ros2 launch io_controllers gpio_controller.launch.py simulated:=true
    ros2 lifecycle set /gpio_controller configure
    ros2 lifecycle set /gpio_controller activate
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Declare launch arguments and wire up the lifecycle node."""
    simulated = LaunchConfiguration('simulated')
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    node_name = LaunchConfiguration('node_name')

    declared_arguments = [
        DeclareLaunchArgument(
            'simulated',
            default_value='false',
            choices=['true', 'false'],
            description='Use the device-agnostic MockIODriver (true) instead of '
            'the real device driver (false). Also advertises set_mock_behavior.',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('io_controllers'), 'config', 'ur10.yaml']
            ),
            description='YAML mapping/config file. Defaults to the sample '
            'shipped in the io_controllers package '
            '(share/io_controllers/config/ur10.yaml). May be an absolute '
            'path or one resolved relative to share/io_controllers/config.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Standard ROS 2 sim-time flag; respected by the publish '
            'and mock-behavior timers.',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace applied to the launched node.',
        ),
        DeclareLaunchArgument(
            'node_name',
            default_value='gpio_controller',
            description='Name of the GPIO controller lifecycle node.',
        ),
    ]

    gpio_controller_node = LifecycleNode(
        package='io_controllers',
        executable='gpio_controller',
        name=node_name,
        namespace=namespace,
        output='screen',
        parameters=[{
            'config_file': config_file,
            'simulated': simulated,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        *declared_arguments,
        gpio_controller_node,
    ])
