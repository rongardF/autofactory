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

"""Launch the dispensing_optimizer lifecycle node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Build the launch description for the dispensing optimizer."""
    simulated = LaunchConfiguration('simulated')
    node_name = LaunchConfiguration('node_name')
    config_file = LaunchConfiguration('config_file')
    camera_node_name = LaunchConfiguration('camera_node_name')
    camera_info_timeout_s = LaunchConfiguration('camera_info_timeout_s')

    declared_arguments = [
        DeclareLaunchArgument(
            'simulated',
            default_value='true',
            choices=['true', 'false'],
            description='Whether the node runs against the simulation (use_sim_time).',
        ),
        DeclareLaunchArgument(
            'node_name',
            default_value='dispensing_optimizer',
            description='Name of the dispensing optimizer lifecycle node.',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('execution_optimizer'),
                'config',
                'dispensing_optimizer.yaml',
            ]),
            description='Path to the optimizer parameter YAML file.',
        ),
        DeclareLaunchArgument(
            'camera_node_name',
            default_value='cameras/basler_camera',
            description='Camera node (with its namespace) serving the get_image action.',
        ),
        DeclareLaunchArgument(
            'camera_info_timeout_s',
            default_value='10.0',
            description='Timeout in seconds for fetching the camera intrinsics on activation.',
        ),
    ]

    optimizer_node = LifecycleNode(
        package='execution_optimizer',
        executable='dispensing_optimizer',
        name=node_name,
        namespace='',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': ParameterValue(simulated, value_type=bool),
                'camera_node_name': camera_node_name,
                'camera_info_timeout_s': ParameterValue(camera_info_timeout_s, value_type=float),
            },
        ],
    )

    return LaunchDescription(declared_arguments + [optimizer_node])
