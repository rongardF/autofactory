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

"""Launch the dispensing_pattern_executor lifecycle node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Build the launch description for the dispensing pattern executor."""
    simulated = LaunchConfiguration('simulated')
    node_name = LaunchConfiguration('node_name')
    tool_sn = LaunchConfiguration('tool_sn')
    endtool_node_name = LaunchConfiguration('endtool_node_name')
    movement_controller_node_name = LaunchConfiguration('movement_controller_node_name')
    config_file = LaunchConfiguration('config_file')

    declared_arguments = [
        DeclareLaunchArgument(
            'simulated',
            default_value='true',
            choices=['true', 'false'],
            description='Whether the executor runs in simulation mode.',
        ),
        DeclareLaunchArgument(
            'node_name',
            default_value='dispensing_pattern_executor',
            description='Name of the dispensing pattern executor lifecycle node.',
        ),
        DeclareLaunchArgument(
            'tool_sn',
            description='End-tool serial number; movement frame is '
            '{tool_sn}_calibrated_tcp.',
        ),
        DeclareLaunchArgument(
            'endtool_node_name',
            description='Node name of the dispenser whose lifecycle is driven.',
        ),
        DeclareLaunchArgument(
            'movement_controller_node_name',
            default_value='/movement_controller/execute_trajectory',
            description='Movement controller node name.',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('pattern_executors'),
                'config',
                'dispensing_pattern_executor.yaml',
            ]),
            description='Path to the executor parameter YAML file.',
        ),
    ]

    executor_node = LifecycleNode(
        package='pattern_executors',
        executable='dispensing_pattern_executor',
        name=node_name,
        namespace='',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': simulated,
                'simulated': simulated,
                'tool_sn': tool_sn,
                'endtool_node_name': endtool_node_name,
                'movement_controller_node_name': movement_controller_node_name,
            },
        ],
    )

    return LaunchDescription(declared_arguments + [executor_node])
