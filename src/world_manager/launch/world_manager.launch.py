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
"""Starts the MoveIt manager always and the Gazebo manager only when ``simulated:=true``."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    simulated = LaunchConfiguration('simulated')

    gazebo_manager = LifecycleNode(
        package='world_manager',
        executable='gazebo_world_manager',
        name='gazebo_world_manager',
        namespace='',
        output='screen',
        condition=IfCondition(simulated),
        parameters=[
            LaunchConfiguration('config_file'),
            {'world_name': LaunchConfiguration('world_name'), 'use_sim_time': simulated},
        ],
    )
    moveit_manager = LifecycleNode(
        package='world_manager',
        executable='moveit2_world_manager',
        name='moveit2_world_manager',
        namespace='',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': simulated},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('simulated', default_value='true', choices=['true', 'false'],
                              description='Run the Gazebo world manager.'),
        DeclareLaunchArgument('world_name', default_value='world', description='Name of the Gazebo world.'),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([FindPackageShare('world_manager'), 'config', 'world_manager.yaml']),
            description='Parameter file of the world manager nodes.'),
        gazebo_manager,
        moveit_manager,
    ])
