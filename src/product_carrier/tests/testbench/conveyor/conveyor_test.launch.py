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

"""Sandbox launch file for testing the ``product_carrier`` conveyor node.

Starts Gazebo with ``conveyor_world.sdf``, spawns the conveyor (fixed to the
world), bridges ``/clock`` and includes ``product_carrier/conveyor.launch.py``
(conveyor node + its Gazebo bridge). Once the conveyor exists, a test pallet is
spawned at the carrier pose; its DetachableJoint welds it to the ``carrier``
link on spawn, so it rides along the passive carrier joint.

The conveyor node starts Unconfigured; drive it from
``src/product_carrier/tests/testbench/conveyor_testbench.ipynb``.

Usage::

    ros2 launch sandbox_gazebo/conveyor/conveyor_test.launch.py
    ros2 launch sandbox_gazebo/conveyor/conveyor_test.launch.py gui:=false

To bring the pallet into the workplace, drive ``base_carrier_joint`` to
``-0.6`` m (its limit, where the pallet covers the presence sensor) with the
Gazebo GUI "Joint Position Controller" plugin (select the ``conveyor`` model),
or from a terminal::

    gz topic -t /model/conveyor/joint/base_carrier_joint/0/cmd_pos \
        -m gz.msgs.Double -p 'data: -0.6'

``0.0`` moves it back out.
"""

from pathlib import Path

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

HERE = Path(__file__).resolve().parent
# hardcoded because tests will anyway not be copied to share
MODEL_PATH = "/workspaces/autofactory/src/product_carrier/model"
WORLD_NAME = 'conveyor_test'
# World pose of the 'carrier' link at joint position 0 (conveyor_origin at the
# world origin). The DetachableJoint keeps the pose the pallet has at attach time.
CARRIER_POSE = ('1.051', '0.3', '0.84')


def generate_launch_description() -> LaunchDescription:
    gui = LaunchConfiguration('gui')

    declared_arguments = [
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false'],
                              description='Start the Gazebo GUI (false = server only).'),
    ]

    # Lets Gazebo resolve model://pallet_model
    resource_path = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', MODEL_PATH)

    # No tf_prefix: the pallet DetachableJoint targets the unprefixed conveyor::carrier link.
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', str(HERE / 'conveyor_test.urdf.xacro'),
            ' simulated:=true',
        ]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': IfElseSubstitution(
                gui,
                if_value=f'-r -v 2 {HERE / "conveyor_world.sdf"}',
                else_value=f'-r -s -v 2 {HERE / "conveyor_world.sdf"}',
            ),
            'on_exit_shutdown': 'true',
        }.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn_conveyor = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-world', WORLD_NAME, '-name', 'conveyor', '-topic', 'robot_description'],
    )

    spawn_pallet = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', WORLD_NAME, '-name', 'pallet',
            '-file', f"{MODEL_PATH}/pallet/model.sdf",
            '-x', CARRIER_POSE[0], '-y', CARRIER_POSE[1], '-z', CARRIER_POSE[2],
        ],
    )

    conveyor_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('product_carrier'), 'launch', 'conveyor.launch.py'])
        ),
        launch_arguments={
            'simulated': 'true',
            'node_name': 'conveyor',
        }.items(),
    )

    return LaunchDescription(
        declared_arguments
        + [
            resource_path,
            gazebo,
            clock_bridge,
            robot_state_publisher,
            spawn_conveyor,
            # The DetachableJoint needs the conveyor model to exist when the pallet is created
            RegisterEventHandler(OnProcessExit(target_action=spawn_conveyor, on_exit=[spawn_pallet])),
            conveyor_node,
        ]
    )
