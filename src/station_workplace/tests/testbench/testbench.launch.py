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
"""Testbench for the ``workplace_rfid_scanner`` node, fully simulated / local.

Brings up, in one launch:

- Gazebo (``testbench_world.sdf``, world ``default``) with the scene of
  ``test.urdf.xacro`` spawned as model ``station`` (conveyor + RFID scanner),
  plus the ``/clock`` bridge and ``robot_state_publisher``;
- ``move_group`` (planning scene services for the workplace node) and RViz;
- ``conveyor`` (``product_carrier``, simulated), ``simatic_rf240r_ascii``
  (``rfid_scanner``, simulated), ``production_manager`` (local assembly file),
  ``object_model_storage`` (``repositories``, local storage);
- ``workplace_rfid_scanner`` (``station_workplace``, simulated).

All lifecycle nodes start Unconfigured; drive them (and the scenario) from
``workplace_testbench.ipynb``. The local-mode files must be in
``download_path`` beforehand; ``setup.sh`` copies them and runs this launch::

    ./setup.sh
    # or, with the assets already in place:
    ros2 launch src/station_workplace/tests/testbench/testbench.launch.py

The conveyor carrier is moved with the Gazebo GUI "Joint Position Controller"
plugin (model ``station``, joint ``base_carrier_joint``; ``-0.6`` = in place,
``0.0`` = out), or from a terminal::

    gz topic -t /model/station/joint/base_carrier_joint/0/cmd_pos \\
        -m gz.msgs.Double -p 'data: -0.6'
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

HERE = Path(__file__).resolve().parent
WORLD_FILE = HERE / 'testbench_world.sdf'
# Must match the world name in WORLD_FILE and the workplace simulation.world_name.
WORLD_NAME = 'default'
# Must match the workplace simulation.station_model_name.
STATION_MODEL_NAME = 'station'
# Must match the xacro device_name of the scanner in test.urdf.xacro.
RFID_DEVICE_NAME = 'simatic_rf240r'


def _include(package: str, launch_file: str, arguments: dict) -> GroupAction:
    """Include a package launch file, scoped so its declared arguments do not leak."""
    return GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])
                ),
                launch_arguments=arguments.items(),
            )
        ],
        scoped=True,
    )


def generate_launch_description() -> LaunchDescription:
    """Declare the testbench arguments and start every node."""
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    station_id = LaunchConfiguration('station_id')
    download_path = LaunchConfiguration('download_path')

    declared_arguments = [
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false'],
                              description='Start the Gazebo GUI (false = server only).'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false'],
                              description='Start RViz.'),
        DeclareLaunchArgument('station_id', default_value='c330289c-0db3-4b5b-9fe0-3d9b63bedb72',
                              description='Station UUID4 (production_manager, object_model_storage).'),
        DeclareLaunchArgument('download_path', default_value='/tmp/station',
                              description='Folder holding local_storage.json, the model entries '
                              'and mock_assembly_file.json.'),
    ]

    # Gazebo resolves the spawned models' model://<model_id>/... URIs from download_path.
    resource_path = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', download_path)

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', str(HERE / 'test.urdf.xacro'),
                 ' simulated:=true']),
        value_type=str,
    )
    robot_description_semantic = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', str(HERE / 'test.srdf.xacro')]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': IfElseSubstitution(
                gui,
                if_value=f'-r -v 2 {WORLD_FILE}',
                else_value=f'-r -s -v 2 {WORLD_FILE}',
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

    spawn_station = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-world', WORLD_NAME, '-name', STATION_MODEL_NAME,
                   '-topic', 'robot_description'],
    )

    # No arm and no controllers: move_group only serves the planning scene
    # (/get_planning_scene, /apply_planning_scene) and publishes it for RViz.
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'robot_description_semantic': robot_description_semantic,
            'use_sim_time': True,
            'publish_robot_description': True,
            'publish_robot_description_semantic': True,
            'planning_pipelines': ['ompl'],
            'default_planning_pipeline': 'ompl',
            'ompl': {'planning_plugins': ['ompl_interface/OMPLPlanner']},
            'allow_trajectory_execution': False,
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', str(HERE / 'testbench.rviz')],
        parameters=[{
            'robot_description': robot_description,
            'robot_description_semantic': robot_description_semantic,
            'use_sim_time': True,
        }],
        condition=IfCondition(rviz),
    )

    conveyor = _include('product_carrier', 'conveyor.launch.py', {
        'simulated': 'true',
        'node_name': 'conveyor',
    })

    rfid_scanner = _include('rfid_scanner', 'simatic_rf240r_ascii.launch.py', {
        'simulated': 'true',
        'device_name': RFID_DEVICE_NAME,
        'node_name': 'simatic_rf240r_ascii',
    })

    production_manager = _include('production_manager', 'production_manager_launch.py', {
        'local_manager': 'true',
        'assembly_file_path': PathJoinSubstitution([download_path, 'mock_assembly_file.json']),
        'station_id': station_id,
        'use_sim_time': 'true',
    })

    # repositories ships no launch file for this node
    object_model_storage = LifecycleNode(
        package='repositories',
        executable='object_model_storage',
        name='object_model_storage',
        namespace='',
        output='screen',
        parameters=[{
            'local_storage': True,
            'download_path': download_path,
            'station_id': station_id,
            'use_sim_time': True,
        }],
    )

    workplace = _include('station_workplace', 'workplace_rfid_scanner.launch.py', {
        'node_name': 'workplace_rfid_scanner',
        'use_sim_time': 'true',
        'simulated': 'true',
        'product_carrier_node_name': 'conveyor',
        'production_manager_node_name': 'production_manager',
        'object_model_repository_node_name': 'object_model_storage',
        'rfid_scanner_node_name': 'simatic_rf240r_ascii',
        'collision_allowed_link': 'conveyor_origin',
        'simulation.world_name': WORLD_NAME,
        'simulation.station_model_name': STATION_MODEL_NAME,
        'simulation.station_link': 'carrier',
    })

    return LaunchDescription(
        declared_arguments
        + [
            resource_path,
            gazebo,
            clock_bridge,
            robot_state_publisher,
            spawn_station,
            move_group,
            rviz_node,
            conveyor,
            rfid_scanner,
            production_manager,
            object_model_storage,
            workplace,
        ]
    )
