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
"""Testbench for the ``mock_localizer`` node.

Brings up, in one launch:

- the whole simulated station (``station.launch.py``): Gazebo with the
  ``mock_product`` world, ``move_group``, the ``movement_controller`` and the
  simulated ``basler_camera``;
- the ``mock_localizer`` lifecycle node (started Unconfigured).

Only the ``station_workplace`` interface is not launched; the notebook
``testbench.ipynb`` mocks it. All lifecycle nodes start Unconfigured; drive them from
the notebook. Normally started with ``setup.sh``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


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
    """Declare the testbench arguments and start the station and the localizer."""
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    declared_arguments = [
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false'],
                              description='Start the Gazebo GUI (false = server only).'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false'],
                              description='Start RViz.'),
    ]

    station = _include('station', 'station.launch.py', {
        'gazebo_gui': gui,
        'rviz': rviz,
        'world_file': PathJoinSubstitution(
            [FindPackageShare('station'), 'config', 'world', 'mock_product.world']),
    })
    localizer = _include('localization', 'mock_localizer.launch.py', {
        'use_sim_time': 'true',
    })

    return LaunchDescription([*declared_arguments, station, localizer])
