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
"""Launch the ``mock_localizer`` lifecycle node.

The node is started *unconfigured*; an external lifecycle manager (or
``ros2 lifecycle set``) drives the transitions. Every node parameter is exposed as a
launch argument of the same name; ``initial_localization_pose`` is given as six
comma-separated numbers.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

# (name, default, description, type)
_PARAMETERS = [
    ('station_workplace_node_name', 'station_workplace',
     'Node publishing the workplace state.', str),
    ('movement_controller_node_name', 'movement_controller',
     'Node serving the execute_trajectory action.', str),
    ('camera_node_name', 'cameras/basler_camera',
     'Camera node (with its namespace) serving the get_image action.', str),
    ('camera_frame_id', 'camera_tcp_calibrated', 'Camera TCP frame.', str),
    ('base_frame_id', 'station', 'Frame of the initial localization pose.', str),
    ('carrier_frame_id', 'carrier', 'Parent frame of product_origin.', str),
    ('initial_localization_pose', '1.0,0.0,1.46,0.0,1.5708,0.0',
     'Initial capture pose x,y,z,roll,pitch,yaw (m, rad) in base_frame_id.',
     lambda value: [float(item) for item in value.split(',')]),
    ('image_capture_settling_s', '0.5', 'Wait before capturing an image [s].', float),
    ('image_capture_timeout_s', '5.0', 'Time allowed for the camera to return an image [s].',
     float),
]


def _launch_node(context: LaunchContext) -> list:
    """Return the lifecycle node, resolved from the launch configuration."""
    def arg(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    parameters = {name: cast(arg(name)) for name, _, _, cast in _PARAMETERS}
    parameters['calibration_poses_file'] = arg('calibration_poses_file')
    parameters['use_sim_time'] = arg('use_sim_time').lower() == 'true'
    return [
        LifecycleNode(
            package='localization',
            executable='mock_localizer',
            name=arg('node_name'),
            namespace='',
            output='screen',
            parameters=[parameters],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Declare the launch arguments and start the node."""
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'node_name', default_value='mock_localizer', description='Name of the node.'))
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use the simulation clock.'))
    ld.add_action(DeclareLaunchArgument(
        'calibration_poses_file',
        default_value=os.path.join(
            get_package_share_directory('localization'), 'config', 'localization_poses.yaml'),
        description='Absolute path of the YAML file with the localization image capture poses.'))
    for name, default, description, _ in _PARAMETERS:
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=description))
    ld.add_action(OpaqueFunction(function=_launch_node))
    return ld
