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
"""Launch the ``workplace_barcode_scanner`` lifecycle node.

The node is started *unconfigured*; an external lifecycle manager drives the
transitions. Every node parameter is exposed as a launch argument of the same
name (e.g. ``label_patterns.serial_number:='SN:(\\w+)'``). The
``label_patterns.*`` arguments must be set, otherwise configure fails.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

_FIELDS = ('product_number', 'product_revision', 'serial_number')

# (name, default, description, type)
_PARAMETERS = [
    ('product_carrier_node_name', 'conveyor', 'Product carrier node name.', str),
    ('production_manager_node_name', 'production_manager',
     'Production manager node name.', str),
    ('object_model_repository_node_name', 'object_model_storage',
     'Object model storage node name.', str),
    ('collision_allowed_link', 'conveyor_origin',
     'Robot link the carrier model and the safety object may collide with.', str),
    ('safety_object.x', '0.25', 'Safety box size along X [m].', float),
    ('safety_object.y', '0.4', 'Safety box size along Y [m].', float),
    ('safety_object.z', '0.3', 'Safety box size along Z [m].', float),
    ('service_call_timeout_sec', '5.0', 'Timeout for service availability and response [s].',
     float),
    ('barcode_scanner_node_name', 'barcode_scanner', 'Barcode scanner node name.', str),
]
for _field in _FIELDS:
    _PARAMETERS.append((f'label_patterns.{_field}', '',
                        f'Regex with exactly one capture group extracting {_field} '
                        'from the label (required).', str))


def _launch_node(context: LaunchContext) -> list:
    """Return the lifecycle node, resolved from the launch configuration."""
    def arg(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    parameters = {name: cast(arg(name)) for name, _, _, cast in _PARAMETERS}
    parameters['use_sim_time'] = arg('use_sim_time').lower() == 'true'
    return [
        LifecycleNode(
            package='station_workplace',
            executable='workplace_barcode_scanner',
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
        'node_name', default_value='workplace_barcode_scanner',
        description='Name of the lifecycle node.'))
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use the simulation clock.'))
    for name, default, description, _ in _PARAMETERS:
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=description))
    ld.add_action(OpaqueFunction(function=_launch_node))
    return ld
