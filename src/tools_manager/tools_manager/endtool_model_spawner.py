# Copyright (c) 2026, Tools Manager Contributors
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
"""Endtool model spawner — add a single tool model to MoveIt 2 (and Gazebo).

The spawner is a run-once utility node. It spawns exactly one tool — identified
by ``model_id`` (its serial number) with assets under ``model_path`` — into the
MoveIt 2 planning scene, additionally spawning it into the Gazebo world when
simulation is enabled. One spawner instance is launched per tool, so several may
run concurrently; each therefore takes a unique node name to avoid clashes. The
tool is spawned onto its rack slot regardless of the real hardware situation; the
``tool_rack`` and ``tool_mount`` nodes reconcile the real mounted tool once activated.
"""

from __future__ import annotations

from pathlib import Path
from threading import Thread
from uuid import uuid4

from rclpy import init, shutdown
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from tools_manager.model.rack_config import RackConfigDTO
from tools_manager.services.gazebo_world_manager import GazeboWorldManager
from tools_manager.services.moveit2_world_manager import Moveit2WorldManager
from tools_manager.utils.config_reader import read_tool_rack_config_file


class EndtoolModelSpawner(Node):
    """A ROS 2 node that spawns a single endtool in Gazebo and MoveIt 2.

    The node declares its parameters on construction; :meth:`spawn_model` does
    the actual work and is expected to be driven once by :func:`main` while a
    background executor spins the node, after which the process exits.
    """

    #: XACRO description location relative to a tool's ``model_path`` folder.
    XACRO_RELATIVE_PATH = Path('model.sdf.xacro')

    #: Collision mesh location relative to a tool's ``model_path`` folder.
    COLLISION_MESH_RELATIVE_PATH = Path('meshes') / 'collision.stl'

    def __init__(self, node_name: str = 'endtool_model_spawner') -> None:
        """Initialize the EndtoolModelSpawner node and declare its parameters.

        A short unique suffix is appended to ``node_name`` so multiple spawner
        instances (one per tool) can run concurrently without name clashes.

        Args:
            node_name (str): The base name of the node.
        """
        super().__init__(f'{node_name}_{uuid4().hex[:8]}')

        self._gazebo_service: GazeboWorldManager | None = None
        self._planner_service: Moveit2WorldManager | None = None

        self.declare_parameter(
            'simulated',
            True,
            ParameterDescriptor(
                description='Whether the node is running in simulation mode '
                '(spawns the tool into Gazebo in addition to the planning scene)',
            ),
        )
        self.declare_parameter(
            'config_file',
            'tool_rack_config.yaml',
            ParameterDescriptor(description='Path to the tool rack configuration file'),
        )
        self.declare_parameter(
            'model_id',
            '',
            ParameterDescriptor(
                description='Unique model ID for the tool to spawn; equal to the '
                'tool serial number (tool_sn)',
            ),
        )
        self.declare_parameter(
            'model_path',
            '',
            ParameterDescriptor(
                description="Directory holding the tool's model assets; its XACRO "
                'description and meshes/collision.stl are resolved relative to this path',
            ),
        )
        self.declare_parameter(
            'world_name',
            'world',
            ParameterDescriptor(description='Name of the Gazebo world to spawn into'),
        )
        self.declare_parameter(
            'station_model_name',
            'station',
            ParameterDescriptor(description='Name of the station model in the Gazebo world'),
        )
        self.declare_parameter(
            'tool_mount_link',
            'tool_mount_tcp',
            ParameterDescriptor(description='Link a tool is welded/attached to when mounted'),
        )

    def spawn_model(self) -> bool:
        """Spawn the configured tool into the planning scene (and Gazebo).

        Reads the rack configuration, constructs the world managers, and spawns
        the ``model_id`` tool onto its rack slot. Spawning into the MoveIt 2
        planning scene always happens; spawning into Gazebo only happens when the
        ``simulated`` parameter is set.

        Returns:
            bool: ``True`` if the tool was spawned successfully, ``False`` if the
            spawn failed or the configuration could not be read.
        """
        try:
            simulated = self.get_parameter('simulated').get_parameter_value().bool_value
            config_file = self.get_parameter('config_file').get_parameter_value().string_value
            model_id = self.get_parameter('model_id').get_parameter_value().string_value
            model_path = self.get_parameter('model_path').get_parameter_value().string_value
            world_name = self.get_parameter('world_name').get_parameter_value().string_value
            station_model_name = (
                self.get_parameter('station_model_name').get_parameter_value().string_value
            )
            tool_mount_link = (
                self.get_parameter('tool_mount_link').get_parameter_value().string_value
            )

            if not model_id:
                self.get_logger().error("Parameter 'model_id' is empty")
                return False

            rack_config: RackConfigDTO = read_tool_rack_config_file(config_file)
        except Exception as exc:
            self.get_logger().error(f'Failed to read tool rack configuration: {exc}')
            return False

        self._planner_service = Moveit2WorldManager(
            node=self,
            rack_config=rack_config,
            tool_mount_link=tool_mount_link,
        )
        if simulated:
            self._gazebo_service = GazeboWorldManager(
                node=self,
                world_name=world_name,
                station_model_name=station_model_name,
                rack_config=rack_config,
                tool_mount_link=tool_mount_link,
            )

        base_path = Path(model_path)
        xacro_path = base_path / self.XACRO_RELATIVE_PATH
        mesh_path = base_path / self.COLLISION_MESH_RELATIVE_PATH

        if not self._planner_service.spawn_model(model_id, str(mesh_path)):
            self.get_logger().error(
                f'Failed to spawn model {model_id} into the MoveIt 2 planning scene'
            )
            return False

        if simulated and self._gazebo_service is not None:
            if not self._gazebo_service.spawn_model(model_id, str(xacro_path)):
                self.get_logger().error(
                    f'Failed to spawn model {model_id} into the Gazebo world'
                )
                return False

        return True


def main(args=None) -> None:
    """Entry point for the ``endtool_model_spawner`` executable.

    Initialises rclpy, creates an :class:`EndtoolModelSpawner`, spins it on a
    background :class:`~rclpy.executors.MultiThreadedExecutor` so the world
    managers' asynchronous service calls can be served, runs the one-shot spawn
    sequence, then tears everything down and exits.

    Args:
        args: Optional command-line arguments forwarded to :func:`rclpy.init`.
    """
    init(args=args)
    node = EndtoolModelSpawner()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        if node.spawn_model():
            node.get_logger().info('Endtool model spawned successfully')
        else:
            node.get_logger().error('Endtool model spawning failed')
    finally:
        executor.shutdown()
        node.destroy_node()
        shutdown()


if __name__ == '__main__':
    main()
