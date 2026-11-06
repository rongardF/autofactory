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
"""Gazebo world manager spawning and removing models over ``ros_gz_bridge``.

The blocking ``/world/<world>/create/blocking`` and ``/remove/blocking`` services
are used: they report the actual outcome, whereas the non-blocking variants
report success as soon as the request is queued.
"""

import threading

from geometry_msgs.msg import Pose
from rclpy.callback_groups import CallbackGroup
from rclpy.client import Client
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity, EntityFactory
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity

from station_workplace.exceptions.world_manager_error import WorldManagerError
from station_workplace.interface.world_manager import WorldManager
from station_workplace.models.simulation_config_dto import SimulationConfigDTO
from station_workplace.utils.async_service_caller import call_service


class GazeboWorldManager(WorldManager):
    """Spawn/remove models via the bridged Gazebo world ``create``/``remove`` (blocking) services.

    Created once per node; the record of spawned models survives
    ``deactivate``/``activate`` cycles. The clients exist only while active.
    """

    def __init__(self, node: Node) -> None:
        """Create the manager without clients."""
        self._node = node
        self._logger = node.get_logger()
        self._lock = threading.Lock()
        self._spawned: list[str] = []
        self._timeout_sec = 5.0
        self._create_client: Client | None = None
        self._remove_client: Client | None = None
        self._never_cancelled = threading.Event()

    def activate(
        self, config: SimulationConfigDTO, timeout_sec: float, callback_group: CallbackGroup,
    ) -> None:
        """Create the Gazebo world service clients."""
        self._timeout_sec = timeout_sec
        self._create_client = self._node.create_client(
            SpawnEntity, config.create_service, callback_group=callback_group)
        self._remove_client = self._node.create_client(
            DeleteEntity, config.remove_service, callback_group=callback_group)

    def deactivate(self) -> None:
        """Destroy the clients; the record of spawned models is kept."""
        for name in ('_create_client', '_remove_client'):
            client = getattr(self, name)
            if client is not None:
                self._node.destroy_client(client)
                setattr(self, name, None)

    @property
    def spawned_models(self) -> tuple[str, ...]:
        """Return the names of the spawned models in spawn order."""
        with self._lock:
            return tuple(self._spawned)

    def spawn_model(self, name: str, sdf: str, pose: Pose) -> None:
        """Spawn a model at a world pose and record it (see :class:`WorldManager`)."""
        factory = EntityFactory()
        factory.name = name
        factory.allow_renaming = False
        factory.sdf = sdf
        factory.pose = pose
        factory.relative_to = 'world'
        request = SpawnEntity.Request(entity_factory=factory)
        try:
            response = call_service(
                self._require(self._create_client), request, self._timeout_sec,
                self._never_cancelled, WorldManagerError, f"spawn model '{name}'")
        except WorldManagerError:
            # Gazebo may still create the model after a lost/late response.
            self._remove_best_effort(name)
            raise
        if not response.success:
            raise WorldManagerError(f"Gazebo rejected spawning model '{name}'")
        with self._lock:
            self._spawned.append(name)
        self._logger.info(f"Spawned model '{name}'")

    def _remove_best_effort(self, name: str) -> None:
        """Try to remove an unrecorded model; failures are only logged."""
        if self._remove_client is None:
            return
        request = DeleteEntity.Request(entity=Entity(name=name, type=Entity.MODEL))
        try:
            call_service(
                self._remove_client, request, self._timeout_sec, self._never_cancelled,
                WorldManagerError, f"remove model '{name}'")
        except Exception as exc:  # noqa: BLE001 - best effort only
            self._logger.warning(f"Best-effort removal of model '{name}' failed: {exc}")

    def remove_model(self, name: str) -> None:
        """Remove a model from the world and forget it (see :class:`WorldManager`)."""
        request = DeleteEntity.Request(entity=Entity(name=name, type=Entity.MODEL))
        response = call_service(
            self._require(self._remove_client), request, self._timeout_sec,
            self._never_cancelled, WorldManagerError, f"remove model '{name}'")
        if not response.success:
            raise WorldManagerError(f"Gazebo rejected removing model '{name}'")
        with self._lock:
            if name in self._spawned:
                self._spawned.remove(name)
        self._logger.info(f"Removed model '{name}'")

    @staticmethod
    def _require(client: Client | None) -> Client:
        if client is None:
            raise WorldManagerError('Gazebo world manager is not active')
        return client
