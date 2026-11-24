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
"""gz-transport implementation of the adapter, using only the ``/blocking`` service variants.

Service requests run in a helper process (see ``GzRequestProcess``); subscriptions and publishers
stay on the in-process node.
"""

from threading import Lock
from typing import Callable

from rclpy.impl.rcutils_logger import RcutilsLogger

from gz.msgs10.boolean_pb2 import Boolean  # type: ignore[attr-defined]
from gz.msgs10.empty_pb2 import Empty  # type: ignore[attr-defined]
from gz.msgs10.entity_factory_pb2 import EntityFactory  # type: ignore[attr-defined]
from gz.msgs10.entity_pb2 import Entity  # type: ignore[attr-defined]
from gz.msgs10.pose_pb2 import Pose  # type: ignore[attr-defined]
from gz.msgs10.scene_pb2 import Scene  # type: ignore[attr-defined]
from gz.msgs10.stringmsg_pb2 import StringMsg  # type: ignore[attr-defined]
from gz.transport13 import Node as GzNode

from world_manager.models import PoseDTO
from world_manager.services.gz_request_process import GzRequestProcess
from world_manager.services.gz_transport_adapter import GzTransportAdapter

def _to_ms(timeout_sec: float) -> int:
    return max(1, int(timeout_sec * 1000))


class RealGzTransportAdapter(GzTransportAdapter):
    """Talks to Gazebo through the gz-transport Python bindings."""

    def __init__(self, logger: RcutilsLogger) -> None:
        """Create the gz-transport node."""
        self._node = GzNode()
        self._requests = GzRequestProcess(logger)
        self._logger = logger
        self._publishers: dict[str, object] = {}
        self._subscribed: set[str] = set()
        self._lock = Lock()

    def close(self) -> None:
        """Stop the request helper process."""
        self._requests.close()

    def _call(self, service: str, request: object, request_type: type, response_type: type,
              timeout_sec: float) -> object | None:
        """Send a request; returns the parsed reply, or None if the service did not answer."""
        ok, payload = self._requests.request(
            service, request.SerializeToString(), request_type.DESCRIPTOR.full_name,
            response_type.DESCRIPTOR.full_name, _to_ms(timeout_sec))
        if not ok:
            return None
        reply = response_type()
        reply.ParseFromString(payload)
        return reply

    def _request(self, service: str, request: object, request_type: type, timeout_sec: float) -> bool:
        reply = self._call(service, request, request_type, Boolean, timeout_sec)
        if reply is None:
            self._logger.warning(f'gz request {service} got no answer')
            return False
        if not reply.data:
            self._logger.warning(f'gz request {service} was refused')
        return bool(reply.data)

    def create_model(self, world: str, model_id: str, sdf: str, pose: PoseDTO, timeout_sec: float) -> bool:
        """Create a model; never allows Gazebo to rename it."""
        factory = EntityFactory()
        factory.name = model_id
        factory.allow_renaming = False
        factory.sdf = sdf
        factory.relative_to = 'world'
        factory.pose.CopyFrom(PoseDTO.to_gazebo_msg(pose))
        return self._request(f'/world/{world}/create/blocking', factory, EntityFactory, timeout_sec)

    def remove_model(self, world: str, model_id: str, timeout_sec: float) -> bool:
        """Remove a model by name."""
        entity = Entity()
        entity.name = model_id
        entity.type = Entity.MODEL
        return self._request(f'/world/{world}/remove/blocking', entity, Entity, timeout_sec)

    def set_model_pose(self, world: str, model_id: str, pose: PoseDTO, timeout_sec: float) -> bool:
        """Set the world pose of a model."""
        msg = PoseDTO.to_gazebo_msg(pose)
        msg.name = model_id
        return self._request(f'/world/{world}/set_pose/blocking', msg, Pose, timeout_sec)

    def list_models(self, world: str, timeout_sec: float) -> list[str] | None:
        """Read the model names from scene/info."""
        scene = self._call(f'/world/{world}/scene/info', Empty(), Empty, Scene, timeout_sec)
        if scene is None:
            return None
        return [model.name for model in scene.model]

    def subscribe_state(self, topic: str, callback: Callable[[str], None]) -> bool:
        """Subscribe once per topic; further calls for the same topic are no-ops."""
        with self._lock:
            if topic in self._subscribed:
                return True
            if not self._node.subscribe(StringMsg, topic, lambda msg: callback(msg.data)):
                return False
            self._subscribed.add(topic)
            return True

    def unsubscribe(self, topic: str) -> None:
        """Drop the subscription of ``topic``."""
        with self._lock:
            if topic in self._subscribed:
                self._node.unsubscribe(topic)
                self._subscribed.discard(topic)

    def publish_empty(self, topic: str) -> None:
        """Publish one Empty message, advertising the topic on first use."""
        with self._lock:
            publisher = self._publishers.get(topic)
            if publisher is None:
                publisher = self._node.advertise(topic, Empty)
                self._publishers[topic] = publisher
        publisher.publish(Empty())
