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
# 3. Neither the name of the copyright holder nor its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND CONTRIBUTORS "AS IS"
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
"""Typed client for moveit2_world_manager."""

from threading import Event
from typing import Sequence

from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import CallbackGroup
from rclpy.client import Client
from rclpy.node import Node
from world_manager.exceptions import WorldManagerError
from world_manager.models import (
    ObjectInfoDTO,
    WorldChangesDTO,
)
from world_manager.srv import ApplyWorldChanges, AttachObject, ListObjects
from world_manager.utils import call_service


class MoveIt2WorldClient:
    """Blocking typed wrapper around the MoveIt planning-scene manager."""

    def __init__(
        self,
        node: Node,
        callback_group: CallbackGroup,
        timeout_sec: float = 15.0,
    ) -> None:
        """Create clients; service availability is checked with a timeout on each call."""
        self._node = node
        self._timeout_sec = timeout_sec
        prefix = '/moveit2_world_manager'
        self._apply: Client = node.create_client(
            ApplyWorldChanges, f'{prefix}/apply_world_changes', callback_group=callback_group)
        self._attach: Client = node.create_client(
            AttachObject, f'{prefix}/attach_object', callback_group=callback_group)
        self._list: Client = node.create_client(
            ListObjects, f'{prefix}/list_objects', callback_group=callback_group)
        self._clients = [self._apply, self._attach, self._list]

    def destroy(self) -> None:
        """Destroy the service clients."""
        for client in self._clients:
            self._node.destroy_client(client)

    def apply(self, changes: WorldChangesDTO, cancel: Event | None = None) -> None:
        """Apply one planning-scene change batch."""
        request = WorldChangesDTO.to_msg(changes)
        response = call_service(self._apply, request, self._timeout_sec, cancel)
        self._raise_for_failure(response.success, response.error_message)

    def attach(
        self,
        object_id: str,
        link_name: str,
        pose: PoseStamped,
        touch_links: Sequence[str],
        cancel: Event | None = None,
    ) -> None:
        """Move an existing collision object between a link and the planning-scene world."""
        request = AttachObject.Request()
        request.object_id = object_id
        request.link_name = link_name
        request.pose = pose
        request.touch_links = list(touch_links)
        response = call_service(self._attach, request, self._timeout_sec, cancel)
        self._raise_for_failure(response.success, response.error_message)

    def list_objects(self, cancel: Event | None = None) -> list[ObjectInfoDTO]:
        """List collision object ids and attachment links from MoveIt."""
        response = call_service(self._list, ListObjects.Request(), self._timeout_sec, cancel)
        self._raise_for_failure(response.success, response.error_message)
        return [ObjectInfoDTO.from_msg(item) for item in response.objects]

    @staticmethod
    def _raise_for_failure(success: bool, error_message: str) -> None:
        if not success:
            raise WorldManagerError(error_message or 'MoveIt world manager operation failed')
