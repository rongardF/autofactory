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
"""Typed client for gazebo_world_manager."""

from threading import Event
from typing import Sequence

from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import CallbackGroup
from rclpy.client import Client
from rclpy.node import Node

from world_manager.exceptions import WorldManagerError
from world_manager.models import (
    DeleteResultDTO,
    ModelInfoDTO,
    ModelSpecDTO,
)
from world_manager.srv import DeleteModels, ListModels, SetModelAttachment, SetModelPose, SpawnModels
from world_manager.utils import call_service


class GazeboWorldClient:
    """Blocking typed wrapper around the Gazebo world manager services."""

    def __init__(
        self,
        node: Node,
        callback_group: CallbackGroup,
        timeout_sec: float = 15.0,
    ) -> None:
        """Create clients; service availability is checked with a timeout on each call."""
        self._timeout_sec = timeout_sec
        prefix = '/gazebo_world_manager'
        self._spawn: Client = node.create_client(
            SpawnModels, f'{prefix}/spawn_models', callback_group=callback_group)
        self._delete: Client = node.create_client(
            DeleteModels, f'{prefix}/delete_models', callback_group=callback_group)
        self._set_pose: Client = node.create_client(
            SetModelPose, f'{prefix}/set_model_pose', callback_group=callback_group)
        self._set_attachment: Client = node.create_client(
            SetModelAttachment, f'{prefix}/set_model_attachment', callback_group=callback_group)
        self._list: Client = node.create_client(
            ListModels, f'{prefix}/list_models', callback_group=callback_group)
        self._clients = [self._spawn, self._delete, self._set_pose, self._set_attachment, self._list]
        self._node = node

    def destroy(self) -> None:
        """Destroy the service clients."""
        for client in self._clients:
            self._node.destroy_client(client)

    def spawn(
        self,
        models: Sequence[ModelSpecDTO],
        owner: str,
        cancel: Event | None = None,
    ) -> list[str]:
        """Spawn models as one batch and return their ids."""
        request = SpawnModels.Request()
        request.owner = owner
        request.models = [ModelSpecDTO.to_msg(model) for model in models]
        response = call_service(self._spawn, request, self._timeout_sec, cancel)
        self._raise_for_failure(response.success, response.error_message)
        return list(response.spawned_ids)

    def delete(
        self,
        model_ids: Sequence[str] = (),
        owner: str = '',
        cancel: Event | None = None,
    ) -> DeleteResultDTO:
        """Delete ids or all registered models of an owner."""
        request = DeleteModels.Request()
        request.model_ids = list(model_ids)
        request.owner = owner
        response = call_service(self._delete, request, self._timeout_sec, cancel)
        self._raise_for_failure(response.success, response.error_message)
        return DeleteResultDTO.from_msg(response)

    def set_pose(self, model_id: str, pose: PoseStamped, cancel: Event | None = None) -> None:
        """Set a registered model's pose."""
        request = SetModelPose.Request()
        request.model_id = model_id
        request.pose = pose
        response = call_service(self._set_pose, request, self._timeout_sec, cancel)
        self._raise_for_failure(response.success, response.error_message)

    def set_attachment(self, model_id: str, link_name: str, cancel: Event | None = None) -> None:
        """Attach the named joint or release all joints when ``link_name`` is empty."""
        request = SetModelAttachment.Request()
        request.model_id = model_id
        request.link_name = link_name
        response = call_service(self._set_attachment, request, self._timeout_sec, cancel)
        self._raise_for_failure(response.success, response.error_message)

    def list_models(self, owner: str = '') -> list[ModelInfoDTO]:
        """List registered models and their observable joint states."""
        request = ListModels.Request()
        request.owner = owner
        response = call_service(self._list, request, self._timeout_sec)
        self._raise_for_failure(response.success, response.error_message)
        return [ModelInfoDTO.from_msg(model) for model in response.models]

    @staticmethod
    def _raise_for_failure(success: bool, error_message: str) -> None:
        if not success:
            raise WorldManagerError(error_message or 'Gazebo world manager operation failed')
