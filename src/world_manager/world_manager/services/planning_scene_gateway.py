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
"""Planning-scene service gateway protocol and ROS 2 implementation."""

from time import monotonic, sleep
from typing import Any, Protocol

from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.callback_groups import CallbackGroup
from rclpy.client import Client
from rclpy.node import Node

from world_manager.exceptions import PlanningSceneError

_POLL_PERIOD_SEC = 0.01


class PlanningSceneGateway(Protocol):
    """Seam between the planning-scene operations and ROS service clients."""

    def get_scene(self, components: int) -> PlanningScene:
        """Return the requested planning-scene components or raise PlanningSceneError."""

    def apply_scene(self, scene: PlanningScene) -> None:
        """Apply one planning-scene diff or raise PlanningSceneError."""


class RosPlanningSceneGateway:
    """Call MoveIt's GetPlanningScene and ApplyPlanningScene services."""

    def __init__(
        self,
        node: Node,
        get_scene_service: str,
        apply_scene_service: str,
        timeout_sec: float,
        callback_group: CallbackGroup,
    ) -> None:
        """Create clients using the supplied reentrant callback group."""
        self._node = node
        self._timeout_sec = timeout_sec
        self._get_client: Client = node.create_client(
            GetPlanningScene, get_scene_service, callback_group=callback_group)
        self._apply_client: Client = node.create_client(
            ApplyPlanningScene, apply_scene_service, callback_group=callback_group)

    def destroy(self) -> None:
        """Destroy both clients during lifecycle cleanup."""
        self._node.destroy_client(self._get_client)
        self._node.destroy_client(self._apply_client)

    def get_scene(self, components: int) -> PlanningScene:
        """Fetch selected scene components."""
        request = GetPlanningScene.Request()
        request.components.components = components
        response = self._call(self._get_client, request, 'get planning scene')
        return response.scene

    def apply_scene(self, scene: PlanningScene) -> None:
        """Apply one diff and check MoveIt's response.success."""
        request = ApplyPlanningScene.Request()
        request.scene = scene
        response = self._call(self._apply_client, request, 'apply planning-scene diff')
        if not response.success:
            raise PlanningSceneError('MoveIt rejected the planning-scene diff')

    def _call(self, client: Client, request: Any, description: str) -> Any:
        deadline = monotonic() + self._timeout_sec
        while True:
            remaining = deadline - monotonic()
            if remaining <= 0:
                raise PlanningSceneError(
                    f"{description}: service '{client.srv_name}' not available within {self._timeout_sec} s")
            if client.wait_for_service(timeout_sec=min(0.1, remaining)):
                break

        future = client.call_async(request)
        deadline = monotonic() + self._timeout_sec
        while not future.done():
            remaining = deadline - monotonic()
            if remaining <= 0:
                client.remove_pending_request(future)
                future.cancel()
                raise PlanningSceneError(
                    f"{description}: no response from '{client.srv_name}' within {self._timeout_sec} s")
            sleep(min(_POLL_PERIOD_SEC, remaining))
        try:
            response = future.result()
        except Exception as e:
            raise PlanningSceneError(f'{description}: service call failed: {e}') from e
        if response is None:
            raise PlanningSceneError(f'{description}: service returned no response')
        return response
