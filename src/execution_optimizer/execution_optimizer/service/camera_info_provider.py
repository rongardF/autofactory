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

"""Fetches and caches the camera intrinsics through the camera's ``get_image`` action."""

import threading
import time

from pydantic import ValidationError
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node

from cameras.action import GetImage

from execution_optimizer.exceptions import CameraInfoError
from execution_optimizer.model.camera_intrinsics_dto import CameraIntrinsicsDTO

_CANCEL_ACK_TIMEOUT_S = 2.0


class _GoalOutcome:
    """Values handed from the action callbacks to the waiting thread."""

    def __init__(self) -> None:
        self.accepted = threading.Event()
        self.result_ready = threading.Event()
        self.handle: ClientGoalHandle | None = None
        self.error: Exception | None = None


class CameraInfoProvider:
    """Obtains the pinhole intrinsics once per activation and caches them.

    The provider is created when the node is configured, but does not touch ROS
    until :meth:`initialize` is called.
    """

    def __init__(
        self, node: Node, action_name: str, timeout_s: float, callback_group: CallbackGroup
    ) -> None:
        """Store the parameters only; nothing is created yet.

        :param node: Node that owns the action client.
        :param action_name: Name of the ``GetImage`` action.
        :param timeout_s: Timeout for the action server and for the result.
        :param callback_group: Callback group of the action client; it must differ from the
            group that runs the (blocking) lifecycle transition callbacks.
        """
        self._node = node
        self._action_name = action_name
        self._timeout_s = timeout_s
        self._callback_group = callback_group
        self._client: ActionClient | None = None
        self._intrinsics: CameraIntrinsicsDTO | None = None

    @property
    def intrinsics(self) -> CameraIntrinsicsDTO | None:
        """Cached intrinsics, ``None`` while not initialized."""
        return self._intrinsics

    def initialize(self) -> CameraIntrinsicsDTO:
        """Create the action client, request one image and cache the intrinsics.

        All waits are bounded by the timeout; results arrive through callbacks.

        :raises CameraInfoError: If the camera is unavailable or returns unusable data.
        """
        self.deinitialize()
        self._client = ActionClient(
            self._node, GetImage, self._action_name, callback_group=self._callback_group
        )
        try:
            self._intrinsics = self._fetch_intrinsics(self._client)
        except Exception:
            self.deinitialize()
            raise
        return self._intrinsics

    def deinitialize(self) -> None:
        """Destroy the action client and drop the cached intrinsics (idempotent)."""
        self._intrinsics = None
        if self._client is not None:
            self._client.destroy()
            self._client = None

    def _fetch_intrinsics(self, client: ActionClient) -> CameraIntrinsicsDTO:
        deadline = time.monotonic() + self._timeout_s
        if not client.wait_for_server(timeout_sec=self._timeout_s):
            raise CameraInfoError(f"Camera action server '{self._action_name}' not available")

        outcome = _GoalOutcome()

        def on_goal_response(future) -> None:
            try:
                outcome.handle = future.result()
            except Exception as error:  # noqa: BLE001 - reported through CameraInfoError
                outcome.error = error
            outcome.accepted.set()

        client.send_goal_async(GetImage.Goal()).add_done_callback(on_goal_response)
        if not outcome.accepted.wait(max(deadline - time.monotonic(), 0.0)):
            raise CameraInfoError(
                f"Camera action '{self._action_name}' did not answer within {self._timeout_s} s"
            )
        handle = outcome.handle
        if handle is None:
            raise CameraInfoError(f'Could not send the image goal: {outcome.error}')
        if not handle.accepted:
            raise CameraInfoError('The camera rejected the image goal')

        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda _future: outcome.result_ready.set())
        if not outcome.result_ready.wait(max(deadline - time.monotonic(), 0.0)):
            cancelled = threading.Event()
            handle.cancel_goal_async().add_done_callback(lambda _future: cancelled.set())
            cancelled.wait(_CANCEL_ACK_TIMEOUT_S)
            raise CameraInfoError(f'No image from the camera within {self._timeout_s} s')

        result = result_future.result().result
        if not result.success:
            raise CameraInfoError(f'Image capture failed: {result.error_message}')
        info = result.camera_info
        try:
            return CameraIntrinsicsDTO(
                fx=info.k[0], cx=info.k[2], fy=info.k[4], cy=info.k[5],
                width=info.width, height=info.height,
            )
        except ValidationError as error:
            raise CameraInfoError(f'Unusable camera intrinsics: {error}') from error
