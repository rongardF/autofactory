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
"""Client of the camera."""

import threading

from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node

from sensor_msgs.msg import Image

from cameras.action import GetImage

from localization.enums.constants import SERVER_WAIT_TIMEOUT_S
from localization.exceptions import ImageCaptureError
from localization.utils.async_action import execute_goal


class CameraClient:
    """Captures undistorted images with the camera's ``get_image`` action."""

    def __init__(
        self, node: Node, action_name: str, timeout_s: float, callback_group: CallbackGroup
    ) -> None:
        """:param timeout_s: Time allowed for the camera to return an image."""
        self._action_name = action_name
        self._timeout_s = timeout_s
        self._client: ActionClient | None = ActionClient(
            node, GetImage, action_name, callback_group=callback_group)

    def capture(self, cancel_event: threading.Event) -> Image:
        """Capture an image taken after this call.

        :raises ImageCaptureError: If no image could be captured.
        :raises LocalizationCancelledError: If ``cancel_event`` was set.
        """
        if self._client is None:
            raise ImageCaptureError('camera client is not initialized')
        if not self._client.wait_for_server(timeout_sec=SERVER_WAIT_TIMEOUT_S):
            raise ImageCaptureError(f"camera action server '{self._action_name}' is not available")
        result = execute_goal(
            self._client, GetImage.Goal(), timeout_s=self._timeout_s, cancel_event=cancel_event,
            description='image capture', error_cls=ImageCaptureError)
        if not result.success:
            raise ImageCaptureError(f'image capture failed: {result.error_message}')
        return result.image

    def destroy(self) -> None:
        """Destroy the action client."""
        if self._client is not None:
            self._client.destroy()
            self._client = None
