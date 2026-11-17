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
"""Client of the movement controller."""

import threading
import uuid
from collections.abc import Sequence

from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.client import Client
from rclpy.node import Node

from movement_controller.action import ExecuteTrajectory
from movement_controller.msg import TrajectoryPath
from movement_controller.services.movement_session import MovementSession
from movement_controller.srv import CloseSession, InSession

from localization.enums.constants import MOVE_TIMEOUT_S, PLANNING_FRAME, SERVER_WAIT_TIMEOUT_S
from localization.exceptions import MotionError
from localization.services.tf_lookup import TfLookup
from localization.utils.async_action import call_service, execute_goal
from localization.utils.pose_conversion import pose6_to_pose_stamped


class MotionClient:
    """Moves the camera TCP to poses with the movement controller."""

    def __init__(
        self,
        node: Node,
        action_name: str,
        tool_frame: str,
        tf_lookup: TfLookup,
        callback_group: CallbackGroup,
        in_session_name: str,
        close_session_name: str,
    ) -> None:
        """:param tool_frame: Frame moved to the target poses (the camera TCP).
        :param in_session_name: Name of the movement controller ``in_session`` service.
        :param close_session_name: Name of the movement controller ``close_session`` service.
        """
        self._node = node
        self._action_name = action_name
        self._tool_frame = tool_frame
        self._tf_lookup = tf_lookup
        self._client: ActionClient | None = ActionClient(
            node, ExecuteTrajectory, action_name, callback_group=callback_group)
        self._in_session_client: Client | None = node.create_client(
            InSession, in_session_name, callback_group=callback_group)
        self._close_session_client: Client | None = node.create_client(
            CloseSession, close_session_name, callback_group=callback_group)
        self._session: MovementSession | None = None

    def open_session(self) -> None:
        """Reserve the movement controller for a sequence of movements.

        :raises MotionError: If the session could not be acquired.
        """
        if self._session is not None:
            raise MotionError('a movement session is already open')
        session = MovementSession(
            in_session_call=lambda request: self._call_session_service(self._in_session_client, request),
            close_session_call=lambda request: self._call_session_service(self._close_session_client, request),
            now_s=lambda: self._node.get_clock().now().nanoseconds / 1e9,
        )
        error = session.ensure_valid()
        if error:
            raise MotionError(f'could not open movement session: {error}')
        self._session = session

    def close_session(self) -> None:
        """Release the movement session; a failure is logged, not raised."""
        session, self._session = self._session, None
        if session is None:
            return
        error = session.close()
        if error:
            self._node.get_logger().error(f'Failed to release movement session: {error}')

    def _call_session_service(self, client: Client | None, request):
        if client is None:
            return None
        return call_service(client, request, timeout_s=SERVER_WAIT_TIMEOUT_S)

    def move_to(self, pose6: Sequence[float], frame_id: str, cancel_event: threading.Event) -> None:
        """Move the tool frame to ``pose6`` given in ``frame_id`` with a PTP movement.

        Requires an open session (:meth:`open_session`), which is extended when its
        timeout is close to expiry.

        :raises MotionError: If the movement could not be executed.
        :raises LocalizationCancelledError: If ``cancel_event`` was set.
        """
        if self._client is None:
            raise MotionError('motion client is not initialized')
        if not self._client.wait_for_server(timeout_sec=SERVER_WAIT_TIMEOUT_S):
            raise MotionError(f"movement action server '{self._action_name}' is not available")
        session = self._session
        if session is None:
            raise MotionError('no movement session is open')
        session_error = session.ensure_valid()
        if session_error:
            raise MotionError(f'movement session: {session_error}')

        # The planner can only resolve robot links, so express the target in its planning frame.
        target = self._tf_lookup.transform_pose(
            pose6_to_pose_stamped(pose6, frame_id), PLANNING_FRAME)
        target.header.stamp = self._node.get_clock().now().to_msg()

        path = TrajectoryPath()
        path.path_id = str(uuid.uuid4())
        path.motion_type = TrajectoryPath.MOTION_TYPE_PTP
        path.target_pose = target
        path.tool_frame = self._tool_frame
        goal = ExecuteTrajectory.Goal()
        goal.paths = [path]
        goal.session_id = session.session_id

        result = execute_goal(
            self._client, goal, timeout_s=MOVE_TIMEOUT_S, cancel_event=cancel_event,
            description='movement', error_cls=MotionError)
        if not result.success:
            raise MotionError(f'movement failed: {result.error_message}')

    def destroy(self) -> None:
        """Release any open session and destroy the action and service clients."""
        self.close_session()
        if self._in_session_client is not None:
            self._node.destroy_client(self._in_session_client)
            self._in_session_client = None
        if self._close_session_client is not None:
            self._node.destroy_client(self._close_session_client)
            self._close_session_client = None
        if self._client is not None:
            self._client.destroy()
            self._client = None
