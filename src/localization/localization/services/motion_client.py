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
from rclpy.node import Node

from movement_controller.action import ExecuteTrajectory
from movement_controller.msg import TrajectoryPath

from localization.enums.constants import MOVE_TIMEOUT_S, PLANNING_FRAME, SERVER_WAIT_TIMEOUT_S
from localization.exceptions import MotionError
from localization.services.tf_lookup import TfLookup
from localization.utils.async_action import execute_goal
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
    ) -> None:
        """:param tool_frame: Frame moved to the target poses (the camera TCP)."""
        self._node = node
        self._action_name = action_name
        self._tool_frame = tool_frame
        self._tf_lookup = tf_lookup
        self._client: ActionClient | None = ActionClient(
            node, ExecuteTrajectory, action_name, callback_group=callback_group)

    def move_to(self, pose6: Sequence[float], frame_id: str, cancel_event: threading.Event) -> None:
        """Move the tool frame to ``pose6`` given in ``frame_id`` with a PTP movement.

        :raises MotionError: If the movement could not be executed.
        :raises LocalizationCancelledError: If ``cancel_event`` was set.
        """
        if self._client is None:
            raise MotionError('motion client is not initialized')
        if not self._client.wait_for_server(timeout_sec=SERVER_WAIT_TIMEOUT_S):
            raise MotionError(f"movement action server '{self._action_name}' is not available")

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

        result = execute_goal(
            self._client, goal, timeout_s=MOVE_TIMEOUT_S, cancel_event=cancel_event,
            description='movement', error_cls=MotionError)
        if not result.success:
            raise MotionError(f'movement failed: {result.error_message}')

    def destroy(self) -> None:
        """Destroy the action client."""
        if self._client is not None:
            self._client.destroy()
            self._client = None
