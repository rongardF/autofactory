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
"""``~/localize_product`` action server."""

import threading
import time
from collections.abc import Callable

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node

from localization.action import LocalizeProduct
from localization.enums.constants import PRODUCT_ORIGIN_FRAME
from localization.interface.localization_service import LocalizationService
from localization.models.localization_result_dto import (
    LocalizationRequestDTO,
    LocalizationResultDTO,
)
from localization.utils.uuid_validator import parse_uuid4

_STOP_TIMEOUT_S = 10.0
# Time for the executor to send the result before the server is destroyed.
_RESULT_DELIVERY_GRACE_S = 0.3


class LocalizeProductActionServer:
    """Runs one localization at a time and reports its outcome through the action result."""

    def __init__(
        self,
        node: Node,
        service: LocalizationService,
        callback_group: CallbackGroup,
        on_goal_accepted: Callable[[], None],
        on_succeeded: Callable[[], None],
    ) -> None:
        """Create the action server.

        :param on_goal_accepted: Called when a goal is accepted; the previous localization is void.
        :param on_succeeded: Called when a localization finished successfully.
        """
        self._node = node
        self._service = service
        self._on_goal_accepted = on_goal_accepted
        self._on_succeeded = on_succeeded
        self._lock = threading.Lock()
        self._active = False
        self._cancel_event = threading.Event()
        self._idle = threading.Event()
        self._idle.set()
        self._server: ActionServer | None = ActionServer(
            node, LocalizeProduct, f'{node.get_name()}/localize_product',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=callback_group,
        )

    def destroy(self) -> None:
        """Cancel a running goal, wait for its result to be handed over and destroy the server."""
        self._cancel_event.set()
        was_running = not self._idle.is_set()
        self._idle.wait(_STOP_TIMEOUT_S)
        if was_running:
            time.sleep(_RESULT_DELIVERY_GRACE_S)
        if self._server is not None:
            self._server.destroy()
            self._server = None

    def _on_goal(self, goal: LocalizeProduct.Goal) -> GoalResponse:
        logger = self._node.get_logger()
        try:
            parse_uuid4(goal.assembly_model_id, 'assembly_model_id')
        except ValueError as exc:
            logger.error(f'Goal rejected: {exc}')
            return GoalResponse.REJECT
        state = self._service.workplace_state()
        if state is None or not state.in_place:
            logger.error('Goal rejected: no product in place in the workplace')
            return GoalResponse.REJECT
        with self._lock:
            if self._active:
                logger.error('Goal rejected: a localization is already running')
                return GoalResponse.REJECT
            self._active = True
            self._idle.clear()
            self._cancel_event = threading.Event()
        self._on_goal_accepted()
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle: ServerGoalHandle) -> CancelResponse:
        self._cancel_event.set()
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle: ServerGoalHandle) -> LocalizeProduct.Result:
        cancel_event = self._cancel_event

        def publish_feedback(done: bool, collection_time_s: int) -> None:
            feedback = LocalizeProduct.Feedback()
            feedback.collection_done = done
            feedback.collection_time_s = collection_time_s
            goal_handle.publish_feedback(feedback)

        try:
            outcome = self._service.localize(
                LocalizationRequestDTO(assembly_model_id=goal_handle.request.assembly_model_id),
                publish_feedback, cancel_event)
        except Exception as exc:  # noqa: BLE001 - failures are reported through the result
            self._node.get_logger().error(f'Localization failed unexpectedly: {exc}')
            outcome = LocalizationResultDTO(error_message=f'unexpected error: {exc}')

        result = LocalizeProduct.Result()
        result.localized_product_frame = PRODUCT_ORIGIN_FRAME
        result.localization_time_s = outcome.localization_time_s
        result.success = outcome.success
        result.error_message = outcome.error_message
        if outcome.success:
            goal_handle.succeed()
            self._on_succeeded()
        elif outcome.cancelled and goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        with self._lock:
            self._active = False
            self._idle.set()
        return result
