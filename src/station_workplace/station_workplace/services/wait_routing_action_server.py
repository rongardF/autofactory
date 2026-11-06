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

import threading
import time

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import CallbackGroup
from rclpy.client import Client
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger

from station_workplace.action import WaitRouting
from station_workplace.enums.process_type_enum import ProcessTypeEnum
from station_workplace.exceptions.process_cancelled_error import ProcessCancelledError
from station_workplace.exceptions.routing_error import RoutingError
from station_workplace.models.common_node_config_dto import CommonNodeConfigDTO
from station_workplace.models.product_data_dto import ProductDataDTO
from station_workplace.models.routing_result_dto import RoutingResultDTO
from station_workplace.models.workplace_state_dto import WorkplaceStateDTO
from station_workplace.services.workplace_state_tracker import WorkplaceStateTracker
from station_workplace.utils.async_service_caller import call_service

UNKNOWN_PRODUCT_MESSAGE = 'Unknown product in station workplace'
CANCELLED_MESSAGE = 'wait_routing cancelled by the client'
DEACTIVATED_MESSAGE = 'wait_routing aborted: node deactivated'


class _ActiveGoal:
    """Stop signals of the single goal being handled."""

    def __init__(self) -> None:
        # Set on a client cancel request or by deactivation.
        self.stop = threading.Event()
        self.aborted_by_node = False
        self.finished = threading.Event()


class WaitRoutingActionServer:
    """``~/wait_routing`` action: route a product in/out and wait for the workplace to settle.

    One goal at a time; further goals are rejected. Goals are only accepted
    while active. :meth:`deactivate` aborts the running goal and waits until its
    result has been handed to rclpy before destroying the server, so the client
    always receives a result.
    """

    _POLL_SEC = 0.05
    _CANCEL_ACK_TIMEOUT_SEC = 1.0
    _EXECUTION_STOP_TIMEOUT_SEC = 5.0
    # Time for the executor to send the result response before the server is destroyed.
    _RESULT_DELIVERY_GRACE_SEC = 0.5

    def __init__(
        self,
        node: Node,
        config: CommonNodeConfigDTO,
        tracker: WorkplaceStateTracker,
        action_callback_group: CallbackGroup,
        client_callback_group: CallbackGroup,
    ) -> None:
        """Store collaborators; the server and clients are created in :meth:`activate`.

        :param node: Node owning the action server and the clients.
        :param config: Validated node configuration.
        :param tracker: Tracker of the settled workplace state.
        :param action_callback_group: Reentrant group of the action server callbacks.
        :param client_callback_group: Group of the product carrier service clients.
        """
        self._node = node
        self._logger = node.get_logger()
        self._config = config
        self._tracker = tracker
        self._action_group = action_callback_group
        self._client_group = client_callback_group
        self._lock = threading.Lock()
        self._active = False
        self._goal: _ActiveGoal | None = None
        self._server: ActionServer | None = None
        self._inject_client: Client | None = None
        self._eject_client: Client | None = None

    # region lifecycle
    def activate(self) -> None:
        """Create the product carrier clients and the action server."""
        self._inject_client = self._node.create_client(
            Trigger, self._config.inject_product_service, callback_group=self._client_group)
        self._eject_client = self._node.create_client(
            Trigger, self._config.eject_product_service, callback_group=self._client_group)
        with self._lock:
            self._active = True
            self._goal = None
        self._server = ActionServer(
            self._node,
            WaitRouting,
            '~/wait_routing',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._action_group,
        )

    def deactivate(self) -> None:
        """Abort the running goal, then destroy the action server and the clients."""
        with self._lock:
            self._active = False
            goal = self._goal
            if goal is not None:
                goal.aborted_by_node = True
                goal.stop.set()
        if goal is not None:
            if goal.finished.wait(self._EXECUTION_STOP_TIMEOUT_SEC):
                time.sleep(self._RESULT_DELIVERY_GRACE_SEC)
            else:
                self._logger.error('wait_routing goal did not stop in time')
        if self._server is not None:
            self._server.destroy()
            self._server = None
        for name in ('_inject_client', '_eject_client'):
            client = getattr(self, name)
            if client is not None:
                self._node.destroy_client(client)
                setattr(self, name, None)
    # endregion lifecycle

    # region action callbacks
    def _on_goal(self, goal_request: WaitRouting.Goal) -> GoalResponse:
        operation = goal_request.routing_operation
        if operation not in {member.value for member in ProcessTypeEnum}:
            self._logger.warning(f"Rejected wait_routing goal: invalid routing_operation '{operation}'")
            return GoalResponse.REJECT
        with self._lock:
            if not self._active:
                self._logger.warning('Rejected wait_routing goal: node is not active')
                return GoalResponse.REJECT
            if self._goal is not None:
                self._logger.warning('Rejected wait_routing goal: another goal is running')
                return GoalResponse.REJECT
            self._goal = _ActiveGoal()
        self._logger.info(f"Accepted wait_routing goal: routing_operation='{operation}'")
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        with self._lock:
            goal = self._goal
        if goal is not None:
            goal.stop.set()
        self._logger.info('wait_routing cancel requested')
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle: ServerGoalHandle) -> WaitRouting.Result:
        start = self._now()
        with self._lock:
            goal = self._goal
        if goal is None:
            # Not reachable: every accepted goal registers itself in _on_goal.
            goal = _ActiveGoal()
        cancelled = False
        try:
            operation = ProcessTypeEnum(goal_handle.request.routing_operation)
            outcome = self._route(operation, goal)
        except ProcessCancelledError:
            cancelled = True
            message = DEACTIVATED_MESSAGE if goal.aborted_by_node else CANCELLED_MESSAGE
            outcome = RoutingResultDTO(success=False, error_message=message)
        except Exception as exc:  # noqa: BLE001 - convert to a failed result at the boundary
            self._logger.error(f"Exception during wait_routing: {str(exc)}")
            outcome = RoutingResultDTO(success=False, error_message="Unknown exception")

        try:
            outcome = outcome.model_copy(update={'routing_time_s': self._elapsed_s(start)})
            self._finish(goal_handle, goal, outcome, cancelled)
            return outcome.to_msg()
        finally:
            with self._lock:
                if self._goal is goal:
                    self._goal = None
            goal.finished.set()
    # endregion action callbacks

    # region routing
    def _route(self, operation: ProcessTypeEnum, goal: _ActiveGoal) -> RoutingResultDTO:
        """Trigger the carrier and wait for the workplace to settle.

        :raises ProcessCancelledError: If the goal is cancelled or the node deactivates.
        :raises RoutingError: If the product carrier service call fails.
        """
        if goal.stop.is_set():
            raise ProcessCancelledError('stopped before start')
        waiter = self._tracker.wait_for(in_place=operation == ProcessTypeEnum.INJECTION)
        try:
            if waiter.is_settled:
                self._logger.info(
                    f'wait_routing {operation.value}: workplace already settled, '
                    'not calling the product carrier')
            else:
                self._trigger_carrier(operation, goal.stop)
                while not waiter.wait(self._POLL_SEC):
                    if goal.stop.is_set():
                        raise ProcessCancelledError(f'{operation.value} wait stopped')
            state = waiter.state
        finally:
            self._tracker.discard(waiter)
        return self._outcome_from_state(operation, state)

    def _trigger_carrier(self, operation: ProcessTypeEnum, stop: threading.Event) -> None:
        if operation == ProcessTypeEnum.INJECTION:
            client, description = self._inject_client, 'inject product'
        else:
            client, description = self._eject_client, 'eject product'
        if client is None:
            raise RoutingError('wait_routing is not active')
        response = call_service(
            client, Trigger.Request(), self._config.service_call_timeout_sec,
            stop, RoutingError, description)
        if not response.success:
            raise RoutingError(f"'{client.srv_name}' failed: {response.message}")
        self._logger.info(f"'{client.srv_name}' succeeded: {response.message}")

    @staticmethod
    def _outcome_from_state(
        operation: ProcessTypeEnum, state: WorkplaceStateDTO | None,
    ) -> RoutingResultDTO:
        if operation == ProcessTypeEnum.EJECTION:
            return RoutingResultDTO(success=True)
        product = None if state is None else state.product_data
        if product is None or product == ProductDataDTO.unknown():
            return RoutingResultDTO(
                success=False, error_message=UNKNOWN_PRODUCT_MESSAGE,
                product_data=ProductDataDTO.unknown())
        return RoutingResultDTO(success=True, product_data=product)
    # endregion routing

    # region helpers
    def _finish(
        self,
        goal_handle: ServerGoalHandle,
        goal: _ActiveGoal,
        outcome: RoutingResultDTO,
        cancelled: bool,
    ) -> None:
        """Set the terminal goal state and log the outcome."""
        if outcome.success:
            self._logger.info(f'wait_routing succeeded: {outcome}')
            goal_handle.succeed()
            return
        self._logger.error(f'wait_routing failed: {outcome.error_message}')
        if cancelled and not goal.aborted_by_node and self._await_cancel_ack(goal_handle):
            goal_handle.canceled()
        else:
            goal_handle.abort()

    def _await_cancel_ack(self, goal_handle: ServerGoalHandle) -> bool:
        """Wait until rclpy moved the goal to CANCELING after accepting the request."""
        deadline = time.monotonic() + self._CANCEL_ACK_TIMEOUT_SEC
        while not goal_handle.is_cancel_requested:
            if time.monotonic() > deadline:
                return False
            time.sleep(0.01)
        return True

    def _now(self) -> Time:
        return self._node.get_clock().now()

    def _elapsed_s(self, start: Time) -> int:
        return max(0, (self._now() - start).nanoseconds // 1_000_000_000)
    # endregion helpers
