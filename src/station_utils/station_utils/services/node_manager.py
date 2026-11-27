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
"""Drive and observe the lifecycle and parameters of another ROS 2 lifecycle node.

One :class:`NodeManager` manages exactly one node. It wraps the standard services
every lifecycle node exposes (``<node>/change_state``, ``<node>/get_state``,
``<node>/get_parameters``, ``<node>/set_parameters``), and reports state changes
from ``<node>/transition_event`` plus the node leaving the ROS graph (``OFFLINE``).
Transition errors the managed node publishes on ``/transition_error`` are attached
to failed transition results and can optionally be collected.

Blocking calls poll their future against the monotonic wall clock instead of
spinning, so they can run on a thread of the owning node's
:class:`~rclpy.executors.MultiThreadedExecutor` — even from inside a lifecycle
transition callback, and regardless of ``use_sim_time``. The service clients live
in their own reentrant callback group so their responses can be processed while
the caller's (usually mutually exclusive) callback group is blocked waiting.
"""

from __future__ import annotations

import time
from collections import deque
from collections.abc import Callable
from threading import Event, Lock
from typing import Any

from lifecycle_msgs.msg import State, Transition, TransitionEvent
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.client import Client
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.task import Future

from station_utils.autofactory_base_node import TRANSITION_ERROR_QOS, TRANSITION_ERROR_TOPIC
from station_utils.enums.lifecycle_state_enum import LifecycleStateEnum
from station_utils.models.node_manager_result_dto import (
    GetParameterResultDTO,
    GetParametersResultDTO,
    GetStateResultDTO,
    NodeManagerResultDTO,
)
from station_utils.msg import TransitionError
from station_utils.utils.node_names import fully_qualified_name

StateCallback = Callable[[LifecycleStateEnum], None]

CANCELLED_MESSAGE = 'cancelled'

_POLL_PERIOD_S = 0.01

_PRIMARY_STATES: dict[int, LifecycleStateEnum] = {
    State.PRIMARY_STATE_UNCONFIGURED: LifecycleStateEnum.UNCONFIGURED,
    State.PRIMARY_STATE_INACTIVE: LifecycleStateEnum.INACTIVE,
    State.PRIMARY_STATE_ACTIVE: LifecycleStateEnum.ACTIVE,
    State.PRIMARY_STATE_FINALIZED: LifecycleStateEnum.FINALIZED,
}

_TRANSITION_LABELS: dict[int, str] = {
    Transition.TRANSITION_CONFIGURE: 'configure',
    Transition.TRANSITION_ACTIVATE: 'activate',
    Transition.TRANSITION_DEACTIVATE: 'deactivate',
    Transition.TRANSITION_CLEANUP: 'cleanup',
}

# Primary state each transition leaves the node in. Used to confirm a transition
# whose response was lost: a freshly created client can hit a DDS discovery race on
# the response path (the request is delivered and executed, but the reply is
# dropped because the server has not yet discovered the client's response reader).
_EXPECTED_STATE: dict[int, LifecycleStateEnum] = {
    Transition.TRANSITION_CONFIGURE: LifecycleStateEnum.INACTIVE,
    Transition.TRANSITION_ACTIVATE: LifecycleStateEnum.ACTIVE,
    Transition.TRANSITION_DEACTIVATE: LifecycleStateEnum.INACTIVE,
    Transition.TRANSITION_CLEANUP: LifecycleStateEnum.UNCONFIGURED,
}


class _CallOutcome:
    """Response of a service call, or why there is none."""

    def __init__(self, response: Any = None, error: str = '', sent: bool = False) -> None:
        self.response = response
        self.error = error
        self.sent = sent
        """True if the request reached the call stage (the service was available)."""


class _TransitionErrorWaiter:
    """Receives the first transition error of the managed node after its creation."""

    def __init__(self) -> None:
        self.received = Event()
        self.message: TransitionError | None = None


class NodeManager:
    """Manage the lifecycle and parameters of one external lifecycle node."""

    def __init__(
        self,
        node: Node,
        managed_node_name: str,
        service_timeout_s: float = 5.0,
        graph_check_period_s: float = 1.0,
        transition_timeout_s: float | None = None,
        transition_error_wait_s: float = 3.0,
        transition_errors_maxlen: int = 3,
    ) -> None:
        """Create the clients, the subscriptions and the graph check.

        :param node: Owning node; creates the clients, subscriptions and timer and
            provides the logger. It must be spun by an executor for state
            notifications, transition errors and service responses.
        :param managed_node_name: Name of the managed node; made fully qualified.
        :param service_timeout_s: Bound for service availability and for each response.
        :param graph_check_period_s: Period (wall clock) of the presence check that
            detects the managed node leaving or (re)joining the ROS graph.
        :param transition_timeout_s: Bound for the response of a transition request;
            defaults to ``service_timeout_s``.
        :param transition_error_wait_s: How long to wait, after a failed transition,
            for the managed node's ``TransitionError`` to arrive.
        :param transition_errors_maxlen: Number of newest errors kept by
            :meth:`monitor_transition_errors`.
        """
        self._node = node
        self._logger = node.get_logger()
        self._name = fully_qualified_name(managed_node_name)
        self._service_timeout_s = service_timeout_s
        self._transition_timeout_s = service_timeout_s if transition_timeout_s is None else transition_timeout_s
        self._transition_error_wait_s = transition_error_wait_s
        self._callback_group = ReentrantCallbackGroup()

        self._change_state_client = node.create_client(
            ChangeState, f'{self._name}/change_state', callback_group=self._callback_group
        )
        self._get_state_client = node.create_client(
            GetState, f'{self._name}/get_state', callback_group=self._callback_group
        )
        self._get_parameters_client = node.create_client(
            GetParameters, f'{self._name}/get_parameters', callback_group=self._callback_group
        )
        self._set_parameters_client = node.create_client(
            SetParameters, f'{self._name}/set_parameters', callback_group=self._callback_group
        )

        self._state_lock = Lock()
        self._observed_state: LifecycleStateEnum | None = None
        self._state_query_pending = False
        self._callbacks: dict[int, StateCallback] = {}
        self._next_callback_handle = 0

        self._errors_lock = Lock()
        self._monitoring = False
        self._transition_errors: deque[TransitionError] = deque(maxlen=transition_errors_maxlen)
        self._error_waiters: set[_TransitionErrorWaiter] = set()
        # Created once (not per transition): a new subscription needs DDS discovery
        # before it receives anything, which would make it miss fast failures.
        self._transition_error_subscription = node.create_subscription(
            TransitionError,
            TRANSITION_ERROR_TOPIC,
            self._on_transition_error,
            TRANSITION_ERROR_QOS,
            callback_group=self._callback_group,
        )
        self._transition_event_subscription = node.create_subscription(
            TransitionEvent,
            f'{self._name}/transition_event',
            self._on_transition_event,
            10,
            callback_group=self._callback_group,
        )
        # Steady clock: must keep ticking under use_sim_time even without /clock.
        self._graph_timer = node.create_timer(
            graph_check_period_s,
            self._check_presence,
            callback_group=self._callback_group,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    @property
    def node_name(self) -> str:
        """Fully-qualified name of the managed node."""
        return self._name

    def destroy(self) -> None:
        """Release all ROS entities; the manager must not be used afterwards."""
        self._node.destroy_timer(self._graph_timer)
        self._node.destroy_subscription(self._transition_event_subscription)
        self._node.destroy_subscription(self._transition_error_subscription)
        for client in (
            self._change_state_client,
            self._get_state_client,
            self._get_parameters_client,
            self._set_parameters_client,
        ):
            self._node.destroy_client(client)
        with self._state_lock:
            self._callbacks.clear()

    # ------------------------------------------------------------------ transition errors

    @property
    def transition_errors(self) -> list[TransitionError]:
        """Transition errors of the managed node collected while monitoring, newest first."""
        with self._errors_lock:
            return list(reversed(self._transition_errors))

    def monitor_transition_errors(self, enabled: bool) -> None:
        """Start or stop collecting the managed node's transition errors.

        Enabling clears the previously collected errors; disabling keeps them.
        Errors are attached to failed transition results regardless of this setting.
        """
        with self._errors_lock:
            if enabled and not self._monitoring:
                self._transition_errors.clear()
            self._monitoring = enabled

    # ------------------------------------------------------------------ state callbacks

    def register_state_callback(self, callback: StateCallback) -> int:
        """Call ``callback(new_state)`` whenever the observed state of the node changes.

        Callbacks run on an executor thread of the owning node. The states reported
        are the primary lifecycle states and ``OFFLINE`` (node not in the ROS graph).

        :returns: Handle for :meth:`unregister_state_callback`.
        """
        with self._state_lock:
            handle = self._next_callback_handle
            self._next_callback_handle += 1
            self._callbacks[handle] = callback
            return handle

    def unregister_state_callback(self, handle: int) -> bool:
        """Remove a callback registered with :meth:`register_state_callback`.

        :returns: True if the callback was registered.
        """
        with self._state_lock:
            return self._callbacks.pop(handle, None) is not None

    # ------------------------------------------------------------------ lifecycle

    def get_state(self, cancel_event: Event | None = None) -> GetStateResultDTO:
        """Read the current primary lifecycle state of the node.

        Fails if the node is unreachable or in a transitional state.
        """
        outcome = self._call(self._get_state_client, GetState.Request(), 'get_state', cancel_event)
        if outcome.error:
            return GetStateResultDTO(success=False, error_message=outcome.error)
        current = outcome.response.current_state
        state = _PRIMARY_STATES.get(current.id)
        if state is None:
            return GetStateResultDTO(
                success=False, error_message=f'{self._name} is in the transitional state {current.label!r}'
            )
        return GetStateResultDTO(success=True, state=state)

    def configure(self, cancel_event: Event | None = None) -> NodeManagerResultDTO:
        """``unconfigured`` -> ``inactive``."""
        return self._transition(Transition.TRANSITION_CONFIGURE, cancel_event)

    def activate(self, cancel_event: Event | None = None) -> NodeManagerResultDTO:
        """``inactive`` -> ``active``."""
        return self._transition(Transition.TRANSITION_ACTIVATE, cancel_event)

    def deactivate(self, cancel_event: Event | None = None) -> NodeManagerResultDTO:
        """``active`` -> ``inactive``."""
        return self._transition(Transition.TRANSITION_DEACTIVATE, cancel_event)

    def cleanup(self, cancel_event: Event | None = None) -> NodeManagerResultDTO:
        """``inactive`` -> ``unconfigured``."""
        return self._transition(Transition.TRANSITION_CLEANUP, cancel_event)

    # ------------------------------------------------------------------ parameters

    def get_parameter(self, name: str, cancel_event: Event | None = None) -> GetParameterResultDTO:
        """Read one parameter of the node."""
        result = self.get_parameters([name], cancel_event)
        if not result.success:
            return GetParameterResultDTO(success=False, error_message=result.error_message)
        return GetParameterResultDTO(success=True, value=result.values[name])

    def get_parameters(self, names: list[str], cancel_event: Event | None = None) -> GetParametersResultDTO:
        """Read several parameters of the node in one call."""
        request = GetParameters.Request()
        request.names = list(names)
        outcome = self._call(self._get_parameters_client, request, 'get_parameters', cancel_event)
        if outcome.error:
            return GetParametersResultDTO(success=False, error_message=outcome.error)
        values = list(outcome.response.values)
        if len(values) != len(names):
            error = f'get_parameters of {self._name} returned {len(values)} values for {len(names)} names'
            self._logger.error(error)
            return GetParametersResultDTO(success=False, error_message=error)
        return GetParametersResultDTO(success=True, values=dict(zip(names, values, strict=True)))

    def set_parameter(
        self, name: str, value: ParameterValue, cancel_event: Event | None = None
    ) -> NodeManagerResultDTO:
        """Set one parameter of the node."""
        return self.set_parameters({name: value}, cancel_event)

    def set_parameters(
        self, parameters: dict[str, ParameterValue], cancel_event: Event | None = None
    ) -> NodeManagerResultDTO:
        """Set several parameters of the node in one call; fails if any is rejected."""
        request = SetParameters.Request()
        request.parameters = [Parameter(name=name, value=value) for name, value in parameters.items()]
        outcome = self._call(self._set_parameters_client, request, 'set_parameters', cancel_event)
        if outcome.error:
            return NodeManagerResultDTO(success=False, error_message=outcome.error)
        results = list(outcome.response.results)
        if len(results) != len(parameters):
            error = f'set_parameters of {self._name} returned {len(results)} results for {len(parameters)} parameters'
            self._logger.error(error)
            return NodeManagerResultDTO(success=False, error_message=error)
        rejected = [
            f'{name} ({result.reason})' if result.reason else name
            for name, result in zip(parameters, results, strict=True)
            if not result.successful
        ]
        if rejected:
            error = f'{self._name} rejected parameters: {", ".join(rejected)}'
            self._logger.error(error)
            return NodeManagerResultDTO(success=False, error_message=error)
        return NodeManagerResultDTO(success=True)

    # ------------------------------------------------------------------ internals

    def _transition(self, transition_id: int, cancel_event: Event | None) -> NodeManagerResultDTO:
        label = _TRANSITION_LABELS[transition_id]
        self._logger.info(f'Requesting {label} of {self._name}')
        request = ChangeState.Request()
        request.transition = Transition(id=transition_id, label=label)
        waiter = _TransitionErrorWaiter()
        with self._errors_lock:
            self._error_waiters.add(waiter)
        try:
            return self._run_transition(transition_id, label, request, waiter, cancel_event)
        finally:
            with self._errors_lock:
                self._error_waiters.discard(waiter)

    def _run_transition(
        self,
        transition_id: int,
        label: str,
        request: ChangeState.Request,
        waiter: _TransitionErrorWaiter,
        cancel_event: Event | None,
    ) -> NodeManagerResultDTO:
        outcome = self._call(
            self._change_state_client, request, label, cancel_event, response_timeout_s=self._transition_timeout_s
        )
        if outcome.error == CANCELLED_MESSAGE:
            return NodeManagerResultDTO(success=False, error_message=CANCELLED_MESSAGE)
        if not outcome.error and outcome.response.success:
            return NodeManagerResultDTO(success=True)
        # A lost response does not mean the transition failed: confirm the actual state.
        expected = _EXPECTED_STATE[transition_id]
        state = self.get_state(cancel_event)
        if state.success and state.state == expected:
            self._logger.warning(
                f"No response to '{label}' of {self._name}, but it is already {expected.value}; "
                'treating the transition as successful'
            )
            return NodeManagerResultDTO(success=True)
        if state.error_message == CANCELLED_MESSAGE:
            return NodeManagerResultDTO(success=False, error_message=CANCELLED_MESSAGE)
        error = f"Transition '{label}' of {self._name} failed"
        # Only a node that received the request can have published why it failed.
        published = self._await_transition_error(waiter, cancel_event) if outcome.sent else None
        if published is not None:
            error = f'{error}: {published.message}'
        elif outcome.error:
            error = f'{error}: {outcome.error}'
        self._logger.error(error)
        return NodeManagerResultDTO(success=False, error_message=error)

    def _call(
        self,
        client: Client,
        request: Any,
        operation: str,
        cancel_event: Event | None,
        response_timeout_s: float | None = None,
    ) -> _CallOutcome:
        """Call a service; availability and response are each bounded by a timeout."""
        description = f'{operation} of {self._name}'
        deadline = time.monotonic() + self._service_timeout_s
        while not client.service_is_ready():
            if cancel_event is not None and cancel_event.is_set():
                return _CallOutcome(error=CANCELLED_MESSAGE)
            if time.monotonic() > deadline:
                error = f'Service for {description} is not available'
                self._logger.error(error)
                return _CallOutcome(error=error)
            time.sleep(_POLL_PERIOD_S)

        sent = True
        future = client.call_async(request)
        deadline = time.monotonic() + (self._service_timeout_s if response_timeout_s is None else response_timeout_s)
        while not future.done():
            if cancel_event is not None and cancel_event.is_set():
                client.remove_pending_request(future)
                return _CallOutcome(error=CANCELLED_MESSAGE, sent=sent)
            if time.monotonic() > deadline:
                client.remove_pending_request(future)
                error = f'Service call for {description} timed out'
                self._logger.error(error)
                return _CallOutcome(error=error, sent=sent)
            time.sleep(_POLL_PERIOD_S)
        if future.exception() is not None:
            error = f'Service call for {description} failed: {future.exception()!r}'
            self._logger.error(error)
            return _CallOutcome(error=error, sent=sent)
        return _CallOutcome(response=future.result(), sent=sent)

    def _await_transition_error(
        self, waiter: _TransitionErrorWaiter, cancel_event: Event | None
    ) -> TransitionError | None:
        """Wait up to ``transition_error_wait_s`` for the managed node's transition error.

        The ``change_state`` reply and the topic message travel independently, so the
        error may arrive after the reply.
        """
        deadline = time.monotonic() + self._transition_error_wait_s
        while not waiter.received.is_set():
            if cancel_event is not None and cancel_event.is_set():
                break
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            waiter.received.wait(min(remaining, _POLL_PERIOD_S))
        return waiter.message

    def _on_transition_error(self, msg: TransitionError) -> None:
        if msg.source_node != self._name:
            return
        with self._errors_lock:
            if self._monitoring:
                self._transition_errors.append(msg)
            for waiter in self._error_waiters:
                if waiter.message is None:
                    waiter.message = msg
                    waiter.received.set()

    def _on_transition_event(self, msg: TransitionEvent) -> None:
        state = _PRIMARY_STATES.get(msg.goal_state.id)
        if state is not None:
            self._update_state(state)

    def _check_presence(self) -> None:
        """Detect the node leaving or (re)joining the ROS graph."""
        present = self._get_state_client.service_is_ready()
        with self._state_lock:
            observed = self._observed_state
            query = present and observed in (None, LifecycleStateEnum.OFFLINE) and not self._state_query_pending
            if query:
                self._state_query_pending = True
        if not present:
            self._update_state(LifecycleStateEnum.OFFLINE)
        elif query:
            # Non-blocking: the timer must not stall the executor thread.
            future = self._get_state_client.call_async(GetState.Request())
            future.add_done_callback(self._on_presence_state)

    def _on_presence_state(self, future: Future) -> None:
        with self._state_lock:
            self._state_query_pending = False
        if future.cancelled() or future.exception() is not None:
            return
        state = _PRIMARY_STATES.get(future.result().current_state.id)
        if state is not None:
            self._update_state(state)

    def _update_state(self, state: LifecycleStateEnum) -> None:
        with self._state_lock:
            if state == self._observed_state:
                return
            self._observed_state = state
            callbacks = list(self._callbacks.values())
        self._logger.info(f'{self._name} is {state.value}')
        for callback in callbacks:
            try:
                callback(state)
            except Exception as error:  # noqa: BLE001 - one faulty callback must not starve the others
                self._logger.error(f'State callback for {self._name} failed: {error!r}')
