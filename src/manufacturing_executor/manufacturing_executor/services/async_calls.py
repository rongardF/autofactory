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
"""Abort-aware service calls and action goals using future callbacks only.

Nothing here spins an executor or blocks on ``send_goal``: every wait polls an
:class:`threading.Event` that is set from a future callback, so the calls can run on
an executor thread while another thread serves the response.
"""

import threading
import time
from collections.abc import Callable
from typing import Any

from action_msgs.msg import GoalStatus
from pydantic import BaseModel, ConfigDict
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.client import Client
from rclpy.task import Future

from manufacturing_executor.exceptions import SystemFailureError
from manufacturing_executor.services.abort_context import AbortContext

_POLL_S = 0.05
_CANCEL_ACK_TIMEOUT_S = 2.0


def wait_event(event: threading.Event, timeout_s: float | None, ctx: AbortContext) -> bool:
    """Wait for ``event``; ``None`` waits without a timeout.

    :returns: True if the event was set, False on timeout.
    :raises ManufacturingError: The abort error of ``ctx``.
    """
    deadline = None if timeout_s is None else time.monotonic() + timeout_s
    while True:
        if event.is_set():
            return True
        ctx.raise_if_aborted()
        if deadline is None:
            event.wait(_POLL_S)
            continue
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return False
        event.wait(min(_POLL_S, remaining))


def sleep_abortable(duration_s: float, ctx: AbortContext) -> None:
    """Wall-clock sleep that ends early with the abort error of ``ctx``."""
    wait_event(threading.Event(), duration_s, ctx)


def _wait_until_ready(
    is_ready: Callable[[], bool], timeout_s: float, ctx: AbortContext, description: str
) -> None:
    deadline = time.monotonic() + timeout_s
    while not is_ready():
        ctx.raise_if_aborted()
        if time.monotonic() > deadline:
            raise SystemFailureError(f'{description}: not available within {timeout_s} s')
        time.sleep(0.02)


class ServiceCall:
    """A service call that was sent and whose response is awaited later."""

    def __init__(self, client: Client, future: Future, description: str) -> None:
        self._client = client
        self._future = future
        self._description = description
        self._done = threading.Event()
        future.add_done_callback(lambda _future: self._done.set())

    @classmethod
    def start(
        cls, client: Client, request: Any, *, availability_timeout_s: float, ctx: AbortContext, description: str
    ) -> 'ServiceCall':
        """Wait for the service, then send the request without waiting for the response."""
        _wait_until_ready(client.service_is_ready, availability_timeout_s, ctx, f'{description} (service)')
        return cls(client, client.call_async(request), description)

    def wait(self, timeout_s: float, ctx: AbortContext) -> Any:
        """Return the response.

        :raises SystemFailureError: On timeout or when the call failed.
        """
        if not wait_event(self._done, timeout_s, ctx):
            self._client.remove_pending_request(self._future)
            raise SystemFailureError(f'{self._description}: no response within {timeout_s} s')
        try:
            response = self._future.result()
        except Exception as error:  # noqa: BLE001
            raise SystemFailureError(f'{self._description}: call failed: {error}') from error
        if response is None:
            raise SystemFailureError(f'{self._description}: call returned no response')
        return response


def call_service(
    client: Client, request: Any, *, timeout_s: float, ctx: AbortContext, description: str
) -> Any:
    """Call a service; availability and response are each bounded by ``timeout_s``."""
    call = ServiceCall.start(
        client, request, availability_timeout_s=timeout_s, ctx=ctx, description=description
    )
    return call.wait(timeout_s, ctx)


class ActionOutcome(BaseModel):
    """Final status and result of an action goal."""

    model_config = ConfigDict(frozen=True, arbitrary_types_allowed=True)

    status: int
    result: Any

    def error_text(self) -> str:
        """Error text of the result (``error_message`` or ``message``)."""
        return (
            getattr(self.result, 'error_message', None)
            or getattr(self.result, 'message', None)
            or f'goal status {self.status}'
        )

    def ensure_success(self, description: str) -> Any:
        """Return the result if the goal succeeded.

        :raises SystemFailureError: If the goal did not succeed or reports ``success=False``.
        """
        if self.status != GoalStatus.STATUS_SUCCEEDED or not self.result.success:
            raise SystemFailureError(f'{description} failed: {self.error_text()}')
        return self.result


class ChildGoal:
    """An accepted action goal that is cancelled when the context is aborted."""

    def __init__(self, handle: ClientGoalHandle, ctx: AbortContext, description: str) -> None:
        self._handle = handle
        self._ctx = ctx
        self._description = description
        self.done = threading.Event()
        self._result_future = handle.get_result_async()
        self._result_future.add_done_callback(lambda _future: self.done.set())
        self._unregister = ctx.register(self.cancel)

    @classmethod
    def start(
        cls,
        client: ActionClient,
        goal: Any,
        *,
        ctx: AbortContext,
        description: str,
        accept_timeout_s: float,
        feedback_callback: Callable[[Any], None] | None = None,
    ) -> 'ChildGoal':
        """Send ``goal`` and wait until it is accepted.

        :raises SystemFailureError: If the server is unavailable or rejects the goal.
        """
        _wait_until_ready(client.server_is_ready, accept_timeout_s, ctx, f'{description} (action server)')
        send_future = client.send_goal_async(goal, feedback_callback=feedback_callback)
        answered = threading.Event()
        send_future.add_done_callback(lambda _future: answered.set())
        try:
            if not wait_event(answered, accept_timeout_s, ctx):
                raise SystemFailureError(f'{description}: goal was not answered within {accept_timeout_s} s')
        except BaseException:
            # The server may still accept the goal later; make sure it does not keep running.
            send_future.add_done_callback(cls._cancel_late)
            raise
        handle = send_future.result()
        if not handle.accepted:
            raise SystemFailureError(f'{description}: goal was rejected')
        return cls(handle, ctx, description)

    @staticmethod
    def _cancel_late(future: Future) -> None:
        try:
            handle = future.result()
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
        except Exception:  # noqa: BLE001 - best effort
            pass

    def add_done_callback(self, callback: Callable[[], None]) -> None:
        """Call ``callback`` (on an executor thread) once the goal has a result; runs at once if it has one."""
        self._result_future.add_done_callback(lambda _future: callback())

    def release(self) -> None:
        """Stop cancelling the goal on abort; for goals that are not awaited with :meth:`wait`."""
        self._unregister()

    def cancel(self, wait: bool = False) -> None:
        """Request cancellation of the goal; optionally wait for the acknowledgement."""
        if self.done.is_set():
            return
        try:
            future = self._handle.cancel_goal_async()
        except Exception:  # noqa: BLE001 - best effort
            return
        if wait:
            acknowledged = threading.Event()
            future.add_done_callback(lambda _future: acknowledged.set())
            acknowledged.wait(_CANCEL_ACK_TIMEOUT_S)

    def wait(self, timeout_s: float | None) -> ActionOutcome:
        """Wait for the result; ``None`` waits without a timeout.

        :raises SystemFailureError: On timeout (the goal is cancelled).
        """
        try:
            if not wait_event(self.done, timeout_s, self._ctx):
                self.cancel(wait=True)
                raise SystemFailureError(f'{self._description}: no result within {timeout_s} s')
            response = self._result_future.result()
            return ActionOutcome(status=response.status, result=response.result)
        finally:
            self._unregister()


def run_action(
    client: ActionClient,
    goal: Any,
    *,
    ctx: AbortContext,
    description: str,
    accept_timeout_s: float,
    result_timeout_s: float | None,
    feedback_callback: Callable[[Any], None] | None = None,
) -> ActionOutcome:
    """Send a goal and wait for its result."""
    child = ChildGoal.start(
        client,
        goal,
        ctx=ctx,
        description=description,
        accept_timeout_s=accept_timeout_s,
        feedback_callback=feedback_callback,
    )
    return child.wait(result_timeout_s)
