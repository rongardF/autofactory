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
"""Cancellable waiting and asynchronous action execution without blocking calls."""

import threading
import time

from rclpy.action import ActionClient

from localization.exceptions import LocalizationCancelledError, LocalizationError

_POLL_S = 0.05
_CANCEL_ACK_TIMEOUT_S = 2.0


def wait_cancellable(duration_s: float, cancel_event: threading.Event) -> bool:
    """Wait ``duration_s`` seconds; return False if ``cancel_event`` was set meanwhile."""
    return not cancel_event.wait(duration_s)


def _wait(event: threading.Event, deadline: float, cancel_event: threading.Event) -> str:
    """Wait for ``event``; return 'done', 'cancelled' or 'timeout'."""
    while True:
        if event.is_set():
            return 'done'
        if cancel_event.is_set():
            return 'cancelled'
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return 'timeout'
        event.wait(min(remaining, _POLL_S))


def execute_goal(
    client: ActionClient,
    goal,
    *,
    timeout_s: float,
    cancel_event: threading.Event,
    description: str,
    error_cls: type[LocalizationError],
):
    """Send ``goal`` and wait for its result using future callbacks.

    The goal is cancelled on the server if ``cancel_event`` is set or ``timeout_s``
    passes before the result arrives.

    :returns: The action result message.
    :raises LocalizationCancelledError: If ``cancel_event`` was set.
    :raises error_cls: If the goal was rejected or timed out.
    """
    accepted = threading.Event()
    result_ready = threading.Event()
    state: dict = {}

    def on_goal_response(future) -> None:
        try:
            state['handle'] = future.result()
        except Exception as exc:  # noqa: BLE001 - reported through the raised error
            state['error'] = exc
        accepted.set()

    client.send_goal_async(goal).add_done_callback(on_goal_response)
    deadline = time.monotonic() + timeout_s

    outcome = _wait(accepted, deadline, cancel_event)
    if outcome == 'cancelled':
        # The server may still accept the goal; give it a moment, then cancel it.
        accepted.wait(_CANCEL_ACK_TIMEOUT_S)
        handle = state.get('handle')
        if handle is not None and handle.accepted:
            handle.cancel_goal_async()
        raise LocalizationCancelledError('cancelled')
    if outcome == 'timeout':
        raise error_cls(f'{description}: goal was not answered within {timeout_s} s')
    handle = state.get('handle')
    if handle is None:
        raise error_cls(f"{description}: could not send the goal: {state.get('error')}")
    if not handle.accepted:
        raise error_cls(f'{description}: goal was rejected')

    result_future = handle.get_result_async()
    result_future.add_done_callback(lambda _future: result_ready.set())

    outcome = _wait(result_ready, deadline, cancel_event)
    if outcome != 'done':
        cancel_done = threading.Event()
        handle.cancel_goal_async().add_done_callback(lambda _future: cancel_done.set())
        cancel_done.wait(_CANCEL_ACK_TIMEOUT_S)
        if outcome == 'cancelled':
            raise LocalizationCancelledError('cancelled')
        raise error_cls(f'{description}: no result within {timeout_s} s')
    return result_future.result().result
