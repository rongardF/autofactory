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
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND CONTRIBUTORS "AS IS"
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
"""Bounded ROS service calls suitable for reentrant executor callbacks."""

from threading import Event
from time import monotonic
from typing import Any, TypeVar

from rclpy.client import Client

from world_manager.exceptions import WorldManagerError

_WAIT_SLICE_SEC = 0.1
_POLL_PERIOD_SEC = 0.01
_Response = TypeVar('_Response')


def call_service(
    client: Client,
    request: Any,
    timeout_sec: float,
    cancel: Event | None = None,
) -> _Response:
    """Wait for a service and its asynchronous response within one total timeout.

    Cancellation only abandons the caller's wait; the remote manager may still complete its
    operation, so callers should re-read state if the outcome matters.
    """
    deadline = monotonic() + timeout_sec
    while True:
        if cancel is not None and cancel.is_set():
            raise WorldManagerError(f"service call to '{client.srv_name}' cancelled")
        remaining = deadline - monotonic()
        if remaining <= 0:
            raise WorldManagerError(
                f"service '{client.srv_name}' not available within {timeout_sec} s")
        if client.wait_for_service(timeout_sec=min(_WAIT_SLICE_SEC, remaining)):
            break

    future = client.call_async(request)
    deadline = monotonic() + timeout_sec
    while not future.done():
        if cancel is not None and cancel.is_set():
            client.remove_pending_request(future)
            future.cancel()
            raise WorldManagerError(
                f"service call to '{client.srv_name}' cancelled; remote outcome may be unknown")
        remaining = deadline - monotonic()
        if remaining <= 0:
            client.remove_pending_request(future)
            future.cancel()
            raise WorldManagerError(
                f"service '{client.srv_name}' did not respond within {timeout_sec} s; "
                'remote outcome may be unknown')
        if cancel is None:
            Event().wait(min(_POLL_PERIOD_SEC, remaining))
        else:
            cancel.wait(min(_POLL_PERIOD_SEC, remaining))

    try:
        response = future.result()
    except Exception as e:
        raise WorldManagerError(f"service '{client.srv_name}' failed: {e}") from e
    if response is None:
        raise WorldManagerError(f"service '{client.srv_name}' returned no response")
    return response
