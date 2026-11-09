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
"""Cancellable, bounded service calls for worker threads."""

import threading
import time
from typing import Any

from rclpy.client import Client

from station_workplace.exceptions.process_cancelled_error import ProcessCancelledError
from station_workplace.exceptions.workplace_error import WorkplaceError

_POLL_PERIOD_SEC = 0.01
_WAIT_SLICE_SEC = 0.1


def call_service(
    client: Client,
    request: Any,
    timeout_sec: float,
    cancel: threading.Event,
    error_cls: type[WorkplaceError],
    description: str,
) -> Any:
    """Call a service and wait for its response, honouring a cancel token.

    The service availability wait and the response wait are each bounded by
    ``timeout_sec``. The function polls the future instead of spinning, so it
    must run on a worker thread (or a reentrant callback group thread) while the
    node's ``MultiThreadedExecutor`` delivers the response on another thread.

    :param client: Service client to call.
    :param request: Request message.
    :param timeout_sec: Timeout for service availability and for the response.
    :param cancel: Cancel token; when set the call is abandoned.
    :param error_cls: Exception raised on unavailability, timeout or no result.
    :param description: Human-readable operation name used in error messages.
    :returns: The service response.
    :raises ProcessCancelledError: If ``cancel`` is set before completion.
    :raises WorkplaceError: ``error_cls`` on unavailability or timeout.
    """
    deadline = time.monotonic() + timeout_sec
    while True:
        if cancel.is_set():
            raise ProcessCancelledError(f'{description}: cancelled')
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise error_cls(
                f"{description}: service '{client.srv_name}' not available "
                f'within {timeout_sec} s')
        if client.wait_for_service(timeout_sec=min(_WAIT_SLICE_SEC, remaining)):
            break

    future = client.call_async(request)
    deadline = time.monotonic() + timeout_sec
    try:
        while not future.done():
            if cancel.wait(_POLL_PERIOD_SEC):
                raise ProcessCancelledError(f'{description}: cancelled')
            if time.monotonic() > deadline:
                raise error_cls(
                    f"{description}: no response from '{client.srv_name}' "
                    f'within {timeout_sec} s')
    except WorkplaceError:
        try:
            client.remove_pending_request(future)
        except Exception:  # noqa: BLE001 - client may already be destroyed
            pass
        future.cancel()
        raise

    response = future.result()
    if response is None:
        raise error_cls(f"{description}: '{client.srv_name}' returned no response")
    return response
