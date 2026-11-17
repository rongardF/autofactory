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
"""MovementSession — client-side handle of a movement controller session (for other packages)."""

import math
from collections.abc import Callable

from movement_controller.srv import CloseSession, InSession

DEFAULT_SESSION_TIMEOUT_S = 60.0
DEFAULT_SESSION_EXTEND_THRESHOLD_S = 30.0


class MovementSession:
    """Acquires, keeps alive and releases one movement controller session as a client.

    Counterpart of the server-side :class:`SessionManager`. The service transport is
    injected, so callers can use their own (blocking or callback based) service calls.

    The session is opened by the first :meth:`ensure_valid` call. The caller cannot
    query the server, so it tracks its own deadline (time of the last ``in_session``
    response + ``timeout_s``) and restarts the timeout before a movement when less
    than ``extend_threshold_s`` remain.
    """

    def __init__(
        self,
        in_session_call: Callable[[InSession.Request], InSession.Response | None],
        close_session_call: Callable[[CloseSession.Request], CloseSession.Response | None],
        now_s: Callable[[], float],
        *,
        timeout_s: float = DEFAULT_SESSION_TIMEOUT_S,
        extend_threshold_s: float = DEFAULT_SESSION_EXTEND_THRESHOLD_S,
    ) -> None:
        """:param in_session_call: Calls ``in_session``; returns ``None`` if the service is unavailable.
        :param close_session_call: Calls ``close_session``; returns ``None`` if the service is unavailable.
        :param now_s: Current time in seconds, from the node clock (matches the server's clock).
        :param timeout_s: Session timeout requested on acquire and on every extension; must be finite and > 0.
        :param extend_threshold_s: Extend before a movement when less than this many seconds remain; must be >= 0.
        :raises ValueError: If ``timeout_s`` or ``extend_threshold_s`` is out of range.
        """
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError(f'timeout_s must be finite and > 0, got {timeout_s}')
        if not math.isfinite(extend_threshold_s) or extend_threshold_s < 0.0:
            raise ValueError(f'extend_threshold_s must be finite and >= 0, got {extend_threshold_s}')
        self._in_session_call = in_session_call
        self._close_session_call = close_session_call
        self._now_s = now_s
        self._timeout_s = timeout_s
        self._extend_threshold_s = extend_threshold_s
        self._session_id = ''
        self._deadline_s = 0.0

    @property
    def session_id(self) -> str:
        """Session ID to put in every goal; empty while no session is open."""
        return self._session_id

    def ensure_valid(self) -> str:
        """Open the session, or extend it when its timeout is close to expiry.

        :returns: Empty string on success, otherwise the error message.
        """
        if self._session_id and self._deadline_s - self._now_s() >= self._extend_threshold_s:
            return ''
        request = InSession.Request()
        request.session_id = self._session_id
        request.session_timeout_s = self._timeout_s
        response = self._in_session_call(request)
        if response is None:
            return 'in_session service is not available or did not respond'
        if not response.success:
            return f'in_session rejected: {response.error_message}'
        self._session_id = response.session_id
        self._deadline_s = self._now_s() + self._timeout_s
        return ''

    def close(self) -> str:
        """Release the session if one is open; safe to call repeatedly.

        :returns: Empty string on success (or nothing to close), otherwise the error message.
        """
        if not self._session_id:
            return ''
        request = CloseSession.Request()
        request.session_id = self._session_id
        self._session_id = ''
        response = self._close_session_call(request)
        if response is None:
            return 'close_session service is not available or did not respond'
        if not response.success:
            return f'close_session rejected: {response.error_message}'
        return ''
