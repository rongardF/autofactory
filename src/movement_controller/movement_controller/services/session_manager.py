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
"""SessionManager — exclusive, time-limited movement session lock."""

import math
from threading import Lock
from typing import Callable
from uuid import uuid4

from movement_controller.models.session_result_dto import SessionResultDTO

USE_DEFAULT_TIMEOUT = -1.0


class SessionManager:
    """Holds at most one movement session and decides who may send movement goals.

    Expiry is lazy: the deadline is checked on every call, no timer is used.
    Time comes from the injected ``now`` callable (seconds), so the node clock
    (and ``use_sim_time``) can be honoured and tests can use a fake clock.
    """

    def __init__(
        self,
        default_timeout_s: float,
        now: Callable[[], float],
        on_expired: Callable[[str], None] | None = None,
    ) -> None:
        """:param default_timeout_s: Timeout used when a caller requests -1; must be finite and > 0.
        :param now: Callable returning the current time in seconds.
        :param on_expired: Optional callback invoked with the session ID when a session is found expired.
        :raises ValueError: If ``default_timeout_s`` is not finite and greater than 0.
        """
        self._on_expired = on_expired
        if not math.isfinite(default_timeout_s) or default_timeout_s <= 0.0:
            raise ValueError(f'session_timeout_s must be finite and > 0, got {default_timeout_s}')
        self._default_timeout_s = default_timeout_s
        self._now = now
        self._lock = Lock()
        self._session_id: str | None = None
        self._deadline: float = 0.0

    def acquire_or_extend(self, session_id: str, timeout_s: float) -> SessionResultDTO:
        """Acquire a new session (empty ``session_id``) or extend the active one.

        :param session_id: Empty to acquire; the active session ID to extend it.
        :param timeout_s: -1 for the node default, otherwise a finite value > 0.
        :returns: Result with the session ID and effective timeout on success.
        """
        if timeout_s == USE_DEFAULT_TIMEOUT:
            effective = self._default_timeout_s
        elif math.isfinite(timeout_s) and timeout_s > 0.0:
            effective = timeout_s
        else:
            return SessionResultDTO(
                error_message=f'Invalid session timeout {timeout_s}: must be -1 (default) or a finite value > 0')

        with self._lock:
            active = self._active_session_id()
            if session_id == '':
                if active is not None:
                    return SessionResultDTO(error_message='Movement controller is locked by another session')
                self._session_id = str(uuid4())
            elif active is None or session_id != active:
                return SessionResultDTO(
                    error_message='Cannot extend: session ID is unknown or expired; acquire a new session with an empty ID')
            self._deadline = self._now() + effective
            return SessionResultDTO(success=True, session_id=self._session_id or '', session_timeout_s=effective)

    def close(self, session_id: str) -> SessionResultDTO:
        """Release the session; succeeds when none is active, fails for a different active session."""
        with self._lock:
            active = self._active_session_id()
            if active is None:
                return SessionResultDTO(success=True)
            if session_id != active:
                return SessionResultDTO(error_message='Cannot close: session ID does not match the active session')
            self._session_id = None
            return SessionResultDTO(success=True)

    def is_goal_allowed(self, session_id: str) -> bool:
        """Return True if a goal carrying ``session_id`` may run: no active session, or the ID matches."""
        with self._lock:
            active = self._active_session_id()
            return active is None or session_id == active

    def clear(self) -> None:
        """Drop any active session."""
        with self._lock:
            self._session_id = None

    def _active_session_id(self) -> str | None:
        """Return the active session ID, clearing it first if expired. Caller must hold the lock."""
        if self._session_id is not None and self._now() >= self._deadline:
            expired, self._session_id = self._session_id, None
            if self._on_expired is not None:
                self._on_expired(expired)
        return self._session_id
