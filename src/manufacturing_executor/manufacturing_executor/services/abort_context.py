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
"""Abort handling of one manufacturing goal."""

import threading
from collections.abc import Callable

from manufacturing_executor.exceptions import ManufacturingError


class AbortContext:
    """Shared by every blocking step of a goal; the first abort wins.

    Registered callbacks (cancelling child goals) run once, on the aborting thread.
    A detached context is never aborted; cleanup code uses it so that it still runs
    after the goal was aborted.
    """

    def __init__(self, detached: bool = False) -> None:
        """:param detached: If True, :meth:`abort` is ignored."""
        self.event = threading.Event()
        self._detached = detached
        self._lock = threading.Lock()
        self._error: ManufacturingError | None = None
        self._callbacks: dict[int, Callable[[], None]] = {}
        self._next_id = 0

    @classmethod
    def detached(cls) -> 'AbortContext':
        """A context that is never aborted."""
        return cls(detached=True)

    @property
    def error(self) -> ManufacturingError | None:
        """The error the context was aborted with, if any."""
        with self._lock:
            return self._error

    def abort(self, error: ManufacturingError) -> None:
        """Abort with ``error``; only the first call has an effect."""
        if self._detached:
            return
        with self._lock:
            if self._error is not None:
                return
            self._error = error
            callbacks = list(self._callbacks.values())
            self._callbacks.clear()
        self.event.set()
        for callback in callbacks:
            try:
                callback()
            except Exception:  # noqa: BLE001 - one failing cancel must not skip the others
                pass

    def register(self, callback: Callable[[], None]) -> Callable[[], None]:
        """Run ``callback`` on abort; returns the function that unregisters it.

        If the context is already aborted the callback runs immediately.
        """
        with self._lock:
            if self._error is None or self._detached:
                callback_id = self._next_id
                self._next_id += 1
                self._callbacks[callback_id] = callback
                return lambda: self._unregister(callback_id)
        callback()
        return lambda: None

    def raise_if_aborted(self) -> None:
        """Raise the abort error, if the context was aborted."""
        error = self.error
        if error is not None:
            raise error

    def _unregister(self, callback_id: int) -> None:
        with self._lock:
            self._callbacks.pop(callback_id, None)
