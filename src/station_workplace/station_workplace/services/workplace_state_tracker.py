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

from station_workplace.models.workplace_state_dto import WorkplaceStateDTO


class StateWaiter:
    """Wait handle for the workplace settling with an expected ``in_place`` value."""

    def __init__(self, in_place: bool) -> None:
        self.in_place = in_place
        self._event = threading.Event()
        self._state: WorkplaceStateDTO | None = None

    @property
    def is_settled(self) -> bool:
        """Return whether the expected state has been reached."""
        return self._event.is_set()

    @property
    def state(self) -> WorkplaceStateDTO | None:
        """Return the settled state, or None if not settled yet."""
        return self._state

    def wait(self, timeout_sec: float) -> bool:
        """Wait up to ``timeout_sec`` for the expected state; return whether it was reached."""
        return self._event.wait(timeout_sec)

    def _settle(self, state: WorkplaceStateDTO) -> None:
        self._state = state
        self._event.set()


class WorkplaceStateTracker:
    """Track the settled workplace state and wake waiters when a process finishes.

    :meth:`settle` is called at the end of every injection/ejection process,
    also when the state equals the previously published one (and therefore no
    message is published on ``~/workplace``). Like the last published state,
    the tracked state survives deactivate/activate cycles and is only
    forgotten by :meth:`reset`.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state: WorkplaceStateDTO | None = None
        self._waiters: set[StateWaiter] = set()

    @property
    def state(self) -> WorkplaceStateDTO | None:
        """Return the last settled state, or None if unknown."""
        with self._lock:
            return self._state

    def settle(self, state: WorkplaceStateDTO) -> None:
        """Record ``state`` and wake all waiters expecting its ``in_place`` value."""
        with self._lock:
            self._state = state
            matched = [w for w in self._waiters if w.in_place == state.in_place]
            self._waiters.difference_update(matched)
        for waiter in matched:
            waiter._settle(state)

    def wait_for(self, in_place: bool) -> StateWaiter:
        """Return a waiter for the next settled state with ``in_place``.

        The waiter is already settled if the current state matches; checking
        and registering are atomic so no settlement is missed.
        """
        waiter = StateWaiter(in_place)
        with self._lock:
            if self._state is not None and self._state.in_place == in_place:
                waiter._settle(self._state)
            else:
                self._waiters.add(waiter)
        return waiter

    def discard(self, waiter: StateWaiter) -> None:
        """Stop tracking ``waiter`` (no-op if already settled)."""
        with self._lock:
            self._waiters.discard(waiter)

    def reset(self) -> None:
        """Forget the tracked state."""
        with self._lock:
            self._state = None
