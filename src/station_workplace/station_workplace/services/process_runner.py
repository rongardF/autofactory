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
"""Serialised execution of injection/ejection processes on worker threads."""

import threading
from collections.abc import Callable, Iterator
from contextlib import contextmanager

from rclpy.impl.rcutils_logger import RcutilsLogger

from station_workplace.enums.process_type_enum import ProcessTypeEnum
from station_workplace.exceptions.process_cancelled_error import ProcessCancelledError

ProcessFunction = Callable[[threading.Event, int], None]


class _Process:
    """A submitted process and its worker thread."""

    def __init__(self, kind: ProcessTypeEnum, thread: threading.Thread, cancel: threading.Event):
        self.kind = kind
        self.thread = thread
        self.cancel = cancel


class ProcessRunner:
    """Run processes one at a time, each on its own worker thread.

    * A new process waits for the previous one to finish (strict serialisation).
    * Submitting an ejection cancels a running or queued injection.
    * :meth:`disable` invalidates the current epoch and cancels the running
      process; once it returns, :meth:`side_effect_guard` prevents any stale
      worker from modifying the scene or publishing.
    """

    def __init__(self, logger: RcutilsLogger) -> None:
        self._logger = logger
        self._lock = threading.Lock()
        # Held for the duration of each side effect; disable() takes it to fence workers.
        self._side_effect_lock = threading.Lock()
        self._epoch = 0
        self._enabled = False
        self._current: _Process | None = None

    def enable(self) -> None:
        """Accept submissions in a new epoch."""
        with self._side_effect_lock, self._lock:
            self._epoch += 1
            self._enabled = True

    def disable(self) -> None:
        """Reject submissions, cancel the running process and invalidate its epoch.

        Does not join the worker; at most it waits for one in-flight side effect.
        """
        with self._side_effect_lock, self._lock:
            self._epoch += 1
            self._enabled = False
            if self._current is not None:
                self._current.cancel.set()

    def submit(self, kind: ProcessTypeEnum, fn: ProcessFunction) -> None:
        """Queue ``fn(cancel, epoch)`` after the current process.

        Ignored while disabled.
        """
        with self._lock:
            if not self._enabled:
                return
            previous = self._current
            if (
                kind == ProcessTypeEnum.EJECTION
                and previous is not None
                and previous.kind == ProcessTypeEnum.INJECTION
            ):
                previous.cancel.set()
            cancel = threading.Event()
            epoch = self._epoch
            thread = threading.Thread(
                target=self._run,
                args=(previous, kind, fn, cancel, epoch),
                name=f'workplace_{kind.value}',
                daemon=True,
            )
            process = _Process(kind, thread, cancel)
            self._current = process
            thread.start()

    def is_current(self, epoch: int) -> bool:
        """Return whether ``epoch`` is still the active epoch."""
        with self._lock:
            return self._enabled and epoch == self._epoch

    @contextmanager
    def side_effect_guard(self, epoch: int) -> Iterator[None]:
        """Run a side effect only if ``epoch`` is still current.

        :raises ProcessCancelledError: If the epoch is stale.
        """
        with self._side_effect_lock:
            if not self.is_current(epoch):
                raise ProcessCancelledError('process abandoned by a lifecycle transition')
            yield

    def _run(
        self,
        previous: _Process | None,
        kind: ProcessTypeEnum,
        fn: ProcessFunction,
        cancel: threading.Event,
        epoch: int,
    ) -> None:
        """Worker body: wait for the previous process, then run ``fn``."""
        if previous is not None:
            previous.thread.join()
        try:
            if cancel.is_set() or not self.is_current(epoch):
                self._logger.info(f'Skipping cancelled {kind.value}')
                return
            self._logger.info(f'Starting {kind.value}')
            fn(cancel, epoch)
        except ProcessCancelledError as exc:
            self._logger.info(f'{kind.value.capitalize()} cancelled: {exc}')
        except Exception as exc:  # noqa: BLE001 - last-resort guard of the worker thread
            self._logger.error(f'Unhandled error in {kind.value}: {exc!r}')
        finally:
            with self._lock:
                if self._current is not None and self._current.thread is threading.current_thread():
                    self._current = None
