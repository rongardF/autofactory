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
"""Publishes the model list from a dedicated thread, only when it changed."""

from threading import Event, Thread
from typing import Callable

from rclpy.impl.rcutils_logger import RcutilsLogger

from world_manager.models import ModelInfoDTO


class ModelListPublisher:
    """Owns the thread that is the only place where the model list is built and published.

    Gz handlers and service callbacks only set the ``dirty`` event; this thread takes a snapshot of the
    committed list, compares it with the last published one and publishes only if it differs.
    """

    def __init__(
        self,
        dirty: Event,
        snapshot: Callable[[], list[ModelInfoDTO]],
        publish: Callable[[list[ModelInfoDTO]], None],
        logger: RcutilsLogger,
    ) -> None:
        """``publish`` is called on the publisher thread only."""
        self._dirty = dirty
        self._snapshot = snapshot
        self._publish = publish
        self._logger = logger
        self._stop = Event()
        self._thread: Thread | None = None
        self._last: list[ModelInfoDTO] | None = None

    @property
    def thread(self) -> Thread | None:
        """The publisher thread, None when not started."""
        return self._thread

    def start(self) -> None:
        """Start the thread (idempotent)."""
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = Thread(target=self._run, name='model_list_publisher', daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop and join the thread."""
        if self._thread is None:
            return
        self._stop.set()
        self._dirty.set()
        self._thread.join(timeout=5.0)
        self._thread = None

    def republish(self) -> None:
        """Forget the last published list so the current one is published once, even if unchanged."""
        self._last = None
        self._dirty.set()

    def _run(self) -> None:
        while True:
            self._dirty.wait()
            if self._stop.is_set():
                return
            self._dirty.clear()
            try:
                current = self._snapshot()
                if current != self._last:
                    self._publish(current)
                    self._last = current
            except Exception as e:
                self._logger.error(f'publishing the model list failed: {e}')
