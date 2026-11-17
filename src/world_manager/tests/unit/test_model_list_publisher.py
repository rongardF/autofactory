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
"""Unit tests for ModelListPublisher."""

from threading import Event, current_thread
from unittest.mock import MagicMock

from rclpy.impl.rcutils_logger import RcutilsLogger
from world_manager.models import ModelInfoDTO
from world_manager.services.model_list_publisher import ModelListPublisher


class _Harness:
    def __init__(self) -> None:
        self.dirty = Event()
        self.models: list[ModelInfoDTO] = []
        self.published: list[list[ModelInfoDTO]] = []
        self.threads: set[str] = set()
        self.published_event = Event()
        self.publisher = ModelListPublisher(
            self.dirty, lambda: list(self.models), self._publish, MagicMock(spec=RcutilsLogger))

    def _publish(self, models: list[ModelInfoDTO]) -> None:
        self.threads.add(current_thread().name)
        self.published.append(models)
        self.published_event.set()

    def wait_published(self, count: int, timeout: float = 2.0) -> bool:
        for _ in range(int(timeout / 0.01)):
            if len(self.published) >= count:
                return True
            self.published_event.wait(0.01)
            self.published_event.clear()
        return len(self.published) >= count

    def settle(self) -> None:
        """Wait until the publisher has consumed the dirty flag."""
        for _ in range(200):
            if not self.dirty.is_set():
                break
            self.published_event.wait(0.005)
        self.published_event.wait(0.05)


def test_list_is_published_only_from_the_publisher_thread():
    """Building and publishing the list never happens on the caller's thread."""
    h = _Harness()
    h.publisher.start()
    try:
        h.dirty.set()
        assert h.wait_published(1)
        assert h.threads == {'model_list_publisher'}
    finally:
        h.publisher.stop()


def test_unchanged_list_is_not_published_again():
    """Marking dirty without a change in the snapshot publishes nothing."""
    h = _Harness()
    h.publisher.start()
    try:
        h.dirty.set()
        assert h.wait_published(1)
        h.dirty.set()
        h.settle()
        assert len(h.published) == 1
    finally:
        h.publisher.stop()


def test_changed_list_is_published():
    """A different snapshot is published."""
    h = _Harness()
    h.publisher.start()
    try:
        h.dirty.set()
        assert h.wait_published(1)
        h.models = [ModelInfoDTO(model_id='a')]
        h.dirty.set()
        assert h.wait_published(2)
        assert h.published[-1][0].model_id == 'a'
    finally:
        h.publisher.stop()


def test_republish_publishes_the_current_list_once_even_if_unchanged():
    """After activation the current list goes out again exactly once."""
    h = _Harness()
    h.publisher.start()
    try:
        h.dirty.set()
        assert h.wait_published(1)
        h.publisher.republish()
        assert h.wait_published(2)
        h.settle()
        assert len(h.published) == 2
    finally:
        h.publisher.stop()


def test_stop_joins_the_thread():
    """Stopping ends the thread."""
    h = _Harness()
    h.publisher.start()
    thread = h.publisher.thread
    h.publisher.stop()
    assert not thread.is_alive()
    assert h.publisher.thread is None
