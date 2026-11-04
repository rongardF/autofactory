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
"""Abstract internal reader service shared by all RFID scanner drivers."""

import threading
from abc import ABC, abstractmethod
from typing import Callable, Final

from rclpy.lifecycle import LifecycleNode

from rfid_scanner.exceptions.tag_access_error import TagAccessError
from rfid_scanner.models.node_config_dto import NodeConfigDTO
from rfid_scanner.models.tag_info_dto import TagInfoDTO


class RfidReader(ABC):
    """Hardware or simulated reader driven by an RFID scanner node.

    The request validation is implemented once here so every driver behaves
    identically; drivers only implement the actual transfer.
    """

    MAX_TRANSFER_BYTES: Final[int] = 248

    def __init__(
        self,
        node: LifecycleNode,
        config: NodeConfigDTO,
        on_detected_changed: Callable[[bool], None],
        on_fatal_error: Callable[[str], None],
    ) -> None:
        """Store collaborators; no resources are acquired here."""
        self._node = node
        self._config = config
        self._logger = node.get_logger()
        self._on_detected_changed = on_detected_changed
        self._on_fatal_error = on_fatal_error
        self._state_lock = threading.RLock()
        self._tag_info: TagInfoDTO | None = None
        self._fatal_reported = False
        # Fatal reports are only forwarded while the reader is fully running;
        # failures during initialize() fail the activation instead.
        self._fatal_reporting_enabled = False

    @abstractmethod
    def initialize(self) -> None:
        """Acquire resources and start the reader; raise on failure."""

    @abstractmethod
    def deinitialize(self) -> None:
        """Release all resources. Idempotent, tolerates partial init, never raises."""

    @property
    def tag_info(self) -> TagInfoDTO | None:
        """Return the detected tag, or ``None`` if no usable tag is detected."""
        with self._state_lock:
            return self._tag_info

    @property
    def is_tag_detected(self) -> bool:
        """Return True if a tag is detected."""
        return self.tag_info is not None

    def read(self, start_address: int, length: int) -> bytes:
        """Validate the request and read ``length`` bytes from the tag."""
        tag = self._validate(start_address, length)
        return self._read_impl(tag, start_address, length)

    def write(self, start_address: int, data: bytes) -> None:
        """Validate the request and write ``data`` to the tag."""
        tag = self._validate(start_address, len(data))
        self._write_impl(tag, start_address, data)

    @abstractmethod
    def _read_impl(self, tag: TagInfoDTO, start_address: int, length: int) -> bytes:
        """Perform a validated read."""

    @abstractmethod
    def _write_impl(self, tag: TagInfoDTO, start_address: int, data: bytes) -> None:
        """Perform a validated write."""

    def _validate(self, start_address: int, size: int) -> TagInfoDTO:
        """Check a request against the common rules and return the tag."""
        if not self.is_tag_detected:
            raise TagAccessError('no tag detected')
        if size <= 0:
            raise TagAccessError('requested size must be > 0')
        if size > self.MAX_TRANSFER_BYTES:
            raise TagAccessError(
                f'requested size {size} exceeds maximum of '
                f'{self.MAX_TRANSFER_BYTES} bytes')
        tag = self.tag_info
        if tag is None:
            raise TagAccessError('tag data not available yet')
        if start_address + size > tag.memory_size:
            raise TagAccessError(
                f'out of bounds: start {start_address} + size {size} > '
                f'tag memory size {tag.memory_size}')
        return tag

    def _set_detected(self, info: TagInfoDTO | None) -> None:
        """Update the detected tag and notify on a detection change."""
        with self._state_lock:
            was_detected = self.is_tag_detected
            self._tag_info = info
            now_detected = self.is_tag_detected
            # Notify under the lock so concurrent changes are reported in order.
            if was_detected != now_detected:
                self._on_detected_changed(now_detected)

    def _report_fatal(self, reason: str) -> None:
        """Report a fatal error once while the reader is running."""
        with self._state_lock:
            if self._fatal_reported or not self._fatal_reporting_enabled:
                return
            self._fatal_reported = True
        self._logger.error(f'Fatal reader error: {reason}')
        self._on_fatal_error(reason)
