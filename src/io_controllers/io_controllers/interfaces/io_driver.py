# Copyright (c) 2026, Movement Controller Contributors
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
"""Abstract driver contract the GPIO controller node depends on.

A concrete driver translates between the normalized, device-agnostic model
(io_names, directions, domains, values) and the specifics of one device family.
The node owns the ROS interface surface and delegates every hardware concern to
a :class:`IODriver` instance, so adding hardware means writing one subclass
without touching the public topic/service/action contract.
"""

from abc import ABC, abstractmethod
from collections.abc import Callable

from io_controllers.models.io_config_dto import IOControllerConfigDTO
from io_controllers.models.io_signal_dto import (
    AnalogSignalDTO,
    DigitalSignalDTO,
    IOStatesSnapshot,
)

# Invoked by the driver on a genuine cached-value change, with the io_name and
# the new immutable signal snapshot. MUST be non-blocking (enqueue-and-return).
ChangeCallback = Callable[[str, DigitalSignalDTO | AnalogSignalDTO], None]


class AbstractIODriver(ABC):
    """Contract between the device-agnostic node and a concrete device backend."""

    @abstractmethod
    def configure(self, config: IOControllerConfigDTO) -> None:
        """Validate the device block and prepare the static mapping.

        Validates the driver-specific ``device`` block, builds the
        name<->address lookup tables, verifies that every ``io_name`` is
        globally unique across all categories, and seeds the value cache from
        each signal's configured ``default``. Establishes no live hardware
        links (that happens in :meth:`activate`).

        Args:
            config: The validated device-agnostic configuration envelope. Its
                ``device`` block is opaque here and is validated by the
                concrete driver.

        Returns:
            None.

        Raises:
            ValueError: If the device block is malformed, or if any ``io_name``
                is duplicated across categories.
        """

    @abstractmethod
    def activate(self) -> None:
        """Establish live hardware links.

        Creates subscriptions and service clients on the owning node's executor
        and waits (with a timeout) for any required servers to become
        available.

        Returns:
            None.

        Raises:
            RuntimeError: If a required hardware link cannot be established
                (e.g. a dependency service does not appear within the timeout).
        """

    @abstractmethod
    def deactivate(self) -> None:
        """Drop live hardware links while keeping the static mapping and cache.

        Destroys subscriptions and service clients created by :meth:`activate`.
        The name<->address maps and the last cached values are retained so the
        driver can be re-activated without re-configuring.

        Returns:
            None.
        """

    @abstractmethod
    def snapshot(self) -> IOStatesSnapshot:
        """Return the latest cached values for all mapped signals.

        Returns:
            IOStatesSnapshot: A normalized snapshot containing an immutable
            value DTO for every mapped digital and analog signal, using the
            driver's most recent cached values.
        """

    @abstractmethod
    def set_change_callback(self, callback: ChangeCallback | None) -> None:
        """Register or clear the per-signal value-change callback.

        The callback is invoked with ``(io_name, new_signal)`` where
        ``new_signal`` is an immutable value snapshot. The driver only invokes
        it for genuine value changes (edge detected against the previous cached
        value), synchronously on the driver's own cache-update thread. The
        callback MUST be non-blocking (enqueue-and-return), so value caching is
        never delayed by downstream consumers.

        Args:
            callback: The callable to invoke on each genuine value change, or
                ``None`` to unregister any previously registered callback.

        Returns:
            None.
        """

    @abstractmethod
    def set_digital(self, io_name: str, state: bool) -> None:
        """Command one digital output.

        Args:
            io_name: The logical name of the target signal. Must resolve to a
                mapped digital output.
            state: The desired logical value to write.

        Returns:
            None.

        Raises:
            KeyError: If ``io_name`` does not resolve to a mapped signal.
            ValueError: If the target signal is not a writable digital output.
            RuntimeError: If the underlying hardware write fails or times out.
                The node catches this and converts it to a ``SetIO`` result at
                the boundary.
        """

    @abstractmethod
    def set_analog(self, io_name: str, state: float) -> None:
        """Command one analog output.

        Args:
            io_name: The logical name of the target signal. Must resolve to a
                mapped analog output.
            state: The desired value to write, interpreted in the signal's
                configured domain (voltage or current).

        Returns:
            None.

        Raises:
            KeyError: If ``io_name`` does not resolve to a mapped signal.
            ValueError: If the target signal is not a writable analog output.
            RuntimeError: If the underlying hardware write fails or times out.
                The node catches this and converts it to a ``SetIO`` result at
                the boundary.
        """
