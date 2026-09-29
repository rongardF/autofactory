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
"""Device-agnostic simulation driver.

``MockIODriver`` implements the full :class:`AbstractIODriver` contract with an
in-memory cache and no hardware links, so the node's entire public surface
(topic, ``set_io``, ``monitor_io``) works with no real robot. It consumes the
same normalized mapping as the real drivers (via the shared mapping
normalizer), so one mock covers every device family. Vendor-only fields such as
``controller_namespace`` are ignored.

Direction rules match real hardware: outputs move only via ``set_digital`` /
``set_analog`` (driven by the node's ``set_io``), and inputs move only via
``apply_mock_change`` (driven by the node's ``set_mock_behavior`` timers).
"""

from io_controllers.drivers.mapping_normalizer import (
    check_unique_io_names,
    normalize_mapping,
)
from io_controllers.enums.io_direction_enum import IODirectionEnum
from io_controllers.interfaces.io_driver import AbstractIODriver, ChangeCallback
from io_controllers.models.io_config_dto import IOControllerConfigDTO
from io_controllers.models.io_signal_dto import (
    AnalogSignalDTO,
    DigitalSignalDTO,
    IOStatesSnapshot,
)

# A cached signal is either digital or analog.
_Signal = DigitalSignalDTO | AnalogSignalDTO


class MockIODriver(AbstractIODriver):
    """In-memory, device-agnostic driver used when ``simulated=true``."""

    def __init__(self) -> None:
        """Create an unconfigured mock driver with an empty cache."""
        self._cache: dict[str, _Signal] = {}
        self._change_callback: ChangeCallback | None = None

    def configure(self, config: IOControllerConfigDTO) -> None:
        """Normalize the mapping, verify uniqueness, and seed the cache.

        Args:
            config: The validated configuration envelope. Its ``device`` block
                is normalized by the shared normalizer selected via
                ``device_type``; vendor-only fields are ignored.

        Returns:
            None.

        Raises:
            ValueError: If the device block is malformed or any ``io_name`` is
                duplicated across categories.
        """
        signals = normalize_mapping(config.device_type, config.device)
        check_unique_io_names(signals)
        # Each normalized signal already has ``state`` seeded from ``default``.
        self._cache = {signal.io_name: signal for signal in signals}

    def activate(self) -> None:
        """No-op: the mock has no hardware links. The cache stays as seeded.

        Returns:
            None.
        """

    def deactivate(self) -> None:
        """No-op: the mock has no hardware links to drop.

        Returns:
            None.
        """

    def snapshot(self) -> IOStatesSnapshot:
        """Return the current cached value of every mapped signal.

        Returns:
            IOStatesSnapshot: Immutable value snapshots split into digital and
            analog lists.
        """
        digital = [s for s in self._cache.values() if isinstance(s, DigitalSignalDTO)]
        analog = [s for s in self._cache.values() if isinstance(s, AnalogSignalDTO)]
        return IOStatesSnapshot(digital=digital, analog=analog)

    def set_change_callback(self, callback: ChangeCallback | None) -> None:
        """Register or clear the value-change callback.

        Args:
            callback: Callable invoked on each genuine value change, or ``None``
                to unregister.

        Returns:
            None.
        """
        self._change_callback = callback

    def set_digital(self, io_name: str, state: bool) -> None:
        """Write a digital output value into the cache.

        Args:
            io_name: The target signal's logical name.
            state: The desired logical value.

        Returns:
            None.

        Raises:
            KeyError: If ``io_name`` is not mapped.
            ValueError: If the signal is not a digital output.
        """
        signal = self._require_output(io_name, DigitalSignalDTO)
        self._apply(signal, bool(state))

    def set_analog(self, io_name: str, state: float) -> None:
        """Write an analog output value into the cache.

        Args:
            io_name: The target signal's logical name.
            state: The desired value in the signal's configured domain.

        Returns:
            None.

        Raises:
            KeyError: If ``io_name`` is not mapped.
            ValueError: If the signal is not an analog output.
        """
        signal = self._require_output(io_name, AnalogSignalDTO)
        self._apply(signal, float(state))

    def apply_mock_change(self, io_name: str, value: bool | float) -> None:
        """Drive a simulated INPUT signal's value (simulation-only hook).

        Called by the node's ``set_mock_behavior`` scheduled timers. Not part
        of the :class:`AbstractIODriver` contract; real drivers never implement
        it.

        Args:
            io_name: The target input signal's logical name.
            value: The new value (bool for digital, float for analog).

        Returns:
            None.

        Raises:
            KeyError: If ``io_name`` is not mapped.
            ValueError: If the signal is an output (only inputs are scriptable).
        """
        signal = self._cache.get(io_name)
        if signal is None:
            raise KeyError(f'unknown io_name: {io_name!r}')
        if signal.direction is not IODirectionEnum.IN:
            raise ValueError(
                f'io_name {io_name!r} is an output; only inputs are scriptable'
            )
        coerced = bool(value) if isinstance(signal, DigitalSignalDTO) else float(value)
        self._apply(signal, coerced)

    def _require_output(self, io_name: str, expected: type[_Signal]) -> _Signal:
        """Resolve *io_name* to a mapped output signal of *expected* type."""
        signal = self._cache.get(io_name)
        if signal is None:
            raise KeyError(f'unknown io_name: {io_name!r}')
        if signal.direction is not IODirectionEnum.OUT:
            raise ValueError(f'io_name {io_name!r} is an input; inputs are not writable')
        if not isinstance(signal, expected):
            kind = 'digital' if expected is DigitalSignalDTO else 'analog'
            raise ValueError(f'io_name {io_name!r} is not a {kind} signal')
        return signal

    def _apply(self, signal: _Signal, value: bool | float) -> None:
        """Update *signal*'s cached value, firing the callback only on change."""
        if signal.state == value:
            return
        updated = signal.model_copy(update={'state': value})
        self._cache[signal.io_name] = updated
        if self._change_callback is not None:
            self._change_callback(updated.io_name, updated)
