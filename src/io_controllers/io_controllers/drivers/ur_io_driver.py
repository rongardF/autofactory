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
"""Universal Robots I/O driver.

Wraps ``ur_robot_driver``'s ``io_and_status_controller``:

* **Read path** — subscribes to ``{controller_namespace}/io_states``
  (``ur_msgs/msg/IOStates``). Each message updates the cache per pin/category;
  every signal whose cached value genuinely changed fires the change callback.
* **Write path** — calls ``{controller_namespace}/set_io``
  (``ur_msgs/srv/SetIO``) for digital outs, flags, and tool voltage, and
  ``{controller_namespace}/set_analog_output`` (``ur_msgs/srv/SetAnalogOutput``)
  for analog outs.
"""

import threading
from typing import Any

from rclpy.node import Node

from ur_msgs.msg import Analog
from ur_msgs.msg import IOStates as URIOStates
from ur_msgs.srv import SetAnalogOutput, SetIO

from io_controllers.drivers.mapping_normalizer import (
    check_unique_io_names,
    normalize_mapping,
)
from io_controllers.enums.device_type_enum import DeviceTypeEnum
from io_controllers.enums.io_domain_enum import IODomainEnum
from io_controllers.interfaces.io_driver import AbstractIODriver, ChangeCallback
from io_controllers.models.io_config_dto import IOControllerConfigDTO
from io_controllers.models.io_signal_dto import (
    AnalogSignalDTO,
    DigitalSignalDTO,
    IOStatesSnapshot,
)
from io_controllers.models.ur_config_dto import URDeviceConfigDTO

# Write kinds, used to route set_digital / set_analog to the right request.
_WRITE_DIGITAL_OUT = 'digital_out'
_WRITE_FLAG = 'flag'
_WRITE_ANALOG_OUT = 'analog_out'
_WRITE_TOOL_VOLTAGE = 'tool_voltage'


class URIODriver(AbstractIODriver):
    """Driver for Universal Robots via ``ur_robot_driver``'s io controller."""

    def __init__(self, node: Node) -> None:
        """Store the owning node used to create subscriptions and clients.

        Args:
            node: The owning lifecycle node. Live ROS entities are created on
                its executor when the driver is activated.
        """
        self._node = node
        self._device: URDeviceConfigDTO | None = None
        self._cache: dict[str, DigitalSignalDTO | AnalogSignalDTO] = {}
        self._change_callback: ChangeCallback | None = None

        # Guards every read/write of ``self._cache`` so a ``snapshot`` taken on
        # one executor thread never observes a partial ``io_states`` update
        # applied on another. Critical sections are tiny (in-memory only); the
        # change callback is invoked outside the lock.
        self._cache_lock = threading.Lock()

        # Read index: category -> {pin -> io_name}, for matching io_states entries.
        self._read_index: dict[str, dict[int, str]] = {}
        # Write index: io_name -> (kind, pin). Tool voltage carries no pin (-1).
        self._write_index: dict[str, tuple[str, int]] = {}

        # Live ROS entities (created in activate, dropped in deactivate).
        self._io_states_sub: Any = None
        self._set_io_client: Any = None
        self._set_analog_client: Any = None

        # Lazily imported ur_msgs symbols (populated in activate).
        self._set_io_srv: Any = None
        self._set_analog_srv: Any = None
        self._analog_msg: Any = None

    def configure(self, config: IOControllerConfigDTO) -> None:
        """Validate the UR device block and build the pin lookup tables.

        Args:
            config: The validated configuration envelope; its ``device`` block
                is parsed into a :class:`URDeviceConfigDTO`.

        Returns:
            None.

        Raises:
            ValueError: If any ``io_name`` is duplicated across categories or a
                pin repeats within a category.
            pydantic.ValidationError: If the UR device block fails validation.
        """
        self._device = URDeviceConfigDTO(**config.device)

        # Normalized cache (shared normalizer => identical shape to the mock).
        signals = normalize_mapping(DeviceTypeEnum.UR, config.device)
        check_unique_io_names(signals)
        self._cache = {signal.io_name: signal for signal in signals}

        # Build the UR-specific pin lookup tables from the validated mapping.
        self._read_index = {}
        self._write_index = {}
        mapping = self._device.mapping

        for category, entries, kind in (
            ('digital_in_states', mapping.digital_in_states, None),
            ('digital_out_states', mapping.digital_out_states, _WRITE_DIGITAL_OUT),
            ('flag_states', mapping.flag_states, _WRITE_FLAG),
            ('analog_in_states', mapping.analog_in_states, None),
            ('analog_out_states', mapping.analog_out_states, _WRITE_ANALOG_OUT),
        ):
            pin_map: dict[int, str] = {}
            for entry in entries:
                if entry.pin in pin_map:
                    raise ValueError(
                        f'duplicate pin {entry.pin} in UR category {category!r}'
                    )
                pin_map[entry.pin] = entry.io_name
                if kind is not None:
                    self._write_index[entry.io_name] = (kind, entry.pin)
            self._read_index[category] = pin_map

        # Tool voltage is command-only (no io_states field, no pin).
        for entry in mapping.tool_voltage:
            self._write_index[entry.io_name] = (_WRITE_TOOL_VOLTAGE, -1)

    def activate(self) -> None:
        """Subscribe to io_states and connect the write-path services.

        Returns:
            None.

        Raises:
            RuntimeError: If a required UR service is not available within the
                configured ``service_timeout_sec``.
        """
        assert self._device is not None  # set in configure(), before activate()
        self._set_io_srv = SetIO
        self._set_analog_srv = SetAnalogOutput
        self._analog_msg = Analog

        ns = self._device.controller_namespace.rstrip('/')
        timeout = self._device.service_timeout_sec

        self._io_states_sub = self._node.create_subscription(
            URIOStates, f'{ns}/io_states', self._on_io_states, 10
        )
        self._set_io_client = self._node.create_client(SetIO, f'{ns}/set_io')
        self._set_analog_client = self._node.create_client(
            SetAnalogOutput, f'{ns}/set_analog_output'
        )

        for client, name in (
            (self._set_io_client, f'{ns}/set_io'),
            (self._set_analog_client, f'{ns}/set_analog_output'),
        ):
            if not client.wait_for_service(timeout_sec=timeout):
                msg = f'UR service not available: {name}'
                self._node.get_logger().error(msg)
                raise RuntimeError(msg)

    def deactivate(self) -> None:
        """Drop the io_states subscription and the write-path service clients.

        Returns:
            None.
        """
        if self._io_states_sub is not None:
            self._node.destroy_subscription(self._io_states_sub)
            self._io_states_sub = None
        if self._set_io_client is not None:
            self._node.destroy_client(self._set_io_client)
            self._set_io_client = None
        if self._set_analog_client is not None:
            self._node.destroy_client(self._set_analog_client)
            self._set_analog_client = None

    def snapshot(self) -> IOStatesSnapshot:
        """Return the current cached value of every mapped signal.

        Returns:
            IOStatesSnapshot: Immutable value snapshots split into digital and
            analog lists.
        """
        with self._cache_lock:
            values = list(self._cache.values())
        digital = [s for s in values if isinstance(s, DigitalSignalDTO)]
        analog = [s for s in values if isinstance(s, AnalogSignalDTO)]
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
        """Command a digital output or flag via ``set_io``.

        Args:
            io_name: The target signal's logical name.
            state: The desired logical value.

        Returns:
            None.

        Raises:
            KeyError: If ``io_name`` is not a mapped signal.
            ValueError: If the signal is an input or not a digital output/flag.
            RuntimeError: If the driver is not activated.
        """
        kind, pin = self._require_write(io_name)
        if kind not in (_WRITE_DIGITAL_OUT, _WRITE_FLAG):
            raise ValueError(f'io_name {io_name!r} is not a digital output')
        request = self._set_io_srv.Request()
        request.fun = (
            self._set_io_srv.Request.FUN_SET_DIGITAL_OUT
            if kind == _WRITE_DIGITAL_OUT
            else self._set_io_srv.Request.FUN_SET_FLAG
        )
        request.pin = pin
        request.state = float(
            self._set_io_srv.Request.STATE_ON
            if state
            else self._set_io_srv.Request.STATE_OFF
        )
        self._send(self._set_io_client, request, io_name)

    def set_analog(self, io_name: str, state: float) -> None:
        """Command an analog output or tool voltage.

        Analog outputs go via ``set_analog_output`` (carrying the configured
        domain); tool voltage goes via ``set_io`` with ``FUN_SET_TOOL_VOLTAGE``
        and, being command-only, updates the cache directly.

        Args:
            io_name: The target signal's logical name.
            state: The desired value in the signal's configured domain.

        Returns:
            None.

        Raises:
            KeyError: If ``io_name`` is not a mapped signal.
            ValueError: If the signal is an input or not an analog output.
            RuntimeError: If the driver is not activated.
        """
        kind, pin = self._require_write(io_name)
        if kind == _WRITE_ANALOG_OUT:
            with self._cache_lock:
                signal = self._cache[io_name]
            assert isinstance(signal, AnalogSignalDTO)  # analog_out is always analog
            request = self._set_analog_srv.Request()
            request.data.pin = pin
            request.data.domain = self._ur_domain(signal.domain)
            request.data.state = float(state)
            self._send(self._set_analog_client, request, io_name)
        elif kind == _WRITE_TOOL_VOLTAGE:
            request = self._set_io_srv.Request()
            request.fun = self._set_io_srv.Request.FUN_SET_TOOL_VOLTAGE
            request.pin = 0
            request.state = float(state)
            self._send(self._set_io_client, request, io_name)
            # Command-only: no readback, so reflect it in the cache directly.
            self._update_cache(io_name, float(state))
        else:
            raise ValueError(f'io_name {io_name!r} is not an analog output')

    def _on_io_states(self, msg: object) -> None:
        """io_states callback: update the cache and fire per-change callbacks.

        Runs on an executor thread (subscription callback). Only genuine value
        changes (exact inequality) fire the change callback. Tool voltage is
        never present here (command-only).

        Args:
            msg: An incoming ``ur_msgs/msg/IOStates`` message.
        """
        for category in ('digital_in_states', 'digital_out_states', 'flag_states'):
            pin_map = self._read_index.get(category, {})
            for entry in getattr(msg, category):
                io_name = pin_map.get(entry.pin)
                if io_name is not None:
                    self._update_cache(io_name, bool(entry.state))

        for category in ('analog_in_states', 'analog_out_states'):
            pin_map = self._read_index.get(category, {})
            for entry in getattr(msg, category):
                io_name = pin_map.get(entry.pin)
                if io_name is not None:
                    self._update_cache(io_name, float(entry.state))

    def _require_write(self, io_name: str) -> tuple[str, int]:
        """Resolve *io_name* to its (kind, pin) write target or raise."""
        if self._set_io_client is None or self._set_analog_client is None:
            raise RuntimeError('UR driver is not activated; no service clients')
        meta = self._write_index.get(io_name)
        if meta is None:
            if io_name in self._cache:
                raise ValueError(
                    f'io_name {io_name!r} is an input; inputs are not writable'
                )
            raise KeyError(f'unknown io_name: {io_name!r}')
        return meta

    def _ur_domain(self, domain: IODomainEnum) -> int:
        """Map a normalized domain to the ur_msgs Analog domain constant."""
        return (
            self._analog_msg.CURRENT
            if domain is IODomainEnum.CURRENT
            else self._analog_msg.VOLTAGE
        )

    def _send(self, client: Any, request: object, io_name: str) -> None:
        """Send a write request asynchronously, logging any failed result."""
        future = client.call_async(request)

        def _done(fut: Any) -> None:
            try:
                result = fut.result()
            except Exception as exc:  # noqa: BLE001 - log and drop
                self._node.get_logger().error(
                    f"UR write for '{io_name}' raised: {exc}"
                )
                return
            if result is not None and not getattr(result, 'success', True):
                self._node.get_logger().error(
                    f"UR write for '{io_name}' returned success=false"
                )

        future.add_done_callback(_done)

    def _update_cache(self, io_name: str, value: bool | float) -> None:
        """Update a cached signal, firing the change callback only on change.

        The compare-and-swap runs under ``_cache_lock``; the change callback is
        invoked outside the lock so a slow (or re-entrant) consumer can never
        stall other cache readers or writers.
        """
        with self._cache_lock:
            signal = self._cache.get(io_name)
            if signal is None or signal.state == value:
                return
            updated = signal.model_copy(update={'state': value})
            self._cache[io_name] = updated
        if self._change_callback is not None:
            self._change_callback(io_name, updated)
