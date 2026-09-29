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
"""Device mapping normalization: vendor ``device`` block -> normalized signals.

The config ``device`` block is driver-defined and opaque to the base loader
(see the config DTO). Turning it into the device-agnostic list of
:class:`DigitalSignalDTO` / :class:`AnalogSignalDTO` therefore requires
device-specific knowledge (the UR category names, for example, encode a
signal's direction and type). That knowledge lives here as small, pure,
ROS-free functions keyed by :class:`DeviceTypeEnum`.

Both the device-agnostic ``MockIODriver`` and the real per-device drivers
reuse these normalizers, so the normalized mapping is identical in simulated
and real modes.
"""

from collections.abc import Callable

from io_controllers.enums.device_type_enum import DeviceTypeEnum
from io_controllers.enums.io_direction_enum import IODirectionEnum
from io_controllers.enums.io_domain_enum import IODomainEnum
from io_controllers.models.io_signal_dto import (
    AnalogSignalDTO,
    DigitalSignalDTO,
)

# A signal in normalized form is either digital or analog.
Signal = DigitalSignalDTO | AnalogSignalDTO

# A normalizer turns a vendor ``device`` block into the normalized signal list.
Normalizer = Callable[[dict], list[Signal]]

# UR mapping categories -> (is_analog, direction). ``tool_voltage`` is handled
# separately because it carries no pin and is always a voltage output.
_UR_DIGITAL_CATEGORIES: dict[str, IODirectionEnum] = {
    'digital_in_states': IODirectionEnum.IN,
    'digital_out_states': IODirectionEnum.OUT,
    'flag_states': IODirectionEnum.OUT,
}
_UR_ANALOG_CATEGORIES: dict[str, IODirectionEnum] = {
    'analog_in_states': IODirectionEnum.IN,
    'analog_out_states': IODirectionEnum.OUT,
}


def _require_io_name(entry: dict, category: str) -> str:
    """Extract a non-empty ``io_name`` from a mapping *entry*."""
    io_name = entry.get('io_name')
    if not isinstance(io_name, str) or not io_name:
        raise ValueError(
            f'mapping entry in {category!r} is missing a valid io_name: {entry!r}'
        )
    return io_name


def normalize_ur_mapping(device: dict) -> list[Signal]:
    """Normalize a Universal Robots ``device`` block into signal DTOs.

    Args:
        device: The vendor ``device`` block. Its ``mapping`` sub-dict groups
            entries into the UR categories (``digital_in_states``,
            ``digital_out_states``, ``flag_states``, ``analog_in_states``,
            ``analog_out_states``, ``tool_voltage``). Vendor-only fields such as
            ``controller_namespace`` are ignored here.

    Returns:
        The normalized list of digital and analog signal DTOs, each with its
        ``state`` seeded to its configured ``default``.

    Raises:
        ValueError: If ``mapping`` is missing/not a mapping, an entry lacks a
            valid ``io_name``, or an analog ``domain`` value is invalid.
    """
    mapping = device.get('mapping')
    if not isinstance(mapping, dict):
        raise ValueError("UR device block requires a 'mapping' dictionary")

    signals: list[Signal] = []

    for category, direction in _UR_DIGITAL_CATEGORIES.items():
        for entry in mapping.get(category, []) or []:
            io_name = _require_io_name(entry, category)
            default = bool(entry.get('default', False))
            signals.append(
                DigitalSignalDTO(
                    io_name=io_name,
                    direction=direction,
                    default=default,
                    state=default,
                )
            )

    for category, direction in _UR_ANALOG_CATEGORIES.items():
        for entry in mapping.get(category, []) or []:
            io_name = _require_io_name(entry, category)
            default = float(entry.get('default', 0.0))
            domain = _parse_domain(entry.get('domain', IODomainEnum.VOLTAGE.value))
            signals.append(
                AnalogSignalDTO(
                    io_name=io_name,
                    direction=direction,
                    domain=domain,
                    default=default,
                    state=default,
                )
            )

    # UR tool voltage: a single settable output in the voltage domain, no pin.
    for entry in mapping.get('tool_voltage', []) or []:
        io_name = _require_io_name(entry, 'tool_voltage')
        default = float(entry.get('default', 0.0))
        signals.append(
            AnalogSignalDTO(
                io_name=io_name,
                direction=IODirectionEnum.OUT,
                domain=IODomainEnum.VOLTAGE,
                default=default,
                state=default,
            )
        )

    return signals


def _parse_domain(value: object) -> IODomainEnum:
    """Coerce a raw analog ``domain`` value into :class:`IODomainEnum`."""
    try:
        return IODomainEnum(value)
    except ValueError as exc:
        valid = ', '.join(d.value for d in IODomainEnum)
        raise ValueError(
            f'invalid analog domain {value!r}; expected one of: {valid}'
        ) from exc


# Registry mapping each supported device type to its normalizer.
_NORMALIZERS: dict[DeviceTypeEnum, Normalizer] = {
    DeviceTypeEnum.UR: normalize_ur_mapping,
}


def normalize_mapping(device_type: DeviceTypeEnum, device: dict) -> list[Signal]:
    """Normalize a vendor ``device`` block for the given *device_type*.

    Args:
        device_type: Which device family the ``device`` block describes.
        device: The vendor-specific ``device`` block.

    Returns:
        The normalized list of signal DTOs produced by the registered
        normalizer for *device_type*.

    Raises:
        ValueError: If no normalizer is registered for *device_type*, or the
            registered normalizer rejects the block.
    """
    normalizer = _NORMALIZERS.get(device_type)
    if normalizer is None:
        raise ValueError(f'no mapping normalizer registered for {device_type!r}')
    return normalizer(device)


def check_unique_io_names(signals: list[Signal]) -> None:
    """Verify that every signal's ``io_name`` is globally unique.

    Args:
        signals: The normalized signal list to check.

    Raises:
        ValueError: If any ``io_name`` appears more than once, listing the
            offending duplicate names.
    """
    seen: set[str] = set()
    duplicates: set[str] = set()
    for signal in signals:
        if signal.io_name in seen:
            duplicates.add(signal.io_name)
        seen.add(signal.io_name)
    if duplicates:
        offenders = ', '.join(sorted(duplicates))
        raise ValueError(f'duplicate io_name(s) in config mapping: {offenders}')
