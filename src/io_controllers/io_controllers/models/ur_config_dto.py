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
"""Universal Robots device configuration models.

These validate the ``device`` block of a UR ``IOControllerConfigDTO``. 
They carry the vendor-specific detail the device-agnostic model
deliberately omits — most importantly the UR **pin** number per signal, which
the driver needs to match ``io_states`` entries on the read path and to build
``set_io`` / ``set_analog_output`` requests on the write path. These models are
ROS-free (no ``ur_msgs`` dependency), so config validation works without a live
robot.
"""

from pydantic import BaseModel, Field

from io_controllers.enums.io_domain_enum import IODomainEnum


class URDigitalEntryDTO(BaseModel, frozen=True):
    """One UR digital signal mapping entry (digital in/out or flag)."""

    io_name: str = Field(
        description='Unique logical name exposed on the normalized interface.',
    )
    pin: int = Field(
        ge=0,
        description='UR pin number within its category (matches io_states pins '
        'and set_io requests).',
    )
    default: bool = Field(
        default=False,
        description='Initial cached value before the first hardware readback.',
    )


class URAnalogEntryDTO(BaseModel, frozen=True):
    """One UR analog signal mapping entry (analog in/out)."""

    io_name: str = Field(
        description='Unique logical name exposed on the normalized interface.',
    )
    pin: int = Field(
        ge=0,
        description='UR analog pin number (matches io_states pins and '
        'set_analog_output requests).',
    )
    domain: IODomainEnum = Field(
        default=IODomainEnum.VOLTAGE,
        description='Configured physical domain; authoritative for both the '
        'reported snapshot and outgoing set_analog_output requests.',
    )
    default: float = Field(
        default=0.0,
        description='Initial cached value before the first hardware readback.',
    )


class URToolVoltageEntryDTO(BaseModel, frozen=True):
    """The UR tool-voltage output entry (command-only; no io_states field)."""

    io_name: str = Field(
        description='Unique logical name exposed on the normalized interface.',
    )
    default: float = Field(
        default=0.0,
        description='Initial cached value; tool voltage is not read back from '
        'hardware, so the cache reflects only the last commanded value.',
    )


class URMappingDTO(BaseModel, frozen=True):
    """The six UR mapping categories."""

    digital_in_states: list[URDigitalEntryDTO] = Field(
        default_factory=list,
        description='Read-only digital inputs.',
    )
    digital_out_states: list[URDigitalEntryDTO] = Field(
        default_factory=list,
        description='Digital outputs (set via FUN_SET_DIGITAL_OUT).',
    )
    flag_states: list[URDigitalEntryDTO] = Field(
        default_factory=list,
        description='Bidirectional flags: readable in io_states, settable via '
        'FUN_SET_FLAG. Modeled as outputs for set_io writability.',
    )
    analog_in_states: list[URAnalogEntryDTO] = Field(
        default_factory=list,
        description='Read-only analog inputs.',
    )
    analog_out_states: list[URAnalogEntryDTO] = Field(
        default_factory=list,
        description='Analog outputs (set via set_analog_output).',
    )
    tool_voltage: list[URToolVoltageEntryDTO] = Field(
        default_factory=list,
        description='Tool voltage output (set via FUN_SET_TOOL_VOLTAGE); '
        'command-only, not present in io_states.',
    )


class URDeviceConfigDTO(BaseModel, frozen=True):
    """Validated UR ``device`` block: connection settings plus the pin mapping."""

    controller_namespace: str = Field(
        description='Namespace of the ur_robot_driver io_and_status_controller; '
        'io_states / set_io / set_analog_output resolve under it.',
    )
    service_timeout_sec: float = Field(
        default=5.0,
        gt=0.0,
        description='Timeout used when waiting for the UR write-path services.',
    )
    mapping: URMappingDTO = Field(
        description='The six UR signal categories with per-signal pin numbers.',
    )
