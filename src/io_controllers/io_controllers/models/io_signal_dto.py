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
"""Normalized I/O signal data models (device-agnostic).

These DTOs are the internal, wire-independent representation of a single mapped
signal and of a full snapshot of all signals. The individual signal DTOs are
frozen immutable value objects: config describes the *mapping* while a driver
tracks live values by constructing new snapshots. ROS messages remain the wire
format; each model exposes a ``to_msg()`` method that translates it into the
corresponding generated ROS message for publishing.
"""

from pydantic import BaseModel, Field

from io_controllers.enums.io_direction_enum import IODirectionEnum
from io_controllers.enums.io_domain_enum import IODomainEnum
from io_controllers.msg import AnalogIO, DigitalIO, IOStates


class DigitalSignalDTO(BaseModel, frozen=True):
    """A single digital (boolean) signal in normalized form."""

    io_name: str = Field(
        description='Unique logical name of the signal from the config mapping.',
    )
    direction: IODirectionEnum = Field(
        description='Whether the signal is read from (IN) or written to (OUT).',
    )
    default: bool = Field(
        default=False,
        description='Initial cached value before any hardware/mock update and '
        'the starting value in simulated mode.',
    )
    state: bool = Field(
        default=False,
        description='Current logical value (read) or desired value (command).',
    )

    def to_msg(self) -> DigitalIO:
        """Convert this DTO into a ``DigitalIO`` ROS message.

        The enum value round-trips directly to the string message field because
        the enum values match the message constants exactly.

        Returns:
            DigitalIO: The equivalent ROS message.
        """
        msg = DigitalIO()
        msg.io_name = self.io_name
        msg.direction = self.direction.value
        msg.state = bool(self.state)
        return msg


class AnalogSignalDTO(BaseModel, frozen=True):
    """A single analog signal in normalized form."""

    io_name: str = Field(
        description='Unique logical name of the signal from the config mapping.',
    )
    direction: IODirectionEnum = Field(
        description='Whether the signal is read from (IN) or written to (OUT).',
    )
    domain: IODomainEnum = Field(
        description='Physical domain of the analog value (voltage or current).',
    )
    default: float = Field(
        default=0.0,
        description='Initial cached value before any hardware/mock update and '
        'the starting value in simulated mode.',
    )
    state: float = Field(
        default=0.0,
        description='Current value (read) or desired value (command).',
    )

    def to_msg(self) -> AnalogIO:
        """Convert this DTO into an ``AnalogIO`` ROS message.

        The enum values round-trip directly to the string message fields because
        the enum values match the message constants exactly.

        Returns:
            AnalogIO: The equivalent ROS message.
        """
        msg = AnalogIO()
        msg.io_name = self.io_name
        msg.direction = self.direction.value
        msg.state = float(self.state)
        msg.domain = self.domain.value
        return msg


class IOStatesSnapshot(BaseModel):
    """Latest normalized cached value of every mapped signal.

    Return type of ``IODriver.snapshot()`` and the source the publish timer
    builds each ``IOStates`` message from. Not frozen: its contents change as
    the driver cache updates. Each contained signal DTO is an immutable value
    snapshot, so a snapshot captured at change time stays stable even if the
    cache advances afterwards.
    """

    digital: list[DigitalSignalDTO] = Field(
        default_factory=list,
        description='Current value snapshots of all mapped digital signals.',
    )
    analog: list[AnalogSignalDTO] = Field(
        default_factory=list,
        description='Current value snapshots of all mapped analog signals.',
    )

    def to_msg(self) -> IOStates:
        """Convert this snapshot into an ``IOStates`` ROS message (header unset).

        The caller is expected to stamp ``msg.header`` (timestamp/frame_id) after
        conversion, since the clock and frame are node concerns.

        Returns:
            IOStates: The equivalent ROS message with ``digital_io`` /
            ``analog_io`` populated and an unstamped header.
        """
        msg = IOStates()
        msg.digital_io = [s.to_msg() for s in self.digital]
        msg.analog_io = [s.to_msg() for s in self.analog]
        return msg
