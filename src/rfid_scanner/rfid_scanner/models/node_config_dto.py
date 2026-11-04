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
"""Frozen configuration of an RFID scanner node."""

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

from rfid_scanner.enums.parity_enum import ParityEnum


class NodeConfigDTO(BaseModel):
    """Node parameters, validated once in ``on_configure``."""

    model_config = ConfigDict(frozen=True, extra='forbid')

    simulated: bool = Field(
        default=True,
        description='Use the Gazebo-simulated reader (True) or real hardware (False).',
    )
    device_name: str = Field(
        default='simatic_rf240r',
        pattern=r'^[A-Za-z][A-Za-z0-9_]*$',
        description='Device name; namespaces the Gazebo topics and write service. '
        'Must match the xacro device_name.',
    )
    serial_port: str = Field(
        default='/dev/ttyUSB0',
        min_length=1,
        description='Serial device of the reader (real hardware only).',
    )
    baud_rate: Literal[19200, 57600, 115200] = Field(
        default=19200,
        description='Serial baud rate; must match the setting stored on the reader.',
    )
    parity: ParityEnum = Field(
        default=ParityEnum.ODD,
        description='Serial parity; must match the setting stored on the reader.',
    )
    stop_bits: Literal[1, 2] = Field(
        default=1,
        description='Serial stop bits; must match the setting stored on the reader.',
    )
    response_timeout_sec: float = Field(
        default=1.0,
        gt=0,
        description='Maximum wait for a command acknowledgement, in seconds.',
    )
    heartbeat_period_sec: float = Field(
        default=1.0,
        gt=0,
        description='Period of the SLG-STATUS heartbeat, in seconds (real hardware only).',
    )

    @property
    def detected_topic(self) -> str:
        """Return the bridged Gazebo detection topic."""
        return f'/{self.device_name}/detected'

    @property
    def tag_data_topic(self) -> str:
        """Return the bridged Gazebo tag content topic."""
        return f'/{self.device_name}/tag_data'

    @property
    def write_service(self) -> str:
        """Return the Gazebo write service name."""
        return f'/{self.device_name}/write_tag'
