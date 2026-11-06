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
"""Frozen configuration of the ``workplace_rfid_scanner`` node."""

from pydantic import BaseModel, ConfigDict, Field

from station_workplace.models.common_node_config_dto import CommonNodeConfigDTO, ROS_NAME_PATTERN


class TagRegisterDTO(BaseModel):
    """Location of one product data field in the RFID tag memory."""

    model_config = ConfigDict(frozen=True, extra='forbid')

    start_address: int = Field(ge=0, description='First tag address of the field.')
    length: int = Field(default=32, ge=1, le=248, description='Byte length of the field.')


class TagRegistersDTO(BaseModel):
    """Locations of all product data fields in the RFID tag memory."""

    model_config = ConfigDict(frozen=True, extra='forbid')

    product_number: TagRegisterDTO = Field(
        default_factory=lambda: TagRegisterDTO(start_address=0),
        description='Location of the product number.',
    )
    product_revision: TagRegisterDTO = Field(
        default_factory=lambda: TagRegisterDTO(start_address=32),
        description='Location of the product revision.',
    )
    serial_number: TagRegisterDTO = Field(
        default_factory=lambda: TagRegisterDTO(start_address=64),
        description='Location of the serial number.',
    )


class RfidNodeConfigDTO(CommonNodeConfigDTO):
    """Parameters of the ``workplace_rfid_scanner`` node."""

    rfid_scanner_node_name: str = Field(
        default='simatic_rf240r_ascii', pattern=ROS_NAME_PATTERN,
        description='RFID scanner node name.',
    )
    tag_detected_timeout_sec: float = Field(
        default=5.0, gt=0,
        description='Maximum wait for tag_detected=True after product arrival [s].',
    )
    tag_registers: TagRegistersDTO = Field(
        default_factory=TagRegistersDTO,
        description='Locations of the product data fields in the tag memory.',
    )

    @property
    def tag_detected_topic(self) -> str:
        """Return the RFID scanner ``tag_detected`` topic."""
        return f'/{self.rfid_scanner_node_name}/tag_detected'

    @property
    def read_data_service(self) -> str:
        """Return the RFID scanner ``read_data`` service."""
        return f'/{self.rfid_scanner_node_name}/read_data'
