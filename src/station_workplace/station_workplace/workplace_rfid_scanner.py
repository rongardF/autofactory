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
"""Workplace lifecycle node identifying the product by its RFID tag.

On ``/<product_carrier_node_name>/product_in_place`` = True the product data is
read from the tag via ``/<rfid_scanner_node_name>/read_data`` once
``/<rfid_scanner_node_name>/tag_detected`` is True; the shared workplace
behaviour is implemented in :class:`WorkplaceNodeBase`.
"""

from typing import Any

import rclpy

from station_workplace.interface.product_data_reader import ProductDataReader
from station_workplace.models.common_node_config_dto import CommonNodeConfigDTO
from station_workplace.models.rfid_node_config_dto import RfidNodeConfigDTO, TagRegistersDTO
from station_workplace.services.rfid_product_data_reader import RfidProductDataReader
from station_workplace.workplace_node_base import WorkplaceNodeBase, spin_node


class WorkplaceRfidScanner(WorkplaceNodeBase):
    """Workplace node reading the product data from an RFID tag."""

    _FIELDS = ('product_number', 'product_revision', 'serial_number')

    def __init__(self, node_name: str = 'workplace_rfid_scanner') -> None:
        """Declare the parameters; resources are created by lifecycle transitions."""
        super().__init__(node_name)

    def _declare_specific_parameters(self) -> None:
        defaults = RfidNodeConfigDTO()
        self._declare('rfid_scanner_node_name', defaults.rfid_scanner_node_name,
                      'RFID scanner node name.')
        self._declare('tag_detected_timeout_sec', defaults.tag_detected_timeout_sec,
                      'Maximum wait for tag_detected=True after product arrival [s] (> 0).')
        registers = TagRegistersDTO()
        for field in self._FIELDS:
            register = getattr(registers, field)
            self._declare(f'tag_registers.{field}.start_address', register.start_address,
                          f'First tag address of the {field} field (>= 0).')
            self._declare(f'tag_registers.{field}.length', register.length,
                          f'Byte length of the {field} field (1..248).')

    def _build_config(self, common: dict[str, Any]) -> CommonNodeConfigDTO:
        return RfidNodeConfigDTO.model_validate({
            **common,
            'rfid_scanner_node_name': self._param('rfid_scanner_node_name'),
            'tag_detected_timeout_sec': self._param('tag_detected_timeout_sec'),
            'tag_registers': {
                field: {
                    'start_address': self._param(f'tag_registers.{field}.start_address'),
                    'length': self._param(f'tag_registers.{field}.length'),
                }
                for field in self._FIELDS
            },
        })

    def _create_reader(self, config: CommonNodeConfigDTO) -> ProductDataReader:
        assert isinstance(config, RfidNodeConfigDTO)
        return RfidProductDataReader(config)


def main(args: list[str] | None = None) -> None:
    """Spin the workplace RFID scanner node until shutdown."""
    rclpy.init(args=args)
    spin_node(WorkplaceRfidScanner())


if __name__ == '__main__':
    main()
