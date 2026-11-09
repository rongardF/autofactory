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
"""Workplace lifecycle node identifying the product by its barcode label.

After ``/<product_carrier_node_name>/product_in_place`` = True the first
``/<barcode_scanner_node_name>/label_data`` stamped later than the arrival is
parsed with the ``label_patterns.*`` regexes; the shared workplace behaviour is
implemented in :class:`WorkplaceNodeBase`.
"""

from typing import Any

import rclpy

from station_workplace.interface.product_data_reader import ProductDataReader
from station_workplace.interface.simulation_hooks import SimulationHooks
from station_workplace.models.barcode_node_config_dto import BarcodeNodeConfigDTO
from station_workplace.models.common_node_config_dto import CommonNodeConfigDTO
from station_workplace.services.barcode_product_data_reader import BarcodeProductDataReader
from station_workplace.services.barcode_simulation_hooks import BarcodeSimulationHooks
from station_workplace.workplace_node_base import WorkplaceNodeBase, spin_node


class WorkplaceBarcodeScanner(WorkplaceNodeBase):
    """Workplace node reading the product data from a barcode label."""

    _FIELDS = ('product_number', 'product_revision', 'serial_number')
    
    def __init__(self, node_name: str = 'workplace_barcode_scanner') -> None:
        """Declare the parameters; resources are created by lifecycle transitions."""
        super().__init__(node_name)

    def _declare_specific_parameters(self) -> None:
        self._declare('barcode_scanner_node_name', 'barcode_scanner',
                      'Barcode scanner node name.')
        for field in self._FIELDS:
            self._declare(f'label_patterns.{field}', '',
                          f'Regex with exactly one capture group extracting {field} '
                          'from the label (required).')

    def _build_config(self, common: dict[str, Any]) -> CommonNodeConfigDTO:
        return BarcodeNodeConfigDTO.model_validate({
            **common,
            'barcode_scanner_node_name': self._param('barcode_scanner_node_name'),
            'label_patterns': {
                field: self._param(f'label_patterns.{field}') for field in self._FIELDS
            },
        })

    def _create_reader(self, config: CommonNodeConfigDTO) -> ProductDataReader:
        assert isinstance(config, BarcodeNodeConfigDTO)
        return BarcodeProductDataReader(config, self.get_logger())

    def _create_simulation_hooks(self, config: CommonNodeConfigDTO) -> SimulationHooks:
        assert isinstance(config, BarcodeNodeConfigDTO)
        return BarcodeSimulationHooks(config)


def main(args: list[str] | None = None) -> None:
    """Spin the workplace barcode scanner node until shutdown."""
    rclpy.init(args=args)
    spin_node(WorkplaceBarcodeScanner())


if __name__ == '__main__':
    main()
