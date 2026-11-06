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
"""Simulation hooks of the RFID workplace node."""

from station_workplace.interface.simulation_hooks import SimulationHooks
from station_workplace.models.product_data_dto import ProductDataDTO
from station_workplace.models.rfid_node_config_dto import RfidNodeConfigDTO
from station_workplace.utils.tag_data_encoder import encode_tag_data


class RfidSimulationHooks(SimulationHooks):
    """Write the product data to the carrier's simulated RFID tag (``tag_data`` argument)."""

    def __init__(self, config: RfidNodeConfigDTO) -> None:
        """Store the tag register layout."""
        self._registers = config.tag_registers

    def carrier_mappings(self, product: ProductDataDTO) -> dict[str, str]:
        """Return the ``tag_data`` argument holding the encoded product data."""
        return {'tag_data': encode_tag_data(product, self._registers)}

    def after_spawn(self, product: ProductDataDTO) -> None:
        """Nothing to do; the tag data is part of the carrier model."""
