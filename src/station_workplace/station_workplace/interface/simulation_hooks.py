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
"""Node-specific behaviour of the simulated product spawning."""

from abc import ABC, abstractmethod

from rclpy.node import Node

from station_workplace.models.product_data_dto import ProductDataDTO


class SimulationHooks(ABC):
    """Hooks of the workplace node variant (RFID, barcode) used by the spawner."""

    def activate(self, node: Node) -> None:
        """Create the node-specific interfaces."""

    def deactivate(self) -> None:
        """Destroy the node-specific interfaces."""

    @abstractmethod
    def carrier_mappings(self, product: ProductDataDTO) -> dict[str, str]:
        """Return the node-specific xacro arguments of the carrier model.

        Called before anything is fetched or spawned.

        :raises ValueError: If the product data cannot be represented.
        """

    @abstractmethod
    def after_spawn(self, product: ProductDataDTO) -> None:
        """Run after both models were spawned.

        :raises Exception: Any failure triggers the rollback of the spawned models.
        """
