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

from abc import ABC, abstractmethod
from typing import Callable

from rclpy.node import Node

from product_carrier.model import CarrierActionResultDTO, ConveyorNodeConfigDTO

PresenceChangedCallback = Callable[[bool], None]


class CarrierController(ABC):
    """Backend abstraction for product carrying hardware.

    A carrier moves products into and out of the robot workplace slot, e.g. a
    conveyor line carries the product along the line, a trolley lifter raises or
    lowers a trolley. Implementations exist per carrier type and per backend
    (simulated or real hardware) so the owning node is backend agnostic.

    The constructor only stores its arguments; all ROS entities, connections and
    I/O are created in :meth:`setup` and released in :meth:`teardown`.
    """

    def __init__(self, node: Node, config: ConveyorNodeConfigDTO) -> None:
        """Store the owning node and its configuration without initialising anything.

        :param node: Node that owns the ROS entities created by the controller.
        :param config: Frozen node configuration.
        """
        self._node = node
        self._config = config

    @abstractmethod
    def setup(self, on_presence_changed: PresenceChangedCallback) -> None:
        """Initialise the backend.

        :param on_presence_changed: Called with the new value whenever the
            product presence state in the workplace changes.
        :raises Exception: If the backend cannot be initialised.
        """

    @abstractmethod
    def teardown(self) -> None:
        """Release everything created in :meth:`setup`. Must be idempotent."""

    @abstractmethod
    def inject_product(self) -> CarrierActionResultDTO:
        """Move a new product into the robot workplace slot."""

    @abstractmethod
    def eject_product(self) -> CarrierActionResultDTO:
        """Move the current product out of the robot workplace slot."""

    @abstractmethod
    def product_in_place(self) -> bool:
        """Return the latest known product presence state in the workplace."""
