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
"""Abstract product data source."""

import threading
from abc import ABC, abstractmethod

from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.time import Time

from station_workplace.models.product_data_dto import ProductDataDTO


class ProductDataReader(ABC):
    """Produce the ``ProductDataDTO`` of the product that arrived in the workplace."""

    @abstractmethod
    def activate(self, node: Node, callback_group: CallbackGroup) -> None:
        """Create the ROS entities (subscriptions, clients)."""

    @abstractmethod
    def deactivate(self) -> None:
        """Destroy the ROS entities."""

    def on_product_arrived(self, arrival_time: Time) -> None:
        """Handle ``product_in_place=True``; called from the trigger callback."""

    def on_product_left(self) -> None:
        """Handle ``product_in_place=False``; called from the trigger callback."""

    @abstractmethod
    def read(self, cancel: threading.Event) -> ProductDataDTO:
        """Read the product data, blocking the calling worker thread.

        :raises InjectionError: If the data cannot be read or is invalid.
        :raises ProcessCancelledError: If ``cancel`` is set while waiting.
        """
