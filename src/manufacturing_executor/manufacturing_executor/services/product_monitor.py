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
"""Detects that the product left the station unexpectedly."""

import threading
from collections.abc import Callable

from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.subscription import Subscription

from manufacturing_executor.utils.node_names import topic
from station_workplace.models.product_data_dto import ProductDataDTO
from station_workplace.models.workplace_state_dto import WorkplaceStateDTO
from station_workplace.msg import WorkplaceState


class ProductMonitor:
    """Watches ``~/workplace`` of the station workplace while a product is armed."""

    _QOS = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    def __init__(self, node: Node, workplace_node_name: str, callback_group: CallbackGroup) -> None:
        self._node = node
        self._lock = threading.Lock()
        self._product: ProductDataDTO | None = None
        self._on_lost: Callable[[str], None] | None = None
        self._subscription: Subscription | None = node.create_subscription(
            WorkplaceState,
            topic(workplace_node_name, 'workplace'),
            self._on_state,
            self._QOS,
            callback_group=callback_group,
        )

    def arm(self, product: ProductDataDTO, on_lost: Callable[[str], None]) -> None:
        """Call ``on_lost`` (once) when the product is gone or replaced; messages before this call are ignored."""
        with self._lock:
            self._product = product
            self._on_lost = on_lost

    def disarm(self) -> None:
        """Stop watching (the expected departure of the product starts)."""
        with self._lock:
            self._product = None
            self._on_lost = None

    def destroy(self) -> None:
        """Disarm and destroy the subscription."""
        self.disarm()
        if self._subscription is not None:
            self._node.destroy_subscription(self._subscription)
            self._subscription = None

    def _on_state(self, message: WorkplaceState) -> None:
        with self._lock:
            expected = self._product
            on_lost = self._on_lost
            if expected is None or on_lost is None:
                return
            state = WorkplaceStateDTO.from_msg(message)
            if state.in_place and state.product_data == expected:
                return
            self._product = None
            self._on_lost = None
        on_lost('Product left the station unexpectedly')
