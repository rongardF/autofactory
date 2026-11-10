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
"""Tracks the workplace state published by the station workplace node."""

import threading
from collections.abc import Callable

from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.subscription import Subscription

from station_workplace.models.product_data_dto import ProductDataDTO
from station_workplace.models.workplace_state_dto import WorkplaceStateDTO
from station_workplace.msg import WorkplaceState


class WorkplaceMonitor:
    """Keeps the latest workplace state and reports when a localization stops being valid."""

    _QOS = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    def __init__(self, on_invalid: Callable[[], None]) -> None:
        """:param on_invalid: Called (once per baseline) when the state differs from the baseline."""
        self._on_invalid = on_invalid
        self._lock = threading.Lock()
        self._latest: WorkplaceStateDTO | None = None
        self._baseline: WorkplaceStateDTO | None = None
        self._node: Node | None = None
        self._subscription: Subscription | None = None

    def activate(self, node: Node, topic: str, callback_group: CallbackGroup) -> None:
        """Subscribe to the workplace state."""
        self._node = node
        self._subscription = node.create_subscription(
            WorkplaceState, topic, self._on_state, self._QOS, callback_group=callback_group)

    def deactivate(self) -> None:
        """Destroy the subscription and forget the state."""
        if self._node is not None and self._subscription is not None:
            self._node.destroy_subscription(self._subscription)
        self._subscription = None
        self._node = None
        with self._lock:
            self._latest = None
            self._baseline = None

    @property
    def latest(self) -> WorkplaceStateDTO | None:
        """Latest received state, or None."""
        with self._lock:
            return self._latest

    def set_baseline(self, state: WorkplaceStateDTO | None) -> None:
        """Set the state a localization is valid for; None disarms the monitor."""
        with self._lock:
            self._baseline = state

    def _on_state(self, msg: WorkplaceState) -> None:
        product = msg.product_data[0] if msg.product_data else None
        state = WorkplaceStateDTO(
            in_place=bool(msg.in_place),
            product_data=None if product is None else ProductDataDTO(
                product_number=product.product_number,
                product_revision=product.product_revision,
                serial_number=product.serial_number,
            ),
        )
        with self._lock:
            self._latest = state
            baseline = self._baseline
            # A localization is void once the product is gone or a different one is in place.
            invalid = baseline is not None and not (
                state.in_place and state.product_data == baseline.product_data)
            if invalid:
                self._baseline = None
        if invalid:
            self._on_invalid()
