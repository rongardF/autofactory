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
"""Product data reader for barcode labels."""

import re
import threading
from collections import deque

from pydantic import ValidationError
from rclpy.callback_groups import CallbackGroup
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.subscription import Subscription
from rclpy.time import Time

from station_workplace.exceptions.injection_error import InjectionError
from station_workplace.exceptions.process_cancelled_error import ProcessCancelledError
from station_workplace.interface.product_data_reader import ProductDataReader
from station_workplace.models.barcode_node_config_dto import BarcodeNodeConfigDTO
from station_workplace.models.product_data_dto import ProductDataDTO
from station_workplace.msg import LabelData
from station_workplace.utils.ascii_decoder import decode_label

_LABEL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
_WAIT_SLICE_SEC = 0.05


class BarcodeProductDataReader(ProductDataReader):
    """Take the product data from the first valid label read after the product arrived.

    Labels stamped after the arrival are queued and parsed in order; a label that
    cannot be decoded or from which the product data cannot be extracted is
    rejected and the next one is awaited. There is no timeout: the wait ends with
    a valid label or when the ejection cancels the injection. Once a valid label
    is accepted, later labels are ignored until the product leaves.
    """

    def __init__(self, config: BarcodeNodeConfigDTO, logger: RcutilsLogger) -> None:
        self._config = config
        self._logger = logger
        self._node: Node | None = None
        self._subscription: Subscription | None = None
        self._condition = threading.Condition()
        self._accepting = False
        self._arrival_ns: int | None = None
        self._pending: deque[bytes] = deque()
        # Incremented on every arrival/departure so a stale read cannot close acceptance.
        self._generation = 0

    def activate(self, node: Node, callback_group: CallbackGroup) -> None:
        """Subscribe to ``label_data``."""
        self._node = node
        self._subscription = node.create_subscription(
            LabelData, self._config.label_data_topic, self._on_label, _LABEL_QOS,
            callback_group=callback_group)

    def deactivate(self) -> None:
        """Destroy the subscription."""
        if self._node is not None and self._subscription is not None:
            self._node.destroy_subscription(self._subscription)
        self._subscription = None
        self._node = None

    def on_product_arrived(self, arrival_time: Time) -> None:
        """Start accepting labels stamped after ``arrival_time``."""
        with self._condition:
            self._generation += 1
            self._arrival_ns = arrival_time.nanoseconds
            self._pending.clear()
            self._accepting = True

    def on_product_left(self) -> None:
        """Stop accepting labels and forget the received ones."""
        with self._condition:
            self._generation += 1
            self._arrival_ns = None
            self._pending.clear()
            self._accepting = False
            self._condition.notify_all()

    def read(self, cancel: threading.Event) -> ProductDataDTO:
        """Wait (without timeout) for a valid label and extract the product data."""
        while True:
            data, generation = self._next_label(cancel)
            try:
                product = self._parse(data)
            except InjectionError as exc:
                self._logger.warning(f'Rejected barcode label: {exc}; waiting for another label')
                continue
            with self._condition:
                if generation == self._generation:
                    self._accepting = False
                    self._pending.clear()
            self._logger.info('Accepted barcode label')
            return product

    def _next_label(self, cancel: threading.Event) -> tuple[bytes, int]:
        """Block until a label is pending; pop it and return it with its generation."""
        with self._condition:
            while True:
                if cancel.is_set():
                    raise ProcessCancelledError('cancelled while waiting for the label')
                if self._pending:
                    return self._pending.popleft(), self._generation
                self._condition.wait(_WAIT_SLICE_SEC)

    def _on_label(self, msg: LabelData) -> None:
        stamp_ns = Time.from_msg(msg.stamp).nanoseconds
        with self._condition:
            if not self._accepting or self._arrival_ns is None or stamp_ns <= self._arrival_ns:
                return
            self._pending.append(bytes(msg.data))
            self._condition.notify_all()

    def _parse(self, data: bytes) -> ProductDataDTO:
        """Decode the label and extract the product data fields.

        :raises InjectionError: If decoding or extraction fails.
        """
        text = decode_label(data)
        patterns = self._config.label_patterns
        fields = {
            'product_number': self._extract('product_number', patterns.product_number, text),
            'product_revision': self._extract('product_revision', patterns.product_revision, text),
            'serial_number': self._extract('serial_number', patterns.serial_number, text),
        }
        try:
            return ProductDataDTO(**fields)
        except ValidationError as exc:
            raise InjectionError(f'invalid product data on label: {exc}') from exc

    @staticmethod
    def _extract(name: str, pattern: re.Pattern, text: str) -> str:
        match = pattern.search(text)
        if match is None:
            raise InjectionError(f"label does not match the {name} pattern '{pattern.pattern}'")
        value = (match.group(1) or '').strip()
        if not value:
            raise InjectionError(f'{name} extracted from the label is empty')
        return value
