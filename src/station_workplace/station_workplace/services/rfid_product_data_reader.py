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
"""Product data reader for RFID tags."""

import threading
import time

from pydantic import ValidationError
from rclpy.callback_groups import CallbackGroup
from rclpy.client import Client
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.subscription import Subscription
from std_msgs.msg import Bool

from rfid_scanner.srv import ReadData
from station_workplace.exceptions.injection_error import InjectionError
from station_workplace.exceptions.process_cancelled_error import ProcessCancelledError
from station_workplace.interface.product_data_reader import ProductDataReader
from station_workplace.models.rfid_node_config_dto import RfidNodeConfigDTO, TagRegisterDTO
from station_workplace.models.product_data_dto import ProductDataDTO
from station_workplace.utils.ascii_decoder import decode_rfid_field
from station_workplace.utils.async_service_caller import call_service


class RfidProductDataReader(ProductDataReader):
    """Read the product data fields from the RFID tag on the carrier."""

    _LATCHED_QOS = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    _WAIT_SLICE_SEC = 0.05

    def __init__(self, config: RfidNodeConfigDTO) -> None:
        self._config = config
        self._node: Node | None = None
        self._subscription: Subscription | None = None
        self._client: Client | None = None
        self._condition = threading.Condition()
        self._tag_detected = False

    def activate(self, node: Node, callback_group: CallbackGroup) -> None:
        """Subscribe to ``tag_detected`` and create the ``read_data`` client."""
        self._node = node
        with self._condition:
            self._tag_detected = False
        self._subscription = node.create_subscription(
            Bool, self._config.tag_detected_topic, self._on_tag_detected, self._LATCHED_QOS,
            callback_group=callback_group)
        self._client = node.create_client(
            ReadData, self._config.read_data_service, callback_group=callback_group)

    def deactivate(self) -> None:
        """Destroy the subscription and the client."""
        if self._node is not None:
            if self._subscription is not None:
                self._node.destroy_subscription(self._subscription)
            if self._client is not None:
                self._node.destroy_client(self._client)
        self._subscription = None
        self._client = None
        self._node = None

    def read(self, cancel: threading.Event) -> ProductDataDTO:
        """Wait for a detected tag and read the three product data fields."""
        self._wait_for_tag(cancel)
        registers = self._config.tag_registers
        fields = {
            'product_number': self._read_field('product_number', registers.product_number, cancel),
            'product_revision': self._read_field(
                'product_revision', registers.product_revision, cancel),
            'serial_number': self._read_field('serial_number', registers.serial_number, cancel),
        }
        try:
            return ProductDataDTO(**fields)
        except ValidationError as exc:
            raise InjectionError(f'invalid product data on tag: {exc}') from exc

    def _on_tag_detected(self, msg: Bool) -> None:
        with self._condition:
            self._tag_detected = bool(msg.data)
            self._condition.notify_all()

    def _wait_for_tag(self, cancel: threading.Event) -> None:
        timeout = self._config.tag_detected_timeout_sec
        deadline = time.monotonic() + timeout
        with self._condition:
            while not self._tag_detected:
                if cancel.is_set():
                    raise ProcessCancelledError('cancelled while waiting for the RFID tag')
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise InjectionError(f'RFID tag not detected within {timeout} s')
                self._condition.wait(min(self._WAIT_SLICE_SEC, remaining))

    def _read_field(
        self, name: str, register: TagRegisterDTO, cancel: threading.Event,
    ) -> str:
        if self._client is None:
            raise InjectionError('RFID reader is not active')
        request = ReadData.Request(start_address=register.start_address, length=register.length)
        response = call_service(
            self._client, request, self._config.service_call_timeout_sec, cancel,
            InjectionError, f'read {name} from RFID tag')
        if not response.success:
            raise InjectionError(f'reading {name} from RFID tag failed: {response.error_message}')
        return decode_rfid_field(bytes(response.data), name)
