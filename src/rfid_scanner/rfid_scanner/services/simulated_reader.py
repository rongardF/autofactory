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
"""Reader backed by the Gazebo ``RfidReader`` system plugin.

Detection and tag content come from the plugin's topics, bridged into ROS 2.
Writes go directly to the plugin's gz-transport write service.
"""

import threading
from typing import Callable

from gz.transport13 import Node as GzNode
from gz.msgs10.dataframe_pb2 import Dataframe as GzDataframe  # type: ignore[attr-defined]; not compiled yet
from gz.msgs10.stringmsg_pb2 import StringMsg  # type: ignore[attr-defined]

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.lifecycle import LifecycleNode
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.subscription import Subscription
from ros_gz_interfaces.msg import Dataframe as RosDataframe
from std_msgs.msg import Bool

from rfid_scanner.exceptions.command_timeout_error import CommandTimeoutError
from rfid_scanner.exceptions.reader_fatal_error import ReaderFatalError
from rfid_scanner.exceptions.tag_access_error import TagAccessError
from rfid_scanner.interface.rfid_reader import RfidReader
from rfid_scanner.models.node_config_dto import NodeConfigDTO
from rfid_scanner.models.tag_info_dto import TagInfoDTO


class SimulatedReader(RfidReader):
    """Reader driven by the Gazebo RFID simulation."""

    _BRIDGE_QOS = QoSProfile(
        depth=10,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
    )

    def __init__(
        self,
        node: LifecycleNode,
        config: NodeConfigDTO,
        on_detected_changed: Callable[[bool], None],
        on_fatal_error: Callable[[str], None],
    ) -> None:
        """Store collaborators; no resources are acquired here."""
        super().__init__(node, config, on_detected_changed, on_fatal_error)
        self._write_lock = threading.Lock()
        self._detected = False
        self._tag_id: str | None = None
        self._payload: bytearray | None = None
        self._subscriptions: list[Subscription] = []
        self._gz_node = None

    def initialize(self) -> None:
        """Subscribe to the bridged topics and create the gz-transport node."""
        with self._state_lock:
            self._clear_state()
            self._fatal_reported = False

        group = MutuallyExclusiveCallbackGroup()
        self._subscriptions.append(self._node.create_subscription(
            Bool, self._config.detected_topic, self._on_detected, self._BRIDGE_QOS,
            callback_group=group))
        self._subscriptions.append(self._node.create_subscription(
            RosDataframe, self._config.tag_data_topic, self._on_tag_data, self._BRIDGE_QOS,
            callback_group=group))

        try:
            self._gz_node = GzNode()
        except Exception as e:
            raise ReaderFatalError(f'failed to create gz-transport node: {e}') from e

        with self._state_lock:
            self._fatal_reporting_enabled = True
        self._logger.info(
            f"Simulated reader ready: detected '{self._config.detected_topic}', "
            f"tag data '{self._config.tag_data_topic}', "
            f"write service '{self._config.write_service}'")

    def deinitialize(self) -> None:
        """Destroy subscriptions, drop the gz node and clear the state."""
        with self._state_lock:
            self._fatal_reporting_enabled = False
        for sub in self._subscriptions:
            try:
                self._node.destroy_subscription(sub)
            except Exception as e:
                self._logger.warning(f'Failed to destroy subscription: {e}')
        self._subscriptions.clear()
        self._gz_node = None
        with self._state_lock:
            self._clear_state()

    @property
    def is_tag_detected(self) -> bool:
        """Return the state of the Gazebo detection topic."""
        with self._state_lock:
            return self._detected

    @property
    def tag_info(self) -> TagInfoDTO | None:
        """Return the detected tag once its payload has been received."""
        with self._state_lock:
            if not self._detected or self._tag_id is None or self._payload is None:
                return None
            return TagInfoDTO(tag_id=self._tag_id, memory_size=len(self._payload))

    def _clear_state(self) -> None:
        """Forget the detection and cached tag (caller holds the state lock)."""
        self._detected = False
        self._tag_id = None
        self._payload = None

    def _on_detected(self, msg: Bool) -> None:
        """Track the Gazebo detection flag."""
        try:
            with self._state_lock:
                detected = bool(msg.data)
                if detected == self._detected:
                    return
                self._detected = detected
                if not detected:
                    self._tag_id = None
                    self._payload = None
                self._on_detected_changed(detected)
        except Exception as e:
            self._report_fatal(f'unexpected error handling detection: {e}')

    def _on_tag_data(self, msg: RosDataframe) -> None:
        """Cache the id and payload of the detected tag."""
        try:
            with self._state_lock:
                if not msg.src_address:
                    self._tag_id = None
                    self._payload = None
                    return
                self._tag_id = msg.src_address
                self._payload = bytearray(msg.data)
        except Exception as e:
            self._report_fatal(f'unexpected error handling tag data: {e}')

    def _read_impl(self, tag: TagInfoDTO, start_address: int, length: int) -> bytes:
        """Slice the cached payload."""
        with self._state_lock:
            if self._payload is None or self._tag_id != tag.tag_id:
                raise TagAccessError('tag changed during request')
            return bytes(self._payload[start_address:start_address + length])

    def _write_impl(self, tag: TagInfoDTO, start_address: int, data: bytes) -> None:
        """Write through the Gazebo write service and update the cache."""
        try:
            with self._write_lock:
                self._write_via_gz(tag, start_address, data)
        except TagAccessError:
            raise
        except Exception as e:
            reason = f'unexpected error during simulated write: {e}'
            self._report_fatal(reason)
            raise ReaderFatalError(reason) from e

    def _write_via_gz(self, tag: TagInfoDTO, start_address: int, data: bytes) -> None:
        """Call the gz write service (caller holds the write lock)."""
        gz_node = self._gz_node
        if gz_node is None:
            raise TagAccessError('reader not initialised')
        service = self._config.write_service
        if not gz_node.service_info(service):
            raise TagAccessError(f'write service {service} not advertised')

        request = GzDataframe()
        request.src_address = tag.tag_id
        request.dst_address = str(start_address)
        request.data = bytes(data)
        timeout_ms = int(self._config.response_timeout_sec * 1000)
        result, response = gz_node.request(
            service, request, GzDataframe, StringMsg, timeout_ms)
        if not result:
            raise CommandTimeoutError('gz write service call failed or timed out')
        if response.data:
            raise TagAccessError(response.data)

        # Optimistic cache update so an immediate read returns the new bytes.
        with self._state_lock:
            if self._tag_id == tag.tag_id and self._payload is not None:
                self._payload[start_address:start_address + len(data)] = data
