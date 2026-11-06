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
"""Simulation hooks of the barcode workplace node."""

from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from station_workplace.interface.simulation_hooks import SimulationHooks
from station_workplace.models.barcode_node_config_dto import BarcodeNodeConfigDTO
from station_workplace.models.product_data_dto import ProductDataDTO
from station_workplace.msg import LabelData
from station_workplace.utils.label_encoder import encode_label


class BarcodeSimulationHooks(SimulationHooks):
    """Publish the product label on ``simulated_label_data`` after spawning."""

    _SIMULATED_LABEL_QOS = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    def __init__(self, config: BarcodeNodeConfigDTO) -> None:
        """Store the topic name."""
        self._topic = config.simulated_label_data_topic
        self._node: Node | None = None
        self._publisher: Publisher | None = None

    def activate(self, node: Node) -> None:
        """Create the latched ``simulated_label_data`` publisher."""
        self._node = node
        self._publisher = node.create_publisher(LabelData, self._topic, self._SIMULATED_LABEL_QOS)

    def deactivate(self) -> None:
        """Destroy the publisher."""
        if self._node is not None and self._publisher is not None:
            self._node.destroy_publisher(self._publisher)
        self._publisher = None
        self._node = None

    def carrier_mappings(self, product: ProductDataDTO) -> dict[str, str]:
        """Validate that the label can be encoded; the carrier needs no extra arguments."""
        encode_label(product)
        return {}

    def after_spawn(self, product: ProductDataDTO) -> None:
        """Publish the product label once."""
        if self._node is None or self._publisher is None:
            raise RuntimeError('simulated label publisher is not active')
        msg = LabelData()
        msg.data = list(encode_label(product))
        msg.stamp = self._node.get_clock().now().to_msg()
        self._publisher.publish(msg)
        self._node.get_logger().info(f'Published simulated label on {self._topic}')
