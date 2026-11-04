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

from threading import Lock

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.subscription import Subscription
from sensor_msgs.msg import LaserScan

from product_carrier.interface import CarrierController, PresenceChangedCallback
from product_carrier.model import CarrierActionResultDTO, ConveyorNodeConfigDTO


class SimulatedConveyorController(CarrierController):
    """Conveyor backend driven by a Gazebo presence lidar.

    Product presence is derived from a single-ray lidar mounted across the
    carrier path at the workplace slot: any in-range return means a product is
    in place. Inject and eject are no-ops because products are moved manually
    in the simulator.
    """
    PRESENCE_SENSOR_TOPIC = 'presence_sensor_scan'

    def __init__(self, node: Node, config: ConveyorNodeConfigDTO) -> None:
        super().__init__(node, config)
        self._lock: Lock = Lock()
        self._subscription: Subscription | None = None
        self._on_presence_changed: PresenceChangedCallback | None = None
        self._in_place = False

    def setup(self, on_presence_changed: PresenceChangedCallback) -> None:
        with self._lock:
            self._on_presence_changed = on_presence_changed
            self._in_place = False
        self._subscription = self._node.create_subscription(
            LaserScan,
            self.PRESENCE_SENSOR_TOPIC,
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self._node.get_logger().info(
            f"Simulated conveyor ready, listening for presence scans on '{self._subscription.topic_name}'"
        )

    def teardown(self) -> None:
        if self._subscription is not None:
            self._node.destroy_subscription(self._subscription)
            self._subscription = None
        with self._lock:
            self._on_presence_changed = None
            self._in_place = False

    def inject_product(self) -> CarrierActionResultDTO:
        return CarrierActionResultDTO(
            success=True,
            message='Simulation: inject_product is a no-op; move the product manually in Gazebo.',
        )

    def eject_product(self) -> CarrierActionResultDTO:
        return CarrierActionResultDTO(
            success=True,
            message='Simulation: eject_product is a no-op; move the product manually in Gazebo.',
        )

    def product_in_place(self) -> bool:
        with self._lock:
            return self._in_place

    def _scan_callback(self, scan: LaserScan) -> None:
        # inf/nan ("no return") fail the comparison and count as empty.
        in_place = any(scan.range_min <= r <= scan.range_max for r in scan.ranges)
        with self._lock:
            if in_place == self._in_place:
                return
            self._in_place = in_place
            callback = self._on_presence_changed
        if callback is not None:
            callback(in_place)
