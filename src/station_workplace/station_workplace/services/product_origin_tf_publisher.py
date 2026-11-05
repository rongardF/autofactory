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
"""Static TF publisher of the estimated product origin."""

import threading
from collections.abc import Sequence

from geometry_msgs.msg import Pose, TransformStamped
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

from station_workplace.enums.constants import ATTACH_LINK, PRODUCT_ORIGIN_FRAME
from station_workplace.utils.transformations import identity_pose, pose6_to_pose


class ProductOriginTfPublisher:
    """Publish ``estimated_product_origin`` relative to ``carrier`` on ``/tf_static``.

    The frame is not published before the first :meth:`publish_pose`; after
    that it is only ever updated (reset to identity), never removed.

    A plain latched publisher is used instead of ``StaticTransformBroadcaster``,
    which ignores updates of a child frame it has already sent.
    """

    TF_STATIC_QOS = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._node: Node | None = None
        self._publisher: Publisher | None = None
        self._ever_published = False

    def activate(self, node: Node) -> None:
        """Create the latched ``/tf_static`` publisher."""
        with self._lock:
            self._node = node
            self._publisher = node.create_publisher(TFMessage, '/tf_static', self.TF_STATIC_QOS)

    def deactivate(self) -> None:
        """Destroy the ``/tf_static`` publisher."""
        with self._lock:
            if self._node is not None and self._publisher is not None:
                self._node.destroy_publisher(self._publisher)
            self._publisher = None
            self._node = None

    def publish_pose(self, pose6: Sequence[float]) -> None:
        """Publish the frame at ``[x, y, z, roll, pitch, yaw]`` relative to the carrier."""
        self._send(pose6_to_pose(pose6))
        self._ever_published = True

    def reset_to_identity(self) -> None:
        """Publish the identity transform, if the frame was ever published."""
        if self._ever_published:
            self._send(identity_pose())

    def _send(self, pose: Pose) -> None:
        with self._lock:
            if self._publisher is None or self._node is None:
                raise RuntimeError('TF publisher is not active')
            transform = TransformStamped()
            transform.header.stamp = self._node.get_clock().now().to_msg()
            transform.header.frame_id = ATTACH_LINK
            transform.child_frame_id = PRODUCT_ORIGIN_FRAME
            transform.transform.translation.x = pose.position.x
            transform.transform.translation.y = pose.position.y
            transform.transform.translation.z = pose.position.z
            transform.transform.rotation = pose.orientation
            self._publisher.publish(TFMessage(transforms=[transform]))
