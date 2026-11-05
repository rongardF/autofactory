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
"""Publisher of the latched ``~/workplace`` state."""

import threading

from rclpy.lifecycle import LifecycleNode, LifecyclePublisher
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from station_workplace.models.workplace_state_dto import WorkplaceStateDTO
from station_workplace.msg import WorkplaceState


class WorkplaceStatePublisher:
    """Publish the workplace state only when it differs from the last published one.

    The last published state survives deactivate/activate cycles and is only
    forgotten by :meth:`reset` (cleanup/shutdown).
    """

    WORKPLACE_QOS = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._node: LifecycleNode | None = None
        self._publisher: LifecyclePublisher | None = None
        self._last_published: WorkplaceStateDTO | None = None

    def activate(self, node: LifecycleNode) -> None:
        """Create the lifecycle publisher (enabled by the node's ``on_activate``)."""
        with self._lock:
            self._node = node
            self._publisher = node.create_lifecycle_publisher(  # type: ignore[assignment]
                WorkplaceState, '~/workplace', self.WORKPLACE_QOS)

    def deactivate(self) -> None:
        """Destroy the publisher; its latched sample is lost."""
        with self._lock:
            if self._node is not None and self._publisher is not None:
                self._node.destroy_lifecycle_publisher(self._publisher)
            self._publisher = None
            self._node = None

    def reset(self) -> None:
        """Forget the last published state."""
        with self._lock:
            self._last_published = None

    def publish(self, state: WorkplaceStateDTO) -> bool:
        """Publish ``state`` unless it equals the last published state.

        :returns: True if a message was published.
        :raises RuntimeError: If the publisher does not exist.
        """
        with self._lock:
            if state == self._last_published:
                return False
            if self._publisher is None or self._node is None:
                raise RuntimeError('workplace publisher is not active')
            self._publisher.publish(state.to_msg())
            self._last_published = state
            self._node.get_logger().info(f'Published workplace state: {state}')
            return True
