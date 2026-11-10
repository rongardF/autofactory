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
"""Publishes frames as dynamic transforms for as long as they are valid."""

import threading

from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.timer import Timer
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import Pose, TransformStamped

from localization.enums.constants import TF_PUBLISH_PERIOD_S


class TfFramePublisher:
    """Republishes the active frames on ``/tf`` periodically.

    TF cannot remove a frame: clearing a frame only stops publishing it, so it
    ages out of the listeners' buffers.
    """

    def __init__(self, node: Node, callback_group: CallbackGroup) -> None:
        """Create the broadcaster and the publishing timer."""
        self._node = node
        self._lock = threading.Lock()
        self._frames: dict[str, tuple[str, Pose]] = {}
        self._broadcaster = TransformBroadcaster(node)
        self._timer: Timer | None = node.create_timer(
            TF_PUBLISH_PERIOD_S, self._publish, callback_group=callback_group)

    def set_frame(self, child_frame: str, parent_frame: str, pose: Pose) -> None:
        """Start (or update) publishing ``child_frame`` at ``pose`` relative to ``parent_frame``."""
        with self._lock:
            self._frames[child_frame] = (parent_frame, pose)
        self._publish()

    def clear_frame(self, child_frame: str) -> None:
        """Stop publishing ``child_frame``."""
        with self._lock:
            self._frames.pop(child_frame, None)

    def teardown(self) -> None:
        """Stop publishing and destroy the timer."""
        with self._lock:
            self._frames.clear()
        if self._timer is not None:
            self._node.destroy_timer(self._timer)
            self._timer = None

    def _publish(self) -> None:
        with self._lock:
            frames = dict(self._frames)
        if not frames:
            return
        stamp = self._node.get_clock().now().to_msg()
        transforms = []
        for child, (parent, pose) in frames.items():
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = parent
            transform.child_frame_id = child
            transform.transform.translation.x = pose.position.x
            transform.transform.translation.y = pose.position.y
            transform.transform.translation.z = pose.position.z
            transform.transform.rotation = pose.orientation
            transforms.append(transform)
        self._broadcaster.sendTransform(transforms)
