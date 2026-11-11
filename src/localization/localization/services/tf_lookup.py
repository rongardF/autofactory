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
"""TF buffer and listener used to look up and apply transforms."""

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PoseStamped, TransformStamped

from localization.exceptions import TransformError


class TfLookup:
    """Looks up the latest available transforms; relies on the node's executor for TF data."""

    def __init__(self, node: Node, timeout_s: float) -> None:
        """:param timeout_s: Time to wait for a transform to become available."""
        self._timeout_s = timeout_s
        self._buffer = Buffer()
        self._listener: TransformListener | None = TransformListener(self._buffer, node)

    def lookup(self, target_frame: str, source_frame: str) -> TransformStamped:
        """Return the pose of ``source_frame`` expressed in ``target_frame``.

        :raises TransformError: If the transform is not available in time.
        """
        try:
            return self._buffer.lookup_transform(
                target_frame, source_frame, Time(), timeout=Duration(seconds=self._timeout_s))
        except TransformException as exc:
            raise TransformError(
                f"transform '{target_frame}' <- '{source_frame}' is not available: {exc}") from exc

    def transform_pose(self, pose: PoseStamped, target_frame: str) -> PoseStamped:
        """Express ``pose`` in ``target_frame`` using the latest transform.

        :raises TransformError: If the transform is not available in time.
        """
        if pose.header.frame_id == target_frame:
            return pose
        return do_transform_pose_stamped(pose, self.lookup(target_frame, pose.header.frame_id))

    def teardown(self) -> None:
        """Destroy the TF subscriptions."""
        if self._listener is not None:
            self._listener.unregister()
            self._listener = None
