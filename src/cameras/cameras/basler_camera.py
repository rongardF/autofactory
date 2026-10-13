#!/usr/bin/env python3
# Copyright (c) 2026, Movement Controller Contributors
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
"""Simulated Basler camera node.

Bridges a Gazebo-provided camera into the same topic layout a real Basler
(``pylon_ros2_camera_wrapper``) node exposes. It subscribes to the simulated
camera streams and republishes them under the node's own name, so downstream
consumers see identical topics whether the camera is real or simulated.

Gazebo renders an ideal pinhole image, but its ogre2 render engine never
applies the ``<distortion>`` lens model to the pixels -- it only exports the
coefficients to ``CameraInfo``. To match a real Basler, this node bakes the
Brown-Conrady (plumb_bob) distortion from ``CameraInfo`` into ``image_raw``.

Subscribes to:
    * ``simulated_camera/image_raw``   (``sensor_msgs/Image``)
    * ``simulated_camera/camera_info`` (``sensor_msgs/CameraInfo``)

Republishes to:
    * ``<node-name>/image_raw``   (``sensor_msgs/Image``, distortion applied)
    * ``<node-name>/camera_info`` (``sensor_msgs/CameraInfo``)
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo, Image


class BaslerCamera(Node):
    """Republishes a simulated Gazebo camera under the real camera's topics."""

    def __init__(self, node_name: str = 'basler_camera') -> None:
        """Set up subscriptions to the simulated camera and republishers."""
        super().__init__(node_name)

        self._bridge = CvBridge()
        # Cached cv2.remap maps that bake lens distortion into each frame, plus
        # the (w, h, K, D) signature they were built from so they are only
        # rebuilt when the calibration actually changes.
        self._map1: np.ndarray | None = None
        self._map2: np.ndarray | None = None
        self._calib_signature: tuple | None = None

        # Republish under the node's own name so real and simulated cameras
        # expose an identical topic layout to downstream consumers.
        image_target: str = f'{self.get_name()}/image_raw'
        camera_info_target: str = f'{self.get_name()}/camera_info'

        self._image_pub = self.create_publisher(
            Image, image_target, qos_profile_sensor_data
        )
        self._camera_info_pub = self.create_publisher(
            CameraInfo, camera_info_target, qos_profile_sensor_data
        )

        self.create_subscription(
            Image, 'simulated_camera/image_raw', self._image_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            'simulated_camera/camera_info',
            self._camera_info_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Simulated Basler camera ready: "
            f"'simulated_camera/image_raw' -> '{image_target}', "
            f"'simulated_camera/camera_info' -> '{camera_info_target}'"
        )

    def _image_callback(self, msg: Image) -> None:
        """Apply lens distortion to an incoming frame, then republish it."""
        if self._map1 is None:
            # No calibration yet, or distortion disabled: pass through untouched.
            self._image_pub.publish(msg)
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            distorted = cv2.remap(frame, self._map1, self._map2, cv2.INTER_LINEAR)
            out = self._bridge.cv2_to_imgmsg(distorted, encoding=msg.encoding)
            out.header = msg.header
            self._image_pub.publish(out)
        except Exception as e:  # noqa: BLE001 - never drop the stream on a bad frame
            self.get_logger().error(f'Distortion failed, forwarding raw frame: {e}')
            self._image_pub.publish(msg)

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """Republish calibration info and (re)build distortion maps as needed."""
        self._camera_info_pub.publish(msg)
        self._update_distortion_maps(msg)

    def _update_distortion_maps(self, msg: CameraInfo) -> None:
        """Rebuild the cached remap grid when the calibration changes."""
        signature = (msg.width, msg.height, tuple(msg.k), tuple(msg.d))
        if signature == self._calib_signature:
            return
        self._calib_signature = signature

        d = np.asarray(msg.d, dtype=np.float64)
        if d.size == 0 or not np.any(d):
            # No distortion to apply; forward frames unchanged.
            self._map1 = None
            self._map2 = None
            self.get_logger().info('CameraInfo has no distortion; forwarding raw frames.')
            return

        k = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        # Backward map: for each output (distorted) pixel, sample the ideal
        # Gazebo image at its undistorted location, so cv2.undistort(image, k, d)
        # recovers the original pinhole frame -- i.e. a real Basler's optics.
        map_x, map_y = cv2.initInverseRectificationMap(
            k, d, np.eye(3), k, (msg.width, msg.height), cv2.CV_32FC1
        )
        self._map1, self._map2 = cv2.convertMaps(map_x, map_y, cv2.CV_16SC2)
        self.get_logger().info(
            f'Distortion maps built for {msg.width}x{msg.height} (plumb_bob).'
        )


def main(args: list[str] | None = None) -> None:
    """Spin the simulated Basler camera node until shutdown."""
    rclpy.init(args=args)
    node = BaslerCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()