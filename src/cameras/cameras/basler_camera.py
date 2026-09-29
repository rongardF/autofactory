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

Subscribes to:
    * ``simulated_camera/image_raw``   (``sensor_msgs/Image``)
    * ``simulated_camera/camera_info`` (``sensor_msgs/CameraInfo``)

Republishes to:
    * ``<node-name>/image_raw``   (``sensor_msgs/Image``)
    * ``<node-name>/camera_info`` (``sensor_msgs/CameraInfo``)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo, Image


class BaslerCamera(Node):
    """Republishes a simulated Gazebo camera under the real camera's topics."""

    def __init__(self, node_name: str = 'basler_camera') -> None:
        """Set up subscriptions to the simulated camera and republishers."""
        super().__init__(node_name)

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
        """Republish an incoming simulated image frame."""
        self._image_pub.publish(msg)

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """Republish incoming simulated camera calibration info."""
        self._camera_info_pub.publish(msg)


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