#!/usr/bin/env python3
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
    * ``<node-name>/image``       (``sensor_msgs/Image``, undistorted)
    * ``<node-name>/camera_info`` (``sensor_msgs/CameraInfo``)

Serves:
    * ``<node-name>/get_image``     (``cameras/action/GetImage``, undistorted image)
    * ``<node-name>/get_image_raw`` (``cameras/action/GetImage``, distorted image)

Publishes the static TF ``camera_tcp_uncalibrated`` -> ``camera_tcp_calibrated``.
"""

import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import StaticTransformBroadcaster

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image

from cameras.action import GetImage


class BaslerCamera(Node):
    """Republishes a simulated Gazebo camera under the real camera's topics."""

    CAMERA_TCP_UNCALIBRATED_FRAME = 'camera_tcp_uncalibrated'
    CAMERA_TCP_CALIBRATED_FRAME = 'camera_tcp_calibrated'

    def __init__(self, node_name: str = 'basler_camera') -> None:
        """Set up subscriptions to the simulated camera, republishers and actions."""
        super().__init__(node_name)

        self.declare_parameter(
            'get_image_timeout_s',
            5.0,
            ParameterDescriptor(
                description='Seconds a get_image / get_image_raw goal waits for a new frame.'
            ),
        )
        self._get_image_timeout_s: float = (
            self.get_parameter('get_image_timeout_s').get_parameter_value().double_value
        )
        if self._get_image_timeout_s <= 0.0:
            raise ValueError('get_image_timeout_s must be greater than 0')

        self._bridge = CvBridge()
        self._group = ReentrantCallbackGroup()
        # Frames must be processed in order, so the subscriptions share a serial group.
        self._sub_group = MutuallyExclusiveCallbackGroup()

        # Latest frames and a frame counter; get_image goals wait for the
        # counter to advance so the returned frame is newer than the request.
        self._frame_cond = threading.Condition()
        self._frame_seq: int = 0
        self._latest_undistorted: Image | None = None
        self._latest_raw: Image | None = None
        self._latest_camera_info: CameraInfo | None = None
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
        self._undistorted_pub = self.create_publisher(
            Image,
            f'{self.get_name()}/image',
            qos_profile_sensor_data
        )
        self._camera_info_pub = self.create_publisher(
            CameraInfo, camera_info_target, qos_profile_sensor_data
        )

        self.create_subscription(
            Image,
            'simulated_camera/image_raw',
            self._image_callback,
            qos_profile_sensor_data,
            callback_group=self._sub_group,
        )
        self.create_subscription(
            CameraInfo,
            'simulated_camera/camera_info',
            self._camera_info_callback,
            qos_profile_sensor_data,
            callback_group=self._sub_group,
        )

        self._get_image_server = ActionServer(
            self,
            GetImage,
            f'{self.get_name()}/get_image',
            execute_callback=lambda goal_handle: self._execute_get_image(goal_handle, raw=False),
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
            callback_group=self._group,
        )
        self._get_image_raw_server = ActionServer(
            self,
            GetImage,
            f'{self.get_name()}/get_image_raw',
            execute_callback=lambda goal_handle: self._execute_get_image(goal_handle, raw=True),
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
            callback_group=self._group,
        )

        # NOTE: identity for now; replace with the result of the camera calibration.
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        calibrated_tf = TransformStamped()
        calibrated_tf.header.stamp = self.get_clock().now().to_msg()
        calibrated_tf.header.frame_id = self.CAMERA_TCP_UNCALIBRATED_FRAME
        calibrated_tf.child_frame_id = self.CAMERA_TCP_CALIBRATED_FRAME
        calibrated_tf.transform.rotation.w = 1.0
        self._static_tf_broadcaster.sendTransform(calibrated_tf)

        self.get_logger().info(
            f"Simulated Basler camera ready: "
            f"'simulated_camera/image_raw' -> '{image_target}', "
            f"'simulated_camera/camera_info' -> '{camera_info_target}'"
        )

    def _image_callback(self, msg: Image) -> None:
        """Apply lens distortion to an incoming frame, then republish it."""
        out = msg
        if self._map1 is not None:
            try:
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                distorted = cv2.remap(frame, self._map1, self._map2, cv2.INTER_LINEAR)
                out = self._bridge.cv2_to_imgmsg(distorted, encoding=msg.encoding)
                out.header = msg.header
            except Exception as e:  # noqa: BLE001 - never drop the stream on a bad frame
                self.get_logger().error(f'Distortion failed, forwarding raw frame: {e}')
                out = msg

        with self._frame_cond:
            self._latest_undistorted = msg
            self._latest_raw = out
            self._frame_seq += 1
            self._frame_cond.notify_all()

        self._image_pub.publish(out)
        self._undistorted_pub.publish(msg)

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """Republish calibration info and (re)build distortion maps as needed."""
        with self._frame_cond:
            self._latest_camera_info = msg
        self._camera_info_pub.publish(msg)
        self._update_distortion_maps(msg)

    def _execute_get_image(self, goal_handle, raw: bool) -> GetImage.Result:
        """Return the first frame received after the goal started executing."""
        result = GetImage.Result()
        deadline = time.monotonic() + self._get_image_timeout_s

        with self._frame_cond:
            start_seq = self._frame_seq
            while self._frame_seq <= start_seq:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.error_message = 'cancelled'
                    return result
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._frame_cond.wait(min(remaining, 0.1))

            frame_ready = self._frame_seq > start_seq
            image = self._latest_raw if raw else self._latest_undistorted
            camera_info = self._latest_camera_info

        if not frame_ready or image is None:
            result.error_message = (
                f'No new camera frame within {self._get_image_timeout_s} s'
            )
        elif camera_info is None:
            result.error_message = 'No camera info received yet'
        else:
            result.image = image
            result.camera_info = camera_info
            result.success = True
            goal_handle.succeed()
            return result

        self.get_logger().error(result.error_message)
        goal_handle.abort()
        return result

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
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()