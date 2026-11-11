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
"""Mock localization: real robot and camera work, but no image processing."""

import threading
import time
from collections.abc import Callable

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from station_workplace.models.workplace_state_dto import WorkplaceStateDTO

from localization.enums.constants import (
    ESTIMATED_PRODUCT_ORIGIN_FRAME,
    LOCALIZATION_ORIGIN_FRAME,
    LOCALIZATION_ORIGIN_PLACEHOLDER_POSE,
    MOCK_CALCULATION_DURATION_S,
    PLANNING_FRAME,
    PRODUCT_ORIGIN_FRAME,
    TF_LOOKUP_TIMEOUT_S,
)
from localization.exceptions import LocalizationCancelledError, LocalizationError
from localization.interface.localization_service import LocalizationService
from localization.models.localization_poses_dto import LocalizationPosesDTO
from localization.models.localization_result_dto import (
    LocalizationRequestDTO,
    LocalizationResultDTO,
)
from localization.models.localizer_config_dto import LocalizerConfigDTO
from localization.services.camera_client import CameraClient
from localization.services.motion_client import MotionClient
from localization.services.tf_frame_publisher import TfFramePublisher
from localization.services.tf_lookup import TfLookup
from localization.services.workplace_monitor import WorkplaceMonitor
from localization.utils.async_action import wait_cancellable
from localization.utils.pose_conversion import pose6_to_pose

WORKPLACE_CHANGED_MESSAGE = 'product left or changed in the workplace during localization'
NO_PRODUCT_MESSAGE = 'no product in place in the workplace'


class MockLocalizationService(LocalizationService):
    """Moves the camera and captures the images, then waits instead of calculating the pose.

    The localized pose is the one estimated by the station workplace.
    """

    def __init__(self, config: LocalizerConfigDTO, poses: LocalizationPosesDTO) -> None:
        """Store the configuration; nothing is created until :meth:`initialize`."""
        self._config = config
        self._poses = poses
        self._node: Node | None = None
        self._on_invalidated: Callable[[], None] = lambda: None
        self._monitor: WorkplaceMonitor | None = None
        self._tf_publisher: TfFramePublisher | None = None
        self._tf_lookup: TfLookup | None = None
        self._motion: MotionClient | None = None
        self._camera: CameraClient | None = None
        self._lock = threading.Lock()
        self._run_cancel_event: threading.Event | None = None
        self._workplace_changed = False

    def initialize(self, node: Node, on_invalidated: Callable[[], None]) -> None:
        """Create the TF, workplace, movement and camera interfaces."""
        self._node = node
        self._on_invalidated = on_invalidated
        io_group = ReentrantCallbackGroup()
        tf_group = ReentrantCallbackGroup()
        self._monitor = WorkplaceMonitor(self._on_workplace_invalid)
        self._monitor.activate(node, self._config.workplace_topic, io_group)
        self._tf_publisher = TfFramePublisher(node, tf_group)
        self._tf_lookup = TfLookup(node, TF_LOOKUP_TIMEOUT_S)
        self._motion = MotionClient(
            node, self._config.execute_trajectory_action, self._config.camera_frame_id,
            self._tf_lookup, io_group)
        self._camera = CameraClient(
            node, self._config.get_image_action, self._config.image_capture_timeout_s, io_group)

    def teardown(self) -> None:
        """Stop publishing and destroy all interfaces."""
        if self._tf_publisher is not None:
            self._tf_publisher.teardown()
        if self._monitor is not None:
            self._monitor.deactivate()
        if self._tf_lookup is not None:
            self._tf_lookup.teardown()
        if self._motion is not None:
            self._motion.destroy()
        if self._camera is not None:
            self._camera.destroy()
        self._monitor = None
        self._tf_publisher = None
        self._tf_lookup = None
        self._motion = None
        self._camera = None
        self._node = None

    def workplace_state(self) -> WorkplaceStateDTO | None:
        """Return the latest workplace state, or None if none was received yet."""
        return self._monitor.latest if self._monitor is not None else None

    def invalidate(self) -> None:
        """Stop publishing ``product_origin``."""
        if self._tf_publisher is not None:
            self._tf_publisher.clear_frame(PRODUCT_ORIGIN_FRAME)

    def localize(
        self,
        request: LocalizationRequestDTO,
        on_collection_done: Callable[[bool, int], None],
        cancel_event: threading.Event,
    ) -> LocalizationResultDTO:
        """Collect the images, wait for the mocked calculation and publish ``product_origin``."""
        started = time.monotonic()
        node = self._node
        monitor, publisher = self._monitor, self._tf_publisher
        if node is None or monitor is None or publisher is None:
            return self._failure('localization service is not initialized', started)

        with self._lock:
            self._run_cancel_event = cancel_event
            self._workplace_changed = False
        state = monitor.latest
        if state is None or not state.in_place:
            return self._failure(NO_PRODUCT_MESSAGE, started)
        monitor.set_baseline(state)

        collection_done = False
        try:
            self._collect_images(cancel_event)
            collection_done = True
            on_collection_done(True, self._elapsed_s(started))
            pose = self._calculate_product_pose(cancel_event)
            if cancel_event.is_set():
                raise LocalizationCancelledError('cancelled')
            publisher.set_frame(PRODUCT_ORIGIN_FRAME, self._config.carrier_frame_id, pose)
            node.get_logger().info(
                f'Localized product {request.assembly_model_id} in '
                f'{self._elapsed_s(started)} s')
            return LocalizationResultDTO(
                success=True, localization_time_s=self._elapsed_s(started))
        except LocalizationCancelledError:
            monitor.set_baseline(None)
            if self._workplace_changed:
                return self._failure(WORKPLACE_CHANGED_MESSAGE, started)
            return self._failure('cancelled', started, cancelled=True)
        except LocalizationError as exc:
            monitor.set_baseline(None)
            return self._failure(str(exc), started)
        finally:
            publisher.clear_frame(LOCALIZATION_ORIGIN_FRAME)
            with self._lock:
                self._run_cancel_event = None
            if not collection_done:
                on_collection_done(False, self._elapsed_s(started))

    def _collect_images(self, cancel_event: threading.Event) -> list[Image]:
        motion, camera = self._motion, self._camera
        publisher, tf_lookup = self._tf_publisher, self._tf_lookup
        if motion is None or camera is None or publisher is None or tf_lookup is None:
            raise LocalizationError('localization service is not initialized')
        config = self._config
        images: list[Image] = []

        motion.move_to(config.initial_localization_pose, config.base_frame_id, cancel_event)
        images.append(self._settle_and_capture(cancel_event))

        publisher.set_frame(
            LOCALIZATION_ORIGIN_FRAME, config.base_frame_id,
            pose6_to_pose(LOCALIZATION_ORIGIN_PLACEHOLDER_POSE))
        # The movement controller needs the frame in TF before the first move relative to it.
        tf_lookup.lookup(PLANNING_FRAME, LOCALIZATION_ORIGIN_FRAME)

        for pose in self._poses.poses:
            motion.move_to(pose, LOCALIZATION_ORIGIN_FRAME, cancel_event)
            images.append(self._settle_and_capture(cancel_event))
        return images

    def _settle_and_capture(self, cancel_event: threading.Event) -> Image:
        if not wait_cancellable(self._config.image_capture_settling_s, cancel_event):
            raise LocalizationCancelledError('cancelled')
        assert self._camera is not None
        return self._camera.capture(cancel_event)

    def _calculate_product_pose(self, cancel_event: threading.Event) -> Pose:
        """Wait the mocked calculation time; the result is the pose the workplace estimated."""
        if not wait_cancellable(MOCK_CALCULATION_DURATION_S, cancel_event):
            raise LocalizationCancelledError('cancelled')
        assert self._tf_lookup is not None
        transform = self._tf_lookup.lookup(
            self._config.carrier_frame_id, ESTIMATED_PRODUCT_ORIGIN_FRAME).transform
        pose = Pose()
        pose.position.x = transform.translation.x
        pose.position.y = transform.translation.y
        pose.position.z = transform.translation.z
        pose.orientation = transform.rotation
        return pose

    def _on_workplace_invalid(self) -> None:
        """The workplace changed: abort a running localization and invalidate a finished one."""
        with self._lock:
            if self._run_cancel_event is not None:
                self._workplace_changed = True
                self._run_cancel_event.set()
        self._on_invalidated()

    @staticmethod
    def _elapsed_s(started: float) -> int:
        return int(round(time.monotonic() - started))

    def _failure(
        self, message: str, started: float, cancelled: bool = False
    ) -> LocalizationResultDTO:
        if self._node is not None:
            self._node.get_logger().error(f'Localization failed: {message}')
        return LocalizationResultDTO(
            success=False, cancelled=cancelled, error_message=message,
            localization_time_s=self._elapsed_s(started))
