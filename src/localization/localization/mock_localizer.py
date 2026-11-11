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
"""``mock_localizer`` lifecycle node.

Stands in for the vision based localizer: it exposes the same interface and moves
the robot and captures the images, but does not calculate the product pose.

Interface:
    * action ``~/localize_product`` (``localization/action/LocalizeProduct``)
    * topic ``~/localized`` (``std_msgs/Bool``, latched): True while the published
      ``product_origin`` frame is valid
    * TF ``localization_origin`` and ``product_origin`` on ``/tf``

The node never changes its own lifecycle state.
"""

import threading
from typing import Any

import rclpy
from pydantic import ValidationError
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, LifecyclePublisher, State, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool

from localization.interface.localization_service import LocalizationService
from localization.models.localization_poses_dto import LocalizationPosesDTO
from localization.models.localizer_config_dto import LocalizerConfigDTO
from localization.services.localize_action_server import LocalizeProductActionServer
from localization.services.mock_localization_service import MockLocalizationService
from localization.utils.poses_file_loader import load_poses_file


class MockLocalizer(LifecycleNode):
    """Lifecycle node mocking the product localizer."""

    # Placeholder; set the real pose with the 'initial_localization_pose' parameter.
    _DEFAULT_INITIAL_POSE = [1.0, 0.0, 1.46, 0.0, 1.5708, 0.0]

    _LOCALIZED_QOS = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    def __init__(self, node_name: str = 'mock_localizer') -> None:
        """Declare the parameters; resources are created by lifecycle transitions."""
        super().__init__(node_name)
        defaults = LocalizerConfigDTO.model_fields
        for name in (
            'station_workplace_node_name', 'movement_controller_node_name', 'camera_node_name',
            'camera_frame_id', 'base_frame_id', 'carrier_frame_id',
            'image_capture_settling_s', 'image_capture_timeout_s',
        ):
            self._declare(name, defaults[name].default, defaults[name].description)
        self._declare(
            'initial_localization_pose', self._DEFAULT_INITIAL_POSE,
            defaults['initial_localization_pose'].description)
        self._declare(
            'calibration_poses_file', '', defaults['calibration_poses_file'].description)

        self._action_group = ReentrantCallbackGroup()
        self._config: LocalizerConfigDTO | None = None
        self._poses: LocalizationPosesDTO | None = None
        self._service: LocalizationService | None = None
        self._action_server: LocalizeProductActionServer | None = None
        self._localized_pub: LifecyclePublisher | None = None
        self._localized_lock = threading.Lock()
        self._localized = False

    def _declare(self, name: str, default: Any, description: str | None) -> None:
        self.declare_parameter(name, default, ParameterDescriptor(description=description or name))

    def _param(self, name: str) -> Any:
        return self.get_parameter(name).value

    # region lifecycle transitions
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Validate the parameters and the poses file; create the (uninitialized) service."""
        try:
            config = LocalizerConfigDTO(**{
                name: self._param(name) for name in LocalizerConfigDTO.model_fields})
        except ValidationError as exc:
            for error in exc.errors():
                location = '.'.join(str(part) for part in error['loc'])
                self.get_logger().error(f"Invalid parameter '{location}': {error['msg']}")
            return TransitionCallbackReturn.FAILURE
        try:
            poses = load_poses_file(config.calibration_poses_file)
        except ValueError as exc:
            self.get_logger().error(f'Invalid calibration poses: {exc}')
            return TransitionCallbackReturn.FAILURE

        self._config = config
        self._poses = poses
        self._service = MockLocalizationService(config, poses)
        self.get_logger().info(f'Configured with {len(poses.poses)} localization poses: {config}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Initialize the service and create the action server and the topic."""
        if self._service is None:
            self.get_logger().error('Activation failed: node is not configured')
            return TransitionCallbackReturn.FAILURE
        try:
            self._service.initialize(self, self._on_invalidated)
            self._localized_pub = self.create_publisher(Bool, '~/localized', self._LOCALIZED_QOS)
            self._action_server = LocalizeProductActionServer(
                self, self._service, self._action_group,
                on_goal_accepted=self._on_goal_accepted, on_succeeded=self._on_succeeded)
            result = super().on_activate(state)
            if result != TransitionCallbackReturn.SUCCESS:
                raise RuntimeError(f'base activation returned {result}')
        except Exception as exc:  # noqa: BLE001 - tear down and fail the transition
            self.get_logger().error(f'Activation failed: {exc}')
            self._destroy_interfaces()
            return TransitionCallbackReturn.FAILURE
        with self._localized_lock:
            self._localized = False
        self._localized_pub.publish(Bool(data=False))
        self.get_logger().info('Activated')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Cancel a running goal, tear down the service and destroy the interface."""
        self._set_localized(False)
        super().on_deactivate(state)
        self._destroy_interfaces()
        self.get_logger().info('Deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Drop the configuration and the service."""
        self._drop_service()
        self.get_logger().info('Cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Release everything that is still held, whatever the current state."""
        if self._action_server is not None or self._localized_pub is not None:
            self._set_localized(False)
            super().on_deactivate(state)
        self._destroy_interfaces()
        self._drop_service()
        return TransitionCallbackReturn.SUCCESS
    # endregion lifecycle transitions

    # region resource handling
    def _destroy_interfaces(self) -> None:
        """Cancel a running goal and destroy the action server, the service and the topic."""
        if self._action_server is not None:
            self._action_server.destroy()
            self._action_server = None
        if self._service is not None:
            self._service.teardown()
        if self._localized_pub is not None:
            self.destroy_publisher(self._localized_pub)
            self._localized_pub = None

    def _drop_service(self) -> None:
        self._service = None
        self._poses = None
        self._config = None
    # endregion resource handling

    # region localized state
    def _set_localized(self, value: bool) -> None:
        """Publish the localized flag if it changed."""
        with self._localized_lock:
            if self._localized == value or self._localized_pub is None:
                return
            self._localized = value
            self._localized_pub.publish(Bool(data=value))

    def _on_goal_accepted(self) -> None:
        """A new localization starts: the previous one is void."""
        self._on_invalidated()

    def _on_invalidated(self) -> None:
        """The product moved or a new localization started: the localization is no longer valid."""
        self._set_localized(False)
        if self._service is not None:
            self._service.invalidate()

    def _on_succeeded(self) -> None:
        self._set_localized(True)
    # endregion localized state


def main(args: list[str] | None = None) -> None:
    """Spin the mock localizer on a multi-threaded executor until shutdown."""
    rclpy.init(args=args)
    node = MockLocalizer()
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
