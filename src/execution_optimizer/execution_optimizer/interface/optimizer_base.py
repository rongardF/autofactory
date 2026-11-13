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

"""Abstract optimizer lifecycle node.

The :class:`OptimizerBase` implements the lifecycle transitions, the camera
intrinsics handling and the ``optimize_patterns`` service callback that are the
same for every operation type. Concrete optimizers provide the operation specific
configuration, grouping service, service interface and pattern conversion through
the abstract hooks declared here.
"""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from collections.abc import Sequence
from typing import Protocol

from pydantic import ValidationError
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.service import Service

from geometry_msgs.msg import PoseStamped

from execution_optimizer.exceptions import OptimizationError
from execution_optimizer.model.camera_intrinsics_dto import CameraIntrinsicsDTO
from execution_optimizer.model.optimizer_config_dto import OptimizerConfigDTO
from execution_optimizer.model.pattern_group_dto import PatternGroupDTO
from execution_optimizer.model.pattern_points_dto import PatternPointsDTO
from execution_optimizer.msg import PatternGroup
from execution_optimizer.service.camera_info_provider import CameraInfoProvider


class GroupingService(Protocol):
    """What the base node needs from an operation specific grouping service."""

    def group(
        self, patterns: Sequence[PatternPointsDTO], intrinsics: CameraIntrinsicsDTO
    ) -> list[PatternGroupDTO]:
        """Group the patterns or raise :class:`OptimizationError`."""
        ...


class OptimizerBase(LifecycleNode, ABC):
    """Shared lifecycle node base for all optimizer variants."""

    _COMMON_PARAMETERS = ('product_frame_id', 'camera_node_name', 'camera_info_timeout_s')

    def __init__(self, node_name: str = 'optimizer_base') -> None:
        """Initialise the node and declare the shared parameters.

        :param node_name: ROS 2 node name passed to :class:`LifecycleNode`.
        :type node_name: str
        """
        super().__init__(node_name)

        self._config: OptimizerConfigDTO | None = None
        self._camera_provider: CameraInfoProvider | None = None
        self._optimizer: GroupingService | None = None
        self._service: Service | None = None

        # The camera action client must not share a group with the lifecycle services,
        # which are blocked while a transition callback waits for the camera.
        self._camera_callback_group = ReentrantCallbackGroup()
        self._service_callback_group = MutuallyExclusiveCallbackGroup()

        self.declare_parameter(
            'product_frame_id',
            'product_origin',
            ParameterDescriptor(
                description='Frame of all pattern poses and of the output capture poses.'
            ),
        )
        self.declare_parameter(
            'camera_node_name',
            'basler_camera',
            ParameterDescriptor(
                description='Camera node serving the get_image action; the action name is '
                '"<camera_node_name>/get_image".'
            ),
        )
        self.declare_parameter(
            'camera_info_timeout_s',
            5.0,
            ParameterDescriptor(
                description='Timeout in seconds for the camera action server and for the '
                'get_image result when the camera intrinsics are fetched.'
            ),
        )

    # region: subclass hooks
    @abstractmethod
    def _build_config(self) -> OptimizerConfigDTO:
        """Read the parameters and build the frozen configuration DTO.

        Raising :class:`pydantic.ValidationError` fails the configure transition.
        """
        raise NotImplementedError()

    @abstractmethod
    def _create_optimizer(self, config: OptimizerConfigDTO) -> GroupingService:
        """Create the (stateless) grouping service for the configuration."""
        raise NotImplementedError()

    @abstractmethod
    def _create_interface(self) -> Service:
        """Create the node interface; called on activation.

        The service must use ``self._service_callback_group`` and call
        :meth:`_handle_optimize`.
        """
        raise NotImplementedError()

    @abstractmethod
    def _to_pattern_points(self, patterns: Sequence) -> list[PatternPointsDTO]:
        """Convert the request's pattern messages into ROS-free pattern points."""
        raise NotImplementedError()

    def _validate_intrinsics(self, intrinsics: CameraIntrinsicsDTO) -> None:
        """Validate configuration against the camera; raise to fail the activation."""
    # endregion: subclass hooks

    # region: configuration helpers
    def _read_common_kwargs(self) -> dict:
        """Read the shared parameters into keyword arguments for the config DTO."""
        return {name: self.get_parameter(name).value for name in self._COMMON_PARAMETERS}
    # endregion: configuration helpers

    # region: lifecycle callbacks
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Build the config and create the internal services without initializing them."""
        self.get_logger().info(f'Configuring from state: {state.label}')
        try:
            config = self._build_config()
        except ValidationError as error:
            for item in error.errors():
                location = '.'.join(str(part) for part in item['loc'])
                self.get_logger().error(f"Invalid parameter '{location}': {item['msg']}")
            return TransitionCallbackReturn.FAILURE
        except Exception as error:  # noqa: BLE001 - a failing transition must not raise
            self.get_logger().error(f'Failed to build the configuration: {error}')
            return TransitionCallbackReturn.FAILURE

        try:
            self._config = config
            self._camera_provider = CameraInfoProvider(
                self,
                config.get_image_action_name,
                config.camera_info_timeout_s,
                self._camera_callback_group,
            )
            self._optimizer = self._create_optimizer(config)
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f'Failed to create the internal services: {error}')
            self._drop_internal()
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info(f'Configured: {config}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Fetch the camera intrinsics, then create the node interface."""
        self.get_logger().info(f'Activating from state: {state.label}')
        if (
            self._config is None
            or self._camera_provider is None
            or self._optimizer is None
        ):
            self.get_logger().error('Sanity check failed, node is not configured.')
            return TransitionCallbackReturn.FAILURE
        try:
            intrinsics = self._camera_provider.initialize()
            self._validate_intrinsics(intrinsics)
            self._service = self._create_interface()
            if super().on_activate(state) != TransitionCallbackReturn.SUCCESS:
                raise RuntimeError('base lifecycle activation failed')
        except Exception as error:  # noqa: BLE001 - tear down and fail the transition
            self.get_logger().error(f'Activation failed: {error}')
            self._destroy_interface()
            self._camera_provider.deinitialize()
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info('Activated')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Destroy the node interface and release the camera intrinsics."""
        self.get_logger().info(f'Deactivating from state: {state.label}')
        super().on_deactivate(state)
        self._destroy_interface()
        if self._camera_provider is not None:
            self._camera_provider.deinitialize()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Delete the internal services and the configuration."""
        self.get_logger().info(f'Cleaning up from state: {state.label}')
        self._drop_internal()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Release everything that is still held, whatever the current state."""
        self.get_logger().info(f'Shutting down from state: {state.label}')
        self._destroy_interface()
        self._drop_internal()
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Log the error transition and defer to the parent class."""
        self.get_logger().error(f'Error occurred in state: {state.label}')
        return super().on_error(state)
    # endregion: lifecycle callbacks

    # region: resource handling
    def _destroy_interface(self) -> None:
        """Destroy the node interface (idempotent)."""
        if self._service is not None:
            self.destroy_service(self._service)
            self._service = None

    def _drop_internal(self) -> None:
        """Delete the camera provider, the grouping service and the configuration."""
        if self._camera_provider is not None:
            self._camera_provider.deinitialize()
        self._camera_provider = None
        self._optimizer = None
        self._config = None
    # endregion: resource handling

    # region: service
    def _handle_optimize(self, request, response):
        """Group the request's patterns; every failure becomes a result object."""
        start = time.perf_counter()
        # Snapshot: a concurrent deactivation drops the provider's intrinsics.
        optimizer = self._optimizer
        provider = self._camera_provider
        config = self._config
        intrinsics = provider.intrinsics if provider is not None else None
        try:
            if optimizer is None or intrinsics is None or config is None:
                raise OptimizationError('Optimizer is not active')
            patterns = self._to_pattern_points(request.patterns)
            groups = optimizer.group(patterns, intrinsics)
            response.groups = [self._to_group_msg(group, config) for group in groups]
            response.success = True
            response.error_message = ''
        except OptimizationError as error:
            self.get_logger().error(str(error))
            self._fail(response, str(error))
        except Exception as error:  # noqa: BLE001 - nothing may cross the service boundary
            self.get_logger().error(f'Unexpected error: {error!r}')
            self._fail(response, f'internal error: {error}')
        response.optimization_time_s = time.perf_counter() - start
        return response

    @staticmethod
    def _fail(response, message: str) -> None:
        """Fill a failure result."""
        response.groups = []
        response.success = False
        response.error_message = message

    @staticmethod
    def _to_group_msg(group: PatternGroupDTO, config: OptimizerConfigDTO) -> PatternGroup:
        """Convert a grouping result into the interface message."""
        pose = PoseStamped()
        pose.header.frame_id = config.product_frame_id
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = group.position
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = group.quaternion_xyzw
        message = PatternGroup()
        message.pattern_ids = list(group.pattern_ids)
        message.image_capture_poses = [pose]
        return message
    # endregion: service
