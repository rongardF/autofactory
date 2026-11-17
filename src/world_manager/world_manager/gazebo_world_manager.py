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
"""Lifecycle node exposing the Gazebo world manager services."""

from threading import Event
from typing import Callable, TypeVar

from pydantic import ValidationError
from rcl_interfaces.msg import ParameterDescriptor
from rclpy import init, ok, shutdown
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from world_manager.exceptions import ActivationError
from world_manager.models import (
    GazeboConfigDTO,
    ModelInfoDTO,
    ModelSpecDTO,
    PoseDTO,
)
from world_manager.msg import ModelList
from world_manager.services.detachable_joint_controller import DetachableJointController
from world_manager.services.gazebo_world_service import GazeboWorldService
from world_manager.services.model_list_publisher import ModelListPublisher
from world_manager.services.model_registry import ModelRegistry
from world_manager.services.pose_resolver import PoseResolver
from world_manager.services.real_gz_transport_adapter import RealGzTransportAdapter
from world_manager.services.sdf_inspector import SdfInspector
from world_manager.srv import (
    DeleteModels,
    ListModels,
    SetModelAttachment,
    SetModelPose,
    SpawnModels,
)

_Response = TypeVar('_Response')


class GazeboWorldManager(LifecycleNode):
    """Owns every spawn, delete, set_pose and attach/detach in the Gazebo world."""

    def __init__(self, node_name: str = 'gazebo_world_manager') -> None:
        """Declare parameters; resources are created in ``on_configure``."""
        super().__init__(node_name)
        self.declare_parameter('world_name', '', ParameterDescriptor(description='Name of the Gazebo world'))
        self.declare_parameter(
            'world_frame', 'world', ParameterDescriptor(description='TF frame that coincides with the world origin'))
        self.declare_parameter(
            'service_timeout_sec', 5.0, ParameterDescriptor(description='Timeout of one gz-transport request'))
        self.declare_parameter('tf_timeout_sec', 3.0, ParameterDescriptor(description='Timeout of a TF lookup'))
        self.declare_parameter(
            'confirm_timeout_sec', 5.0,
            ParameterDescriptor(description='Wait for a joint state event after an attach/detach request'))
        self.declare_parameter(
            'republish_period_sec', 0.25,
            ParameterDescriptor(description='Period at which an unconfirmed attach/detach request is republished'))
        self.declare_parameter(
            'operation_timeout_sec', 10.0,
            ParameterDescriptor(description='Total deadline of one mutating service call'))
        self.declare_parameter(
            'rollback_timeout_sec', 1.0,
            ParameterDescriptor(description='Per-joint confirmation timeout for rollback and detach before removal'))

        self._group = ReentrantCallbackGroup()
        self._active = False
        self._dirty = Event()
        self._service: GazeboWorldService | None = None
        self._publisher_thread: ModelListPublisher | None = None
        self._tf_buffer: Buffer | None = None
        self._tf_listener: TransformListener | None = None
        self._models_pub = None
        self._servers: list = []

    def _read_config(self) -> GazeboConfigDTO:
        names = (
            'world_name', 'world_frame', 'service_timeout_sec', 'tf_timeout_sec', 'confirm_timeout_sec',
            'republish_period_sec', 'operation_timeout_sec', 'rollback_timeout_sec',
        )
        return GazeboConfigDTO(**{name: self.get_parameter(name).value for name in names})

    def _lookup_tf(self, target: str, source: str, timeout_sec: float) -> PoseDTO:
        assert self._tf_buffer is not None
        transform = self._tf_buffer.lookup_transform(
            target, source, Time(), Duration(seconds=timeout_sec)).transform
        return PoseDTO.from_transform_msg(transform, target)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Create the Gazebo service and persistent model registry."""
        try:
            config = self._read_config()
        except ValidationError as e:
            self.get_logger().error(f'invalid parameters: {e}')
            return TransitionCallbackReturn.FAILURE
        try:
            logger = self.get_logger()
            adapter = RealGzTransportAdapter(logger)
        except Exception as e:
            self.get_logger().error(f'cannot create the gz-transport node: {e}')
            return TransitionCallbackReturn.FAILURE

        registry = ModelRegistry(on_change=self._dirty.set)
        self._service = GazeboWorldService(
            adapter=adapter,
            registry=registry,
            controller=DetachableJointController(adapter, registry, config.republish_period_sec, logger),
            inspector=SdfInspector(),
            resolver=PoseResolver(self._lookup_tf, config.world_frame, config.tf_timeout_sec),
            config=config,
            logger=logger,
        )

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Check that the world answers, then start accepting requests."""
        assert self._service is not None
        try:
            self._service.activate()
        except ActivationError as e:
            self.get_logger().error(str(e))
            return TransitionCallbackReturn.FAILURE
        try:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._models_pub = self.create_lifecycle_publisher(
                ModelList, '~/models',
                QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                           durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
            for service_type, name, callback in (
                (SpawnModels, '~/spawn_models', self._on_spawn),
                (DeleteModels, '~/delete_models', self._on_delete),
                (SetModelPose, '~/set_model_pose', self._on_set_pose),
                (SetModelAttachment, '~/set_model_attachment', self._on_set_attachment),
                (ListModels, '~/list_models', self._on_list),
            ):
                self._servers.append(self.create_service(service_type, name, callback, callback_group=self._group))
            result = super().on_activate(state)
            if result != TransitionCallbackReturn.SUCCESS:
                self._destroy_active_interfaces()
                return result
            self._active = True
            self._publisher_thread = ModelListPublisher(
                self._dirty, self._service.list_models, self._publish_models, self.get_logger())
            self._publisher_thread.start()
            self._publisher_thread.republish()
        except Exception as e:
            self.get_logger().error(f'failed to activate interfaces: {e}')
            self._active = False
            self._destroy_active_interfaces()
            return TransitionCallbackReturn.FAILURE
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Remove public interfaces; keep the model registry for reactivation."""
        self._active = False
        if self._publisher_thread is not None:
            self._publisher_thread.stop()
            self._publisher_thread = None
        result = super().on_deactivate(state)
        self._destroy_active_interfaces()
        return result

    def _destroy_active_interfaces(self) -> None:
        if self._publisher_thread is not None:
            self._publisher_thread.stop()
            self._publisher_thread = None
        for server in self._servers:
            self.destroy_service(server)
        self._servers = []
        if self._models_pub is not None:
            self.destroy_lifecycle_publisher(self._models_pub)
            self._models_pub = None
        if self._tf_listener is not None:
            self._tf_listener.unregister()
        self._tf_listener = None
        self._tf_buffer = None

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Release configured resources."""
        self._active = False
        self._destroy_active_interfaces()
        self._service = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Same as cleanup."""
        return self.on_cleanup(state)

    def _publish_models(self, models: list[ModelInfoDTO]) -> None:
        """Called on the publisher thread only."""
        msg = ModelList()
        msg.models = [ModelInfoDTO.to_msg(model) for model in models]
        self._models_pub.publish(msg)

    def _run(self, response: _Response, action: str, operation: Callable[[_Response], None]) -> _Response:
        """Run ``operation`` and convert any failure into an unsuccessful response."""
        if not self._active or self._service is None:
            response.success = False
            response.error_message = 'node not active'
            return response
        try:
            operation(response)
            response.success = True
            response.error_message = ''
        except Exception as e:
            self.get_logger().error(f'{action} failed: {e}')
            response.success = False
            response.error_message = str(e)
        return response

    def _on_spawn(self, request: SpawnModels.Request, response: SpawnModels.Response) -> SpawnModels.Response:
        def operation(res: SpawnModels.Response) -> None:
            specs = [
                ModelSpecDTO.from_msg(m)
                for m in request.models
            ]
            res.spawned_ids = self._service.spawn(specs, request.owner)

        return self._run(response, 'spawn_models', operation)

    def _on_delete(self, request: DeleteModels.Request, response: DeleteModels.Response) -> DeleteModels.Response:
        def operation(res: DeleteModels.Response) -> None:
            result, error = self._service.delete(list(request.model_ids), request.owner)
            res.deleted_ids = list(result.deleted_ids)
            res.unknown_ids = list(result.unknown_ids)
            if error:
                raise RuntimeError(error)

        response.deleted_ids = []
        response.unknown_ids = []
        return self._run(response, 'delete_models', operation)

    def _on_set_pose(self, request: SetModelPose.Request, response: SetModelPose.Response) -> SetModelPose.Response:
        return self._run(
            response, 'set_model_pose',
            lambda _: self._service.set_pose(request.model_id, PoseDTO.from_stamped_msg(request.pose)))

    def _on_set_attachment(
        self, request: SetModelAttachment.Request, response: SetModelAttachment.Response,
    ) -> SetModelAttachment.Response:
        return self._run(
            response, 'set_model_attachment',
            lambda _: self._service.set_attachment(request.model_id, request.link_name))

    def _on_list(self, request: ListModels.Request, response: ListModels.Response) -> ListModels.Response:
        def operation(res: ListModels.Response) -> None:
            res.models = [ModelInfoDTO.to_msg(model) for model in self._service.list_models(request.owner)]

        return self._run(response, 'list_models', operation)


def main(args: list[str] | None = None) -> None:
    """Entry point of the ``gazebo_world_manager`` executable."""
    init(args=args)
    node = GazeboWorldManager()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if ok():
            shutdown()
