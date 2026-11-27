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
"""Lifecycle node that owns all MoveIt planning-scene object and ACM changes."""

from typing import Callable, TypeVar

from pydantic import ValidationError
from rcl_interfaces.msg import ParameterDescriptor
from rclpy import init, ok, shutdown
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from station_utils.autofactory_base_node import AutofactoryBaseNode

from world_manager.models import (
    MoveItConfigDTO,
    ObjectInfoDTO,
    PoseDTO,
    WorldChangesDTO,
)
from world_manager.services.planning_scene_gateway import RosPlanningSceneGateway
from world_manager.services.planning_scene_service import PlanningSceneService
from world_manager.srv import ApplyWorldChanges, AttachObject, ListObjects

_Response = TypeVar('_Response')
_NOT_ACTIVE = 'node not active'


class MoveIt2WorldManager(AutofactoryBaseNode):
    """Owns the MoveIt planning scene's collision objects and ACM."""

    def __init__(self, node_name: str = 'moveit2_world_manager') -> None:
        """Declare the service parameters; create resources in configure."""
        super().__init__(node_name)
        self.declare_parameter(
            'get_scene_service', '/get_planning_scene',
            ParameterDescriptor(description='MoveIt GetPlanningScene service name'))
        self.declare_parameter(
            'apply_scene_service', '/apply_planning_scene',
            ParameterDescriptor(description='MoveIt ApplyPlanningScene service name'))
        self.declare_parameter(
            'service_timeout_sec', 5.0,
            ParameterDescriptor(description='Availability and response timeout for MoveIt services'))
        self.declare_parameter(
            'planning_frame', 'world',
            ParameterDescriptor(description='Planning-scene frame for world collision objects'))
        self._active = False
        self._callback_group = ReentrantCallbackGroup()
        self._scene_service: PlanningSceneService | None = None
        self._gateway: RosPlanningSceneGateway | None = None
        self._config: MoveItConfigDTO | None = None
        self._servers: list = []

    def _read_config(self) -> MoveItConfigDTO:
        return MoveItConfigDTO(
            get_scene_service=self.get_parameter('get_scene_service').value,
            apply_scene_service=self.get_parameter('apply_scene_service').value,
            service_timeout_sec=self.get_parameter('service_timeout_sec').value,
            planning_frame=self.get_parameter('planning_frame').value,
        )

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Validate and retain the MoveIt configuration."""
        try:
            self._config = self._read_config()
        except ValidationError as e:
            self.get_logger().error(f'invalid parameters: {e}')
            self._publish_transition_error(f'invalid parameters')
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Create the MoveIt clients and expose the manager services."""
        assert self._config is not None
        try:
            self._gateway = RosPlanningSceneGateway(
                self, self._config.get_scene_service, self._config.apply_scene_service,
                self._config.service_timeout_sec, self._callback_group,
            )
            self._scene_service = PlanningSceneService(self._gateway, self._config.planning_frame)
            for service_type, name, callback in (
                (ApplyWorldChanges, '~/apply_world_changes', self._on_apply),
                (AttachObject, '~/attach_object', self._on_attach),
                (ListObjects, '~/list_objects', self._on_list),
            ):
                self._servers.append(self.create_service(
                    service_type, name, callback, callback_group=self._callback_group))
            result = super().on_activate(state)
            if result != TransitionCallbackReturn.SUCCESS:
                self._destroy_active_interfaces()
                return result
            self._active = True
        except Exception as e:
            self.get_logger().error(f'failed to activate interfaces: {e}')
            self._destroy_active_interfaces()
            self._publish_transition_error(f'failed to activate interfaces')
            return TransitionCallbackReturn.FAILURE
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Remove the manager services and MoveIt service clients."""
        self._active = False
        result = super().on_deactivate(state)
        self._destroy_active_interfaces()
        return result

    def _destroy_active_interfaces(self) -> None:
        for server in self._servers:
            self.destroy_service(server)
        self._servers = []
        if self._gateway is not None:
            self._gateway.destroy()
            self._gateway = None
        self._scene_service = None

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Release configured resources."""
        self._active = False
        self._destroy_active_interfaces()
        self._config = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Release resources on shutdown."""
        return self.on_cleanup(state)

    def _run(
        self,
        response: _Response,
        action: str,
        operation: Callable[[_Response], None],
    ) -> _Response:
        """Run an operation and convert errors to the ROS response boundary."""
        if not self._active or self._scene_service is None:
            response.success = False
            response.error_message = _NOT_ACTIVE
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

    def _on_apply(
        self,
        request: ApplyWorldChanges.Request,
        response: ApplyWorldChanges.Response,
    ) -> ApplyWorldChanges.Response:
        def operation(_: ApplyWorldChanges.Response) -> None:
            changes = WorldChangesDTO.from_msg(request)
            self._scene_service.apply_world_changes(changes)

        return self._run(response, 'apply_world_changes', operation)

    def _on_attach(
        self,
        request: AttachObject.Request,
        response: AttachObject.Response,
    ) -> AttachObject.Response:
        return self._run(
            response,
            'attach_object',
            lambda _: self._scene_service.attach_object(
                request.object_id,
                request.link_name,
                PoseDTO.from_stamped_msg(request.pose),
                tuple(request.touch_links),
            ),
        )

    def _on_list(
        self,
        request: ListObjects.Request,
        response: ListObjects.Response,
    ) -> ListObjects.Response:
        def operation(res: ListObjects.Response) -> None:
            res.objects = [
                ObjectInfoDTO.to_msg(item)
                for item in self._scene_service.list_objects()
            ]

        response.objects = []
        return self._run(response, 'list_objects', operation)


def main(args: list[str] | None = None) -> None:
    """Entry point of the ``moveit2_world_manager`` executable."""
    init(args=args)
    node = MoveIt2WorldManager()
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
