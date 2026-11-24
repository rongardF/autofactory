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
"""``manufacturing_executor`` lifecycle node.

Runs the manufacturing loop of a station while the ``~/manufacture`` action is
active: route the product in, fetch its operation, localize it, mount the endtool,
execute the pattern groups with before/after images, report the result and eject the
product when the station has no more work for it.

Interface (available only while the node is active):
    * action ``~/manufacture`` (``manufacturing_executor/action/Manufacture``); the feedback
      ``execution_phase`` is published before every phase that is actually executed

The node never changes its own lifecycle state.
"""

import threading
from typing import Any

import rclpy
from pydantic import ValidationError
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.time import Time

from manufacturing_executor.action import Manufacture
from manufacturing_executor.enums.dependency_key_enum import DependencyKeyEnum
from manufacturing_executor.exceptions import (
    ManufacturingCancelledError,
    ManufacturingError,
    SystemFailureError,
)
from manufacturing_executor.models.dependency_nodes_dto import DependencyNodesDTO
from manufacturing_executor.models.manufacturing_executor_config_dto import (
    ManufacturingExecutorConfigDTO,
)
from manufacturing_executor.services.abort_context import AbortContext
from manufacturing_executor.services.dependency_manager import DependencyManager
from manufacturing_executor.services.execution_service import ExecutionService
from manufacturing_executor.services.executor_service import ExecutorService
from manufacturing_executor.services.idle_pose_service import IdlePoseService
from manufacturing_executor.services.image_capture_service import ImageCaptureService
from manufacturing_executor.services.lifecycle_client import LifecycleClientRegistry
from manufacturing_executor.services.loop_clients import LoopClients
from manufacturing_executor.services.manufacturing_loop import ManufacturingLoop
from manufacturing_executor.services.optimizer_service import OptimizerService
from manufacturing_executor.services.product_monitor import ProductMonitor
from manufacturing_executor.services.production_service import ProductionService
from manufacturing_executor.services.routing_service import RoutingService
from manufacturing_executor.services.tool_inventory import ToolInventory
from manufacturing_executor.services.tool_selection_service import ToolSelectionService


class ManufacturingExecutor(LifecycleNode):
    """Lifecycle node serving the ``~/manufacture`` action."""

    _SCALAR_PARAMETERS = (
        'camera_tcp',
        'flange_tcp',
        'base_frame_id',
        'service_call_timeout_s',
        'image_capture_settling_s',
        'lifecycle_transition_timeout_s',
        'localization_result_timeout_s',
        'optimization_timeout_s',
        'mount_tool_timeout_s',
        'image_capture_timeout_s',
        'movement_session_timeout_s',
    )

    def __init__(self, node_name: str = 'manufacturing_executor') -> None:
        """Declare the parameters; resources are created by lifecycle transitions."""
        super().__init__(node_name)
        defaults = ManufacturingExecutorConfigDTO.model_fields
        for name in self._SCALAR_PARAMETERS:
            self._declare(name, defaults[name].default, defaults[name].description)
        names, values = DependencyNodesDTO().defaults_as_parameters()
        self._declare('dependency_nodes.names', names, 'Dependency keys; parallel to dependency_nodes.values.')
        self._declare('dependency_nodes.values', values, 'Node names of the dependencies; parallel to dependency_nodes.names.')
        self._declare(
            'execution_optimizer_nodes',
            list(defaults['execution_optimizer_nodes'].default),
            defaults['execution_optimizer_nodes'].description,
        )
        self._declare(
            'pattern_executor_nodes',
            list(defaults['pattern_executor_nodes'].default),
            defaults['pattern_executor_nodes'].description,
        )
        idle_pose = defaults['idle_pose'].default_factory()
        self._declare('idle_pose.position', list(idle_pose.position), 'Idle pose position [x, y, z] in base_frame_id [m].')
        self._declare(
            'idle_pose.orientation', list(idle_pose.orientation),
            'Idle pose orientation [roll, pitch, yaw] in base_frame_id [rad].',
        )

        self._group = ReentrantCallbackGroup()
        self._config: ManufacturingExecutorConfigDTO | None = None
        self._registry: LifecycleClientRegistry | None = None
        self._dependencies: DependencyManager | None = None
        self._monitor: ProductMonitor | None = None
        self._inventory: ToolInventory | None = None
        # NOTE: avoid the name `_clients`: rclpy Node uses it internally.
        self._loop_clients: LoopClients | None = None
        self._loop: ManufacturingLoop | None = None
        self._action_server: ActionServer | None = None

        self._state_lock = threading.Lock()
        self._goal_running = False
        self._goal_finished = threading.Event()
        self._goal_finished.set()
        self._ctx = AbortContext.detached()
        self._goal_start: Time | None = None

    def _declare(self, name: str, default: Any, description: str | None) -> None:
        self.declare_parameter(name, default, ParameterDescriptor(description=description or name))

    # region lifecycle transitions
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Validate the parameters and freeze them into the configuration."""
        try:
            config = ManufacturingExecutorConfigDTO(
                dependency_nodes=DependencyNodesDTO.from_parameters(
                    list(self.get_parameter('dependency_nodes.names').value),
                    list(self.get_parameter('dependency_nodes.values').value),
                ),
                execution_optimizer_nodes=tuple(self.get_parameter('execution_optimizer_nodes').value),
                pattern_executor_nodes=tuple(self.get_parameter('pattern_executor_nodes').value),
                idle_pose={
                    'position': tuple(self.get_parameter('idle_pose.position').value),
                    'orientation': tuple(self.get_parameter('idle_pose.orientation').value),
                },
                **{name: self.get_parameter(name).value for name in self._SCALAR_PARAMETERS},
            )
        except ValidationError as error:
            for item in error.errors():
                location = '.'.join(str(part) for part in item['loc'])
                self.get_logger().error(f"Invalid parameter '{location}': {item['msg']}")
            return TransitionCallbackReturn.FAILURE
        except ValueError as error:
            self.get_logger().error(f'Invalid parameters: {error}')
            return TransitionCallbackReturn.FAILURE
        self._config = config
        self.get_logger().info(f'Configured: {config}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Verify and bring up the dependencies, create the clients and the action server."""
        config = self._config
        if config is None:
            self.get_logger().error('Activation failed: node is not configured')
            return TransitionCallbackReturn.FAILURE
        try:
            self._registry = LifecycleClientRegistry(
                self,
                callback_group=self._group,
                call_timeout_s=config.service_call_timeout_s,
                transition_timeout_s=config.lifecycle_transition_timeout_s,
            )
            self._dependencies = DependencyManager(self, config, self._registry)
            self._dependencies.activate()
            self._create_loop(config)
            self._action_server = ActionServer(
                self,
                Manufacture,
                '~/manufacture',
                execute_callback=self._execute_callback,
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback,
                callback_group=self._group,
            )
            with self._state_lock:
                self._goal_running = False
            result = super().on_activate(state)
            if result != TransitionCallbackReturn.SUCCESS:
                raise RuntimeError(f'base activation returned {result}')
        except Exception as error:  # noqa: BLE001 - roll back and fail the transition
            self.get_logger().error(f'Activation failed: {error}')
            self._release(owned_nodes=self._dependencies is not None and self._loop is not None)
            return TransitionCallbackReturn.FAILURE
        self.get_logger().info('Activated')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Abort a running goal, release the owned nodes and destroy the interfaces."""
        failures = self._release(owned_nodes=True)
        super().on_deactivate(state)
        if failures:
            self.get_logger().error(f'Deactivation incomplete: {failures}')
            return TransitionCallbackReturn.FAILURE
        self.get_logger().info('Deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Drop the configuration."""
        self._config = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Release everything that is still held, whatever the current state."""
        failures = self._release(owned_nodes=self._dependencies is not None)
        self._config = None
        if failures:
            self.get_logger().error(f'Shutdown incomplete: {failures}')
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State) -> TransitionCallbackReturn:
        """Log the error transition and defer to the parent class."""
        self.get_logger().error(f'Error occurred in state: {state.label}')
        return super().on_error(state)
    # endregion lifecycle transitions

    # region resource handling
    def _create_loop(self, config: ManufacturingExecutorConfigDTO) -> None:
        assert self._registry is not None
        dependencies = config.dependency_nodes
        self._monitor = ProductMonitor(
            self, dependencies.node_name(DependencyKeyEnum.STATION_WORKPLACE), self._group
        )
        self._inventory = ToolInventory(
            self,
            dependencies.node_name(DependencyKeyEnum.TOOL_RACK),
            dependencies.node_name(DependencyKeyEnum.TOOL_MOUNT),
            self._group,
        )
        clients = LoopClients(self, config, self._group)
        self._loop_clients = clients
        timeout_s = config.service_call_timeout_s
        image_capture = ImageCaptureService(self, config, clients)
        self._loop = ManufacturingLoop(
            self,
            config,
            clients,
            RoutingService(self, clients, self._monitor, timeout_s),
            ProductionService(self, clients, timeout_s),
            ToolSelectionService(self, config, clients, self._registry, self._inventory),
            OptimizerService(self, config, clients, self._registry),
            ExecutorService(self, config, clients, self._registry, self._inventory),
            ExecutionService(self, config, clients, image_capture),
            IdlePoseService(self, config, clients),
        )

    def _release(self, owned_nodes: bool) -> list[str]:
        """Abort a running goal and release everything created on activation.

        :param owned_nodes: Whether the localizer, optimizers and executors are released as well.
        :returns: Descriptions of the steps that failed.
        """
        failures: list[str] = []
        config = self._config
        with self._state_lock:
            running = self._goal_running
        if running:
            self._ctx.abort(SystemFailureError('Manufacturing aborted: node deactivated'))
            timeout_s = config.lifecycle_transition_timeout_s if config is not None else 10.0
            if not self._goal_finished.wait(timeout_s):
                failures.append('the running goal did not finish')
        if self._action_server is not None:
            self._action_server.destroy()
            self._action_server = None
        if self._dependencies is not None and owned_nodes:
            failures.extend(self._dependencies.deactivate())
        self._dependencies = None
        self._loop = None
        destroyers = [
            resource.destroy
            for resource in (self._monitor, self._inventory, self._loop_clients)
            if resource is not None
        ]
        if self._registry is not None:
            destroyers.append(self._registry.destroy_all)
        for destroy in destroyers:
            try:
                destroy()
            except Exception as error:  # noqa: BLE001 - keep releasing the rest
                failures.append(str(error))
        self._monitor = self._inventory = self._loop_clients = self._registry = None
        return failures
    # endregion resource handling

    # region action callbacks
    def _goal_callback(self, request: Manufacture.Goal) -> GoalResponse:
        """Accept a goal only if the node is active and no other goal is running."""
        with self._state_lock:
            if self._loop is None or self._goal_running:
                self.get_logger().warning('Rejecting goal: node is not active or already manufacturing')
                return GoalResponse.REJECT
            self._goal_running = True
            self._goal_finished.clear()
            self._ctx = AbortContext()
            self._goal_start = self.get_clock().now()
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """Accept every cancel request; the loop ends with ``success=True``."""
        self.get_logger().info('Cancel requested for the manufacturing goal')
        self._ctx.abort(ManufacturingCancelledError())
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle: ServerGoalHandle) -> Manufacture.Result:
        """Run the loop; convert every outcome into the result, nothing is raised."""
        ctx = self._ctx
        loop = self._loop
        start = self._goal_start
        error: ManufacturingError | None = None
        try:
            if loop is None:
                raise SystemFailureError('Manufacturing loop is not available')
            loop.run(ctx, lambda phase: self._publish_phase(goal_handle, phase))
        except ManufacturingError as caught:
            error = caught
        except Exception as caught:  # noqa: BLE001 - nothing may cross the action boundary
            self.get_logger().error(f'Unexpected error: {caught!r}')
            error = SystemFailureError(f'Unexpected error: {caught}')
        # The cause of the abort wins over the error it provoked in a child call.
        error = ctx.error or error
        if error is None:
            error = SystemFailureError('Manufacturing loop ended unexpectedly')
        if self._monitor is not None:
            self._monitor.disarm()

        result = Manufacture.Result()
        end = self.get_clock().now()
        result.total_manufacturing_time_s = (end - start).nanoseconds / 1e9 if start is not None else 0.0
        if isinstance(error, ManufacturingCancelledError):
            result.success = True
            result.error_message = ''
            self._finish_cancelled(goal_handle)
        else:
            result.success = False
            result.error_message = str(error)
            self.get_logger().error(f'Manufacturing aborted: {error}')
            goal_handle.abort()
        with self._state_lock:
            self._goal_running = False
        self._goal_finished.set()
        return result

    def _publish_phase(self, goal_handle: ServerGoalHandle, phase: str) -> None:
        """Publish the ``execution_phase`` feedback; a failed publish never stops manufacturing."""
        self.get_logger().info(f'Execution phase: {phase}')
        feedback = Manufacture.Feedback()
        feedback.execution_phase = phase
        try:
            goal_handle.publish_feedback(feedback)
        except Exception as error:  # noqa: BLE001 - feedback is informational only
            self.get_logger().warning(f'Could not publish feedback {phase!r}: {error}')

    def _finish_cancelled(self, goal_handle: ServerGoalHandle) -> None:
        """Mark the goal canceled once rclpy recorded the cancel request."""
        waited = threading.Event()
        deadline_polls = int(2.0 / 0.02)
        for _ in range(deadline_polls):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return
            waited.wait(0.02)
        goal_handle.abort()
    # endregion action callbacks


def main(args: list[str] | None = None) -> None:
    """Spin the node on a multi-threaded executor until shutdown."""
    rclpy.init(args=args)
    node = ManufacturingExecutor()
    executor = MultiThreadedExecutor(num_threads=8)
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
