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

"""Abstract pattern executor lifecycle node.

The :class:`PatternExecutor` provides the shared, pattern-type-agnostic
behaviour for every concrete executor: lifecycle management, the movement
controller action client, the latched ``~/executing_patterns`` status topic,
the end-tool lifecycle manager, the single-goal concurrency guard, and the
common abort/cancel/error flow.

Concrete executors (for example the dispensing executor) provide the
type-specific action interface and the per-action-type behaviour through the
abstract hooks declared here.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from threading import Lock

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.publisher import Publisher
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from movement_controller.action import ExecuteTrajectory
from std_msgs.msg import Bool

from pattern_executors.model.pattern_executor_config_dto import PatternExecutorConfigDTO
from pattern_executors.service.endtool_lifecycle_manager import EndtoolLifecycleManager
from pattern_executors.service.movement_dispatcher import MovementDispatcher


class PatternExecutor(LifecycleNode, ABC):
    """Shared lifecycle node base for all pattern executor implementations."""

    _ENDTOOL_STATE_ACTIVE = 'active'
    _ENDTOOL_STATE_INACTIVE = 'inactive'

    def __init__(self, node_name: str = 'pattern_executor') -> None:
        """Initialise the executor node and declare the shared parameters.

        :param node_name: ROS 2 node name passed to :class:`LifecycleNode`.
        :type node_name: str
        """
        super().__init__(node_name)

        self._config: PatternExecutorConfigDTO | None = None
        self._movement_action_client: ActionClient | None = None
        self._endtool_manager: EndtoolLifecycleManager | None = None
        self._dispatcher: MovementDispatcher | None = None
        self._action_server: ActionServer | None = None
        self._executing_patterns_publisher: Publisher | None = None

        self._is_active = False
        self._is_executing = False
        self._execution_lock = Lock()

        self._action_callback_group = ReentrantCallbackGroup()
        self._movement_callback_group = ReentrantCallbackGroup()
        self._endtool_callback_group = ReentrantCallbackGroup()

        self.declare_parameter(
            'tool_sn',
            '',
            ParameterDescriptor(
                description='End-tool serial number; movement frame is '
                '{tool_sn}_calibrated_tcp.'
            ),
        )
        self.declare_parameter(
            'endtool_node_name',
            '',
            ParameterDescriptor(
                description='Node name of the end tool whose lifecycle this '
                'executor drives.'
            ),
        )
        self.declare_parameter(
            'movement_controller_node_name',
            '/movement_controller/execute_trajectory',
            ParameterDescriptor(
                description='Node name of the movement controller.'
            ),
        )
        self.declare_parameter(
            'movement_server_timeout_s',
            5.0,
            ParameterDescriptor(
                description='Timeout in seconds for movement action server '
                'availability checks.'
            ),
        )
        self.declare_parameter(
            'endtool_service_timeout_s',
            5.0,
            ParameterDescriptor(
                description='Timeout in seconds for end-tool lifecycle and '
                'parameter service calls.'
            ),
        )
        self.declare_parameter(
            'simulated',
            True,
            ParameterDescriptor(
                description='Simulation flag reserved for future Gazebo-world '
                'manipulation; no behavioural effect yet.'
            ),
        )

    # region: configuration helpers
    def _read_base_config_kwargs(self) -> dict:
        """Read the shared executor parameters into a keyword-argument dict."""
        return {
            'tool_sn': self.get_parameter('tool_sn')
            .get_parameter_value().string_value,
            'endtool_node_name': self.get_parameter('endtool_node_name')
            .get_parameter_value().string_value,
            'movement_controller_node_name': self.get_parameter('movement_controller_node_name')
            .get_parameter_value().string_value,
            'movement_server_timeout_s': self.get_parameter(
                'movement_server_timeout_s'
            ).get_parameter_value().double_value,
            'endtool_service_timeout_s': self.get_parameter(
                'endtool_service_timeout_s'
            ).get_parameter_value().double_value,
            'simulated': self.get_parameter('simulated')
            .get_parameter_value().bool_value,
        }

    @abstractmethod
    def _build_config(self) -> PatternExecutorConfigDTO:
        """Build the validated executor configuration DTO from parameters.

        Concrete executors read any additional parameters and construct their
        specific configuration DTO (a :class:`PatternExecutorConfigDTO`
        subclass). Raising :class:`pydantic.ValidationError` fails configure.
        """
        raise NotImplementedError()
    # endregion: configuration helpers

    # region: lifecycle callbacks
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Read parameters, build config, and create client-side resources."""
        self.get_logger().info(f'Configuring from state: {state.label}')
        if super().on_configure(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Failed to configure base lifecycle node.')
            return TransitionCallbackReturn.FAILURE

        try:
            self._config = self._build_config()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f'Failed to build executor config: {error}')
            self._config = None
            return TransitionCallbackReturn.FAILURE

        self._movement_action_client = ActionClient(
            self,
            ExecuteTrajectory,
            self._config.movement_controller_node_name,
            callback_group=self._movement_callback_group,
        )
        self._endtool_manager = EndtoolLifecycleManager(
            self,
            self._config.endtool_node_name,
            self._config.endtool_service_timeout_s,
        )
        self._dispatcher = MovementDispatcher(
            self,
            self._movement_action_client,
            self._config.movement_server_timeout_s,
        )

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate the end tool, verify the movement server, open interfaces."""
        self.get_logger().info(f'Activating from state: {state.label}')
        if super().on_activate(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Failed to activate base lifecycle node.')
            return TransitionCallbackReturn.FAILURE
        try:
            if (
                self._config is None
                or self._movement_action_client is None
                or self._endtool_manager is None
                or self._dispatcher is None
            ):
                self.get_logger().error(
                    'Sanity check failed, executor is not configured. Cannot activate.'
                )
                return TransitionCallbackReturn.FAILURE

            if not self._activate_end_tool():
                return TransitionCallbackReturn.FAILURE

            if not self._movement_action_client.wait_for_server(
                timeout_sec=self._config.movement_server_timeout_s
            ):
                self.get_logger().error('Movement controller action server not available.')
                self._deactivate_end_tool()
                return TransitionCallbackReturn.FAILURE

            try:
                self._on_endtool_activated()
            except Exception as error:  # noqa: BLE001
                self.get_logger().error(f'Failed to create end-tool clients: {error}')
                self._deactivate_end_tool()
                return TransitionCallbackReturn.FAILURE

            self._executing_patterns_publisher = self.create_publisher(
                Bool,
                '~/executing_patterns',
                QoSProfile(
                    depth=1,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=ReliabilityPolicy.RELIABLE,
                ),
            )
            self._publish_executing(False)

            self._action_server = self._create_action_server()
            self._is_active = True

            return TransitionCallbackReturn.SUCCESS
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f'Failed to activate executor: {error}')
            return TransitionCallbackReturn.FAILURE
    

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Close interfaces, abort any in-flight goal, deactivate the end tool."""
        self.get_logger().info(f'Deactivating from state: {state.label}')
        if super().on_deactivate(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Failed to deactivate base lifecycle node.')
            return TransitionCallbackReturn.FAILURE

        self._is_active = False

        if self._action_server is not None:
            self._action_server.destroy()
            self._action_server = None

        with self._execution_lock:
            goal_in_flight = self._is_executing

        if goal_in_flight:
            self.get_logger().warning(
                'Deactivating while a goal is executing; forcing abort cleanup.'
            )
            self._abort_movement_and_cleanup()

        try:
            self._on_endtool_deactivating()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f'Failed to destroy end-tool clients: {error}')

        self._deactivate_end_tool()

        self._publish_executing(False)
        if self._executing_patterns_publisher is not None:
            self.destroy_publisher(self._executing_patterns_publisher)
            self._executing_patterns_publisher = None

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Destroy the movement client and end-tool manager, drop config."""
        self.get_logger().info(f'Cleaning up from state: {state.label}')
        if super().on_cleanup(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Failed to cleanup base lifecycle node.')
            return TransitionCallbackReturn.FAILURE

        if self._movement_action_client is not None:
            self._movement_action_client.destroy()
            self._movement_action_client = None

        if self._endtool_manager is not None:
            self._endtool_manager.destroy()
            self._endtool_manager = None

        self._dispatcher = None
        self._config = None
        self._release_execution_slot()

        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Log the error transition and defer to the base implementation."""
        self.get_logger().error(f'Error occurred in state: {state.label}')
        return super().on_error(state)
    # endregion: lifecycle callbacks

    # region: end-tool lifecycle
    def _activate_end_tool(self) -> bool:
        """Drive the end tool to ``active`` following the activation rules."""
        if self._endtool_manager is None:
            self.get_logger().error('End-tool manager is not initialized; activation failed.')
            return False

        current_state = self._endtool_manager.get_node_state()
        if current_state is None:
            self.get_logger().error(
                'Could not read end-tool lifecycle state; activation failed.'
            )
            return False

        if current_state == self._ENDTOOL_STATE_ACTIVE:
            self.get_logger().warning(
                'End tool is already active; continuing without transition.'
            )
            return True

        if current_state != self._ENDTOOL_STATE_INACTIVE:
            self.get_logger().error(
                f"End tool is in '{current_state}'; this executor only activates "
                f"an 'inactive' end tool. Activation failed."
            )
            return False

        if not self._endtool_manager.activate_node():
            self.get_logger().error('Failed to activate the end tool.')
            return False

        return True

    def _deactivate_end_tool(self) -> None:
        """Deactivate the end tool when it is currently active."""
        if self._endtool_manager is None:
            return

        current_state = self._endtool_manager.get_node_state()
        if current_state == self._ENDTOOL_STATE_ACTIVE:
            if not self._endtool_manager.deactivate_node():
                self.get_logger().error('Failed to deactivate the end tool.')
    # endregion: end-tool lifecycle

    # region: concurrency and status helpers
    def _reserve_execution_slot(self) -> bool:
        """Atomically reserve the single execution slot if idle and active.

        :returns: ``True`` if the slot was reserved; ``False`` if the executor
            is inactive or already executing a goal.
        """
        with self._execution_lock:
            if not self._is_active or self._is_executing:
                return False
            self._is_executing = True
            return True

    def _release_execution_slot(self) -> None:
        """Release the execution slot so a new goal can be accepted."""
        with self._execution_lock:
            self._is_executing = False

    def _publish_executing(self, value: bool) -> None:
        """Publish the latched ``~/executing_patterns`` status when active."""
        if self._executing_patterns_publisher is not None:
            self._executing_patterns_publisher.publish(Bool(data=value))

    def _abort_movement_and_cleanup(self) -> None:
        """Cancel any in-flight movement goal and run type-specific cleanup."""
        if self._dispatcher is not None:
            try:
                self._dispatcher.cancel_active_goal()
            except Exception as error:  # noqa: BLE001
                self.get_logger().error(f'Failed to cancel movement goal: {error}')

        try:
            self._abort_cleanup()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f'Abort cleanup failed: {error}')
    # endregion: concurrency and status helpers

    # region: subclass hooks
    @abstractmethod
    def _create_action_server(self) -> ActionServer:
        """Create and return the type-specific action server.

        The server is created only on activation and destroyed on deactivation.
        Implementations must register the goal, cancel, and execute callbacks
        (goal acceptance must call :meth:`_reserve_execution_slot`).
        """
        raise NotImplementedError()

    @abstractmethod
    def _on_endtool_activated(self) -> None:
        """Create type-specific end-tool clients after the end tool is active."""
        raise NotImplementedError()

    @abstractmethod
    def _on_endtool_deactivating(self) -> None:
        """Destroy type-specific end-tool clients before deactivating the tool."""
        raise NotImplementedError()

    @abstractmethod
    def _abort_cleanup(self) -> None:
        """Perform type-specific safe-stop on abort, cancel, or failure."""
        raise NotImplementedError()
    # endregion: subclass hooks
