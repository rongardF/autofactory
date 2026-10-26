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

"""Concrete dispensing pattern executor node.

Executes :class:`DispensingPattern` goals: it drives the volumetric dispenser
end tool, runs the pattern movements through ``movement_controller`` while
firing feedback-timed valve/wait actions, and reports per-element and
per-pattern dispensing metrics.
"""

from __future__ import annotations

from copy import deepcopy
from threading import Lock
from time import monotonic, sleep

from rcl_interfaces.msg import ParameterDescriptor, ParameterType, ParameterValue
from rclpy import init, ok, shutdown
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.client import Client
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.time import Time

from builtin_interfaces.msg import Time as RosTime
from endtools.srv import StartDispensing, StopDispensing
from pattern_executors.action import ExecuteDispensingPattern
from pattern_executors.interface.pattern_executor import PatternExecutor
from pattern_executors.model.dispensing_executor_config_dto import (
    DispensingExecutorConfigDTO,
)
from pattern_executors.model.pattern_executor_config_dto import PatternExecutorConfigDTO
from pattern_executors.msg import (
    DispensedElementMetric,
    DispensingMovement,
    DispensingPattern,
    DispensingPatternMetric,
    MovementPath,
    PatternAction,
)
from pattern_executors.util.units_converter import cc_to_m3, m3_s_to_cc_s




class DispensingPatternExecutor(PatternExecutor):
    """Pattern executor for volumetric dispensing operations."""

    _FLOWRATE_PARAMETER_NAME = 'flowrate'
    _FLOWRATE_COMPARISON_DIGITS = 6
    _POSITION_EPSILON_M = 1e-6

    def __init__(self, node_name: str = 'dispensing_pattern_executor') -> None:
        """Initialise the dispensing executor and declare dispensing parameters.

        :param node_name: ROS 2 node name passed to :class:`PatternExecutor`.
        :type node_name: str
        """
        super().__init__(node_name)

        self.declare_parameter(
            'dispense_start_service_name',
            'dispense_start',
            ParameterDescriptor(
                description='Relative service name that opens the dispenser '
                'valve (open_valve).'
            ),
        )
        self.declare_parameter(
            'dispense_stop_service_name',
            'dispense_stop',
            ParameterDescriptor(
                description='Relative service name that closes the dispenser '
                'valve (close_valve).'
            ),
        )

        self._dispense_start_client: Client | None = None
        self._dispense_stop_client: Client | None = None

        self._metrics_lock = Lock()
        self._current_element_metrics: list[DispensedElementMetric] = []
        self._current_open_element_id: str | None = None
        self._last_error = ''

    # region: configuration
    def _build_config(self) -> PatternExecutorConfigDTO:
        """Build the validated dispensing executor configuration DTO."""
        return DispensingExecutorConfigDTO(
            **self._read_base_config_kwargs(),
            dispense_start_service_name=self.get_parameter(
                'dispense_start_service_name'
            ).get_parameter_value().string_value,
            dispense_stop_service_name=self.get_parameter(
                'dispense_stop_service_name'
            ).get_parameter_value().string_value,
        )

    @property
    def _dispensing_config(self) -> DispensingExecutorConfigDTO:
        """Return the configuration narrowed to the dispensing DTO type."""
        assert isinstance(self._config, DispensingExecutorConfigDTO)
        return self._config
    # endregion: configuration

    # region: subclass hooks
    def _create_action_server(self) -> ActionServer:
        """Create the ExecuteDispensingPattern action server."""
        return ActionServer(
            node=self,
            action_type=ExecuteDispensingPattern,
            action_name='~/execute_dispensing_pattern',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._action_callback_group,
        )

    def _on_endtool_activated(self) -> None:
        """Create the dispenser start/stop service clients."""
        config = self._dispensing_config
        prefix = (
            config.endtool_node_name
            if config.endtool_node_name.startswith('/')
            else f'/{config.endtool_node_name}'
        )
        self._dispense_start_client = self.create_client(
            StartDispensing,
            f'{prefix}/{config.dispense_start_service_name}',
            callback_group=self._endtool_callback_group,
        )
        self._dispense_stop_client = self.create_client(
            StopDispensing,
            f'{prefix}/{config.dispense_stop_service_name}',
            callback_group=self._endtool_callback_group,
        )

    def _on_endtool_deactivating(self) -> None:
        """Destroy the dispenser start/stop service clients."""
        if self._dispense_start_client is not None:
            self.destroy_client(self._dispense_start_client)
            self._dispense_start_client = None
        if self._dispense_stop_client is not None:
            self.destroy_client(self._dispense_stop_client)
            self._dispense_stop_client = None

    def _abort_cleanup(self) -> None:
        """Force the dispenser valve closed so dispensing cannot leak."""
        if self._dispense_stop_client is None:
            return
        response = self._call_service(
            self._dispense_stop_client,
            StopDispensing.Request(),
            'dispense_stop (abort cleanup)',
        )
        if response is None:
            self.get_logger().error(
                'Abort cleanup could not confirm the dispenser valve is closed.'
            )
    # endregion: subclass hooks

    # region: action callbacks
    def _goal_callback(self, goal_request: ExecuteDispensingPattern.Goal) -> GoalResponse:
        """Validate the goal and reserve the single execution slot."""
        validation_error = self._validate_goal(goal_request)
        if validation_error is not None:
            self.get_logger().error(f'Rejecting invalid goal: {validation_error}')
            return GoalResponse.REJECT

        if not self._reserve_execution_slot():
            self.get_logger().warning(
                'Rejecting goal: executor is busy or not active.'
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """Accept cancellation and cancel any in-flight movement goal."""
        self.get_logger().info('Cancel requested for dispensing goal.')
        if self._dispatcher is not None:
            self._dispatcher.cancel_active_goal()
        return CancelResponse.ACCEPT

    def _execute_callback(
        self,
        goal_handle: ServerGoalHandle,
    ) -> ExecuteDispensingPattern.Result:
        """Execute every pattern in the goal and build the aggregate result."""
        goal: ExecuteDispensingPattern.Goal = goal_handle.request
        self._publish_executing(True)
        start_time = self.get_clock().now()
        metrics: list[DispensingPatternMetric] = []
        error_message = ''
        failed = False

        try:
            for pattern in goal.patterns:
                if goal_handle.is_cancel_requested:
                    failed = True
                    error_message = 'Goal cancelled by client request'
                    break

                self._prepare_pattern(pattern)
                pattern_metric = self._execute_pattern(
                    pattern, goal_handle
                )
                if pattern_metric is None:
                    failed = True
                    error_message = self._last_error
                    break

                metrics.append(pattern_metric)
                self._publish_pattern_feedback(goal_handle, pattern_metric)
        except Exception as error:  # noqa: BLE001
            failed = True
            error_message = str(error)

        if failed:
            self.get_logger().error(
                f'Dispensing goal did not complete: {error_message}'
            )
            self._abort_movement_and_cleanup()

        self._publish_executing(False)
        self._release_execution_slot()

        result = ExecuteDispensingPattern.Result()
        result.operation_id = goal.operation_id
        result.execution_time_s = self._elapsed_s(start_time)
        result.metrics = metrics

        if not failed:
            result.success = True
            result.error_message = ''
            goal_handle.succeed()
            return result

        result.success = False
        result.error_message = error_message
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result
    # endregion: action callbacks

    # region: execution
    def _prepare_pattern(self, pattern: DispensingPattern) -> None:
        """Reconfigure the end-tool flow rate for this pattern when it differs."""
        assert self._endtool_manager is not None

        target_cc_s = m3_s_to_cc_s(pattern.flowrate_m3_s)
        current_parameter = self._endtool_manager.get_parameter(
            self._FLOWRATE_PARAMETER_NAME
        )
        if current_parameter is None:
            raise RuntimeError('Could not read end-tool flowrate parameter.')

        current_cc_s = current_parameter.double_value
        if round(current_cc_s, self._FLOWRATE_COMPARISON_DIGITS) == round(
            target_cc_s, self._FLOWRATE_COMPARISON_DIGITS
        ):
            return

        self.get_logger().info(
            f'Reconfiguring end-tool flowrate to {target_cc_s} cc/s.'
        )
        if not self._endtool_manager.deactivate_node():
            raise RuntimeError('Failed to deactivate end tool for flowrate change.')
        if not self._endtool_manager.cleanup_node():
            raise RuntimeError('Failed to clean up end tool for flowrate change.')

        flowrate_value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE,
            double_value=target_cc_s,
        )
        if not self._endtool_manager.set_parameter(
            self._FLOWRATE_PARAMETER_NAME, flowrate_value
        ):
            raise RuntimeError('Failed to set end-tool flowrate parameter.')
        if not self._endtool_manager.configure_node():
            raise RuntimeError('Failed to configure end tool after flowrate change.')
        if not self._endtool_manager.activate_node():
            raise RuntimeError('Failed to re-activate end tool after flowrate change.')
        if self._endtool_manager.get_node_state() != 'active':
            raise RuntimeError(
                'End tool did not reach active after flowrate reconfigure.'
            )

    def _execute_pattern(
        self,
        pattern: DispensingPattern,
        goal_handle: ServerGoalHandle,
    ) -> DispensingPatternMetric | None:
        """Run one pattern's approach and movements, returning its metric.

        :returns: The aggregated pattern metric on success, or ``None`` when the
            pattern failed or was cancelled (``self._last_error`` is set).
        """
        assert self._dispatcher is not None

        with self._metrics_lock:
            self._current_element_metrics = []
            self._current_open_element_id = None

        pattern_start = self.get_clock().now()
        tool_frame = self._dispensing_config.calibrated_tcp_frame

        approach_result = self._dispatcher.execute_batches(
            [self._build_approach_movement(pattern)],
            tool_frame,
            self._fire_action,
        )
        if not approach_result.success:
            self._last_error = (
                f'Approach move failed: {approach_result.error_message}'
            )
            return None

        if goal_handle.is_cancel_requested:
            self._last_error = 'Goal cancelled by client request'
            return None

        movement_result = self._dispatcher.execute_batches(
            list(pattern.movements),
            tool_frame,
            self._fire_action,
        )
        if not movement_result.success:
            self._last_error = (
                f'Movement execution failed: {movement_result.error_message}'
            )
            return None

        with self._metrics_lock:
            element_metrics = list(self._current_element_metrics)

        return self._build_pattern_metric(
            pattern, element_metrics, self._elapsed_s(pattern_start)
        )

    def _build_approach_movement(
        self, pattern: DispensingPattern
    ) -> DispensingMovement:
        """Build a valve-closed PTP approach movement to the pattern origin."""
        movement = DispensingMovement()
        path = MovementPath()
        path.path_id = f'{pattern.pattern_id}'
        path.motion_type = MovementPath.MOTION_TYPE_PTP
        path.target_pose = deepcopy(pattern.start_pose)
        movement.path = path
        movement.actions = []
        return movement

    def _fire_action(
        self,
        action: PatternAction,
        movement: DispensingMovement,
        motion_stamp: RosTime,
    ) -> None:
        """Execute a single pattern action (open/close valve or wait).

        :param motion_stamp: Instant the movement_controller reported the phase
            transition at. Forwarded to the dispenser so it resolves the TCP pose
            at that instant rather than whenever the service call happens to run.
        """
        if action.type == PatternAction.TYPE_OPEN_VALVE:
            self._open_valve(movement, motion_stamp)
        elif action.type == PatternAction.TYPE_CLOSE_VALVE:
            self._close_valve(movement, motion_stamp)
        elif action.type == PatternAction.TYPE_WAIT:
            if action.wait_time_s > 0.0:
                # Node clock so the wait honours use_sim_time.
                completed = self.get_clock().sleep_for(
                    Duration(seconds=action.wait_time_s)
                )
                if not completed:
                    raise RuntimeError(
                        'Wait action interrupted (shutdown or clock source change).'
                    )
        else:
            raise RuntimeError(f"Unknown pattern action type '{action.type}'.")

    def _open_valve(
        self, movement: DispensingMovement, motion_stamp: RosTime
    ) -> None:
        """Start dispensing and record the open element's originating path id."""
        if self._dispense_start_client is None:
            raise RuntimeError('Dispenser start client is not available.')

        request = StartDispensing.Request()
        request.pose_stamp = motion_stamp
        response = self._call_service(
            self._dispense_start_client, request, 'dispense_start'
        )
        if response is None or not response.success:
            raise RuntimeError('Failed to start dispensing.')

        with self._metrics_lock:
            self._current_open_element_id = movement.path.path_id

    def _close_valve(
        self, movement: DispensingMovement, motion_stamp: RosTime
    ) -> None:
        """Stop dispensing and record the completed element metric."""
        if self._dispense_stop_client is None:
            raise RuntimeError('Dispenser stop client is not available.')

        request = StopDispensing.Request()
        request.pose_stamp = motion_stamp
        response = self._call_service(
            self._dispense_stop_client, request, 'dispense_stop'
        )
        if response is None or not response.success:
            detail = response.error_message if response is not None else 'no response'
            raise RuntimeError(f'Failed to stop dispensing: {detail}')

        with self._metrics_lock:
            element_metric = DispensedElementMetric()
            element_metric.element_id = (
                self._current_open_element_id or movement.path.path_id
            )
            element_metric.dispensed_volume_m3 = cc_to_m3(response.dispensed_volume)
            element_metric.dispensing_time_s = response.duration_seconds
            self._current_element_metrics.append(element_metric)
            self._current_open_element_id = None

    def _build_pattern_metric(
        self,
        pattern: DispensingPattern,
        element_metrics: list[DispensedElementMetric],
        pattern_execution_time_s: float,
    ) -> DispensingPatternMetric:
        """Aggregate element metrics into one pattern metric."""
        pattern_metric = DispensingPatternMetric()
        pattern_metric.pattern_id = pattern.pattern_id
        pattern_metric.dispensed_volume_m3 = sum(
            element.dispensed_volume_m3 for element in element_metrics
        )
        pattern_metric.pattern_execution_time_s = pattern_execution_time_s
        pattern_metric.element_metrics = element_metrics
        return pattern_metric

    def _publish_pattern_feedback(
        self,
        goal_handle: ServerGoalHandle,
        pattern_metric: DispensingPatternMetric,
    ) -> None:
        """Publish per-pattern completion feedback."""
        feedback = ExecuteDispensingPattern.Feedback()
        feedback.pattern_id = pattern_metric.pattern_id
        feedback.execution_time_s = pattern_metric.pattern_execution_time_s
        feedback.metric = pattern_metric
        goal_handle.publish_feedback(feedback)
    # endregion: execution

    # region: validation and helpers
    def _validate_goal(self, goal: ExecuteDispensingPattern.Goal) -> str | None:
        """Validate goal contents; return an error message or ``None`` if valid.

        A movement is treated as zero-length (the only valid carrier of a
        blocking ``wait`` action) when its target position equals the preceding
        reference position within the same goal frame: the pattern start pose
        for the first movement, or the previous movement's target otherwise.
        """
        if len(goal.patterns) == 0:
            return 'goal contains no patterns'

        valid_types = {
            PatternAction.TYPE_OPEN_VALVE,
            PatternAction.TYPE_CLOSE_VALVE,
            PatternAction.TYPE_WAIT,
        }
        valid_timings = {PatternAction.TIMING_START, PatternAction.TIMING_END}

        for pattern in goal.patterns:
            reference_position = pattern.start_pose.pose.position
            for movement in pattern.movements:
                movement_position = movement.path.target_pose.pose.position
                has_wait = False
                for action in movement.actions:
                    if action.type not in valid_types:
                        return f"invalid action type '{action.type}'"
                    if action.timing not in valid_timings:
                        return f"invalid action timing '{action.timing}'"
                    if action.type == PatternAction.TYPE_WAIT:
                        has_wait = True
                if has_wait and not self._positions_equal(
                    reference_position, movement_position
                ):
                    return (
                        f"wait action on non-zero-length movement "
                        f"'{movement.path.path_id}'"
                    )
                reference_position = movement_position
        return None

    def _positions_equal(self, first, second) -> bool:
        """Return whether two geometry positions coincide within an epsilon."""
        return (
            abs(first.x - second.x) <= self._POSITION_EPSILON_M
            and abs(first.y - second.y) <= self._POSITION_EPSILON_M
            and abs(first.z - second.z) <= self._POSITION_EPSILON_M
        )

    def _elapsed_s(self, start: Time) -> float:
        """Return seconds elapsed since ``start`` on the node clock.

        Execution-time metrics use the node clock so they honour
        ``use_sim_time``; timeout deadlines intentionally stay on the wall clock.
        """
        return (self.get_clock().now() - start).nanoseconds / 1e9

    def _call_service(self, client: Client, request, description: str):
        """Call a service asynchronously and wait on the wall clock for a reply."""
        timeout = self._dispensing_config.endtool_service_timeout_s
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f'Service not available for: {description}')
            return None

        future = client.call_async(request)
        deadline = monotonic() + timeout
        while not future.done():
            if monotonic() > deadline:
                self.get_logger().error(f'Service call timed out for: {description}')
                return None
            sleep(0.01)

        return future.result()
    # endregion: validation and helpers


def main(args=None) -> None:
    """Entry point for the ``dispensing_pattern_executor`` executable.

    Initialises rclpy, creates a :class:`DispensingPatternExecutor`, and spins
    it with a :class:`MultiThreadedExecutor` so action execution, movement
    feedback, and end-tool service calls run concurrently without deadlock.

    :param args: Optional command-line arguments forwarded to :func:`rclpy.init`.
    :type args: list[str] | None
    """
    init(args=args)
    node = DispensingPatternExecutor()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if ok():
            shutdown()


if __name__ == '__main__':
    main()
