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
"""URMovementController — ROS2 LifecycleNode for UR robot trajectory execution."""

from math import pi
from time import sleep
from threading import Lock, Event
from asyncio import Future

from rclpy import init, shutdown
from pydantic import ValidationError
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.action.client import CancelGoal, ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.publisher import Publisher

from std_msgs.msg import String
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.action import (
    ExecuteTrajectory_GetResult_Response as MoveitExecuteTrajectoryResponse,
    ExecuteTrajectory as MoveItExecuteTrajectory
)

from movement_controller.action import ExecuteTrajectory
from movement_controller.exceptions import (
    ExecutionFailedError,
)
from movement_controller.enums.feedback_status_enum import FeedbackStatusEnum
from movement_controller.models import TrajectoryGoalDTO, ConstraintConfigDTO
from movement_controller.services.planning_coordinator import PlanningCoordinator
from movement_controller.utils.trajectory_grouper import TrajectoryGrouper


class MovementController(LifecycleNode):
    """LifecycleNode that exposes an ExecuteTrajectory action server for robot arms."""

    def __init__(self, node_name: str = 'movement_controller') -> None:
        """Initialise the MovementController lifecycle node.

        Declares all ROS 2 parameters with defaults and descriptions and
        creates the internal thread-safety primitives used to serialise
        concurrent goal and cancel callbacks.

        :param node_name: ROS 2 node name passed to :class:`rclpy.lifecycle.LifecycleNode`.
        :type node_name: str
        """
        super().__init__(node_name)
        self._action_server: ActionServer | None = None
        self._execute_trajectory_client: ActionClient | None = None
        self._is_executing: bool = False
        self._goal_handle: ServerGoalHandle | None = None
        self._goal_handle_lock: Lock = Lock()
        self._executing_lock: Lock = Lock()
        self._planner_service: PlanningCoordinator | None = None
        self._trajectory_goal: TrajectoryGoalDTO | None = None
        self._constraint_config: ConstraintConfigDTO | None = None
        self._cancellation_pub: Publisher | None = None

        # region: parameters
        self.declare_parameter(
            'moveit_group_name',
            'ur_manipulator',
            ParameterDescriptor(description='MoveIt2 planning group name'),
        )
        self.declare_parameter(
            'moveit_connection_timeout',
            10.0,
            ParameterDescriptor(description='Seconds to wait for MoveItPy to connect before failing on_configure'),
        )

        # Workspace bounding box constraint parameters
        self.declare_parameter(
            'constraints.workspace.x_min',
            -1e9,
            ParameterDescriptor(description='Workspace bounding box x lower bound (m). Sentinel -1e9 = unconstrained.'),
        )
        self.declare_parameter(
            'constraints.workspace.x_max',
            1e9,
            ParameterDescriptor(description='Workspace bounding box x upper bound (m). Sentinel +1e9 = unconstrained.'),
        )
        self.declare_parameter(
            'constraints.workspace.y_min',
            -1e9,
            ParameterDescriptor(description='Workspace bounding box y lower bound (m). Sentinel -1e9 = unconstrained.'),
        )
        self.declare_parameter(
            'constraints.workspace.y_max',
            1e9,
            ParameterDescriptor(description='Workspace bounding box y upper bound (m). Sentinel +1e9 = unconstrained.'),
        )
        self.declare_parameter(
            'constraints.workspace.z_min',
            -1e9,
            ParameterDescriptor(description='Workspace bounding box z lower bound (m). Sentinel -1e9 = unconstrained.'),
        )
        self.declare_parameter(
            'constraints.workspace.z_max',
            1e9,
            ParameterDescriptor(description='Workspace bounding box z upper bound (m). Sentinel +1e9 = unconstrained.'),
        )

        # Orientation constraint parameters
        self.declare_parameter(
            'constraints.orientation.tolerance_x',
            pi * 2,
            ParameterDescriptor(description='Orientation tolerance around x axis (radians). Default 2π = unconstrained.'),
        )
        self.declare_parameter(
            'constraints.orientation.tolerance_y',
            pi * 2,
            ParameterDescriptor(description='Orientation tolerance around y axis (radians). Default 2π = unconstrained.'),
        )
        self.declare_parameter(
            'constraints.orientation.tolerance_z',
            pi * 2,
            ParameterDescriptor(description='Orientation tolerance around z axis (radians). Default 2π = unconstrained.'),
        )

        # Speed/acceleration cap parameters
        self.declare_parameter(
            'constraints.max_cartesian_speed',
            0.0,
            ParameterDescriptor(description='Node-level max cartesian speed cap (0..1 ratio, 0.0 = unconstrained). Goals with any path.cartesian_speed exceeding this are rejected.'),
        )
        self.declare_parameter(
            'constraints.max_cartesian_acceleration',
            0.0,
            ParameterDescriptor(description='Node-level max cartesian acceleration cap (0..1 ratio, 0.0 = unconstrained). Goals with any path.cartesian_acceleration exceeding this are rejected.'),
        )
        self.declare_parameter(
            'constraints.max_joint_speed',
            0.0,
            ParameterDescriptor(description='Node-level max joint speed cap (0..1 ratio, 0.0 = unconstrained). Goals with any path.joint_speed exceeding this are rejected.'),
        )
        self.declare_parameter(
            'constraints.max_joint_acceleration',
            0.0,
            ParameterDescriptor(description='Node-level max joint acceleration cap (0..1 ratio, 0.0 = unconstrained). Goals with any path.joint_acceleration exceeding this are rejected.'),
        )

        # OMPL (collision-aware PTP) tuning parameters
        self.declare_parameter(
            'ompl_planning_time',
            5.0,
            ParameterDescriptor(description='OMPL allowed_planning_time in seconds for collision-aware PTP planning.'),
        )
        self.declare_parameter(
            'ompl_planning_attempts',
            10,
            ParameterDescriptor(description='OMPL num_planning_attempts for collision-aware PTP planning.'),
        )
        # endregion: parameters

        # region: callback groups
        self._server_callback_group = ReentrantCallbackGroup()
        self._client_callback_group = ReentrantCallbackGroup()
        # endregion: callback groups

    # region: lifecycle callbacks
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Read and validate parameters, then instantiate the :class:`PlanningCoordinator`.

        Reads all constraint parameters declared in :meth:`__init__`, builds a
        :class:`~movement_controller.models.ConstraintConfigDTO`, and passes it
        to the planner service.  On any validation or configuration error the
        planner service reference is cleared and ``FAILURE`` is returned so the
        lifecycle manager can retry.

        :param state: Lifecycle state being transitioned from.
        :type state: LifecycleState
        :returns: ``SUCCESS`` on successful configuration, ``FAILURE`` otherwise.
        :rtype: TransitionCallbackReturn
        """
        self.get_logger().info(f'Configuring from state: {state.label}')

        # Read and validate constraint parameters
        try:
            moveit_group_name = self.get_parameter('moveit_group_name').get_parameter_value().string_value
            self._planner_service = PlanningCoordinator(node=self, moveit_group_name=moveit_group_name)
            self.get_logger().info('PlanningCoordinator initialised successfully')

            x_min = self.get_parameter('constraints.workspace.x_min').get_parameter_value().double_value
            x_max = self.get_parameter('constraints.workspace.x_max').get_parameter_value().double_value
            y_min = self.get_parameter('constraints.workspace.y_min').get_parameter_value().double_value
            y_max = self.get_parameter('constraints.workspace.y_max').get_parameter_value().double_value
            z_min = self.get_parameter('constraints.workspace.z_min').get_parameter_value().double_value
            z_max = self.get_parameter('constraints.workspace.z_max').get_parameter_value().double_value
            tol_x = self.get_parameter('constraints.orientation.tolerance_x').get_parameter_value().double_value
            tol_y = self.get_parameter('constraints.orientation.tolerance_y').get_parameter_value().double_value
            tol_z = self.get_parameter('constraints.orientation.tolerance_z').get_parameter_value().double_value
            max_cart_speed = self.get_parameter('constraints.max_cartesian_speed').get_parameter_value().double_value
            max_cart_accel = self.get_parameter('constraints.max_cartesian_acceleration').get_parameter_value().double_value
            max_joint_speed = self.get_parameter('constraints.max_joint_speed').get_parameter_value().double_value
            max_joint_accel = self.get_parameter('constraints.max_joint_acceleration').get_parameter_value().double_value
            self.get_logger().info('Parameters read successfully, building constraint configuration DTO')

            dto = ConstraintConfigDTO(
                x_min=x_min,
                x_max=x_max,
                y_min=y_min,
                y_max=y_max,
                z_min=z_min,
                z_max=z_max,
                orientation_tolerance_x=tol_x,
                orientation_tolerance_y=tol_y,
                orientation_tolerance_z=tol_z,
                max_cartesian_speed=max_cart_speed,
                max_cartesian_acceleration=max_cart_accel,
                max_joint_speed=max_joint_speed,
                max_joint_acceleration=max_joint_accel
            )

            self.get_logger().info('Constraint configuration DTO built and validated successfully')
            self._constraint_config = dto
            self._planner_service.set_constraints(dto)
            self.get_logger().info('Constraint configuration applied successfully')

            ompl_planning_time = self.get_parameter('ompl_planning_time').get_parameter_value().double_value
            ompl_planning_attempts = self.get_parameter('ompl_planning_attempts').get_parameter_value().integer_value
            self._planner_service.set_ompl_tuning(ompl_planning_time, ompl_planning_attempts)
            self.get_logger().info(
                f'OMPL tuning applied: planning_time={ompl_planning_time}s, '
                f'planning_attempts={ompl_planning_attempts}'
            )
        except ValidationError as e:
            self.get_logger().error(f'Constraint parameter validation failed: {e}')
            self._planner_service = None  # ensure planner service is not used if validation fails
            return TransitionCallbackReturn.FAILURE
        except Exception as e:
            self.get_logger().error(f'Unknown error while configuring: {e}')
            self._planner_service = None  # ensure planner service is not used if config fails
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate the node: start the planner service, create the action server and clients.

        Activates :class:`PlanningCoordinator` and waits for the
        ``plan_sequence_path`` service using the ``moveit_connection_timeout``
        parameter.  On success, creates the
        ``movement_controller/execute_trajectory`` action server and the
        ``execute_trajectory`` MoveIt 2 action client.  Any failure tears down
        all partially-created resources before returning ``FAILURE``.

        :param state: Lifecycle state being transitioned from.
        :type state: LifecycleState
        :returns: ``SUCCESS`` if all components initialised successfully,
            ``FAILURE`` otherwise.
        :rtype: TransitionCallbackReturn
        """
        self.get_logger().info(f'Activating from state: {state.label}')
        try:
            if self._planner_service is not None:
                self._planner_service.on_activate()
            else:
                self.get_logger().error('PlanningCoordinator not initialised during on_activate')
                return TransitionCallbackReturn.FAILURE
            self.get_logger().info(f'PlanningCoordinator activated successfully')
        
            timeout = self.get_parameter('moveit_connection_timeout').get_parameter_value().double_value
            self.get_logger().debug(f'Waiting for plan_sequence_path service (timeout={timeout}s)')
            if self._planner_service is None or not self._planner_service.wait_for_service(timeout_sec=timeout):
                self.get_logger().error(
                    f'/plan_sequence_path service not available after {timeout}s — '
                    'is move_group launched with pilz_industrial_motion_planner/MoveGroupSequenceService capability?'
                )
                return TransitionCallbackReturn.FAILURE
            
            with self._goal_handle_lock:
                self._goal_handle = None  # reset goal handle on activation
            self.get_logger().debug('plan_sequence_path service is available')

            # region: topics and subscriptions
            self._cancellation_pub = self.create_publisher(
                String, "/trajectory_execution_event", 1
            )
            # endregion: topics and subscriptions
            
            # region: action server and client setup
            self._action_server = ActionServer(
                self,
                ExecuteTrajectory,
                "movement_controller/execute_trajectory",
                execute_callback=self._execute_callback,
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback,
                callback_group=self._server_callback_group,
            )
            self.get_logger().info(f'Action server created at: movement_controller/execute_trajectory')

            self._execute_trajectory_client = ActionClient(
                node=self,
                action_type=MoveItExecuteTrajectory, 
                action_name='execute_trajectory',
                callback_group=self._client_callback_group
            )
            self.get_logger().info(f'Execute trajectory client created at: execute_trajectory')
            # endregion: action server and client setup
        except Exception as e:
            self.get_logger().error(f'Unknown error during activation: {e}')
            if self._action_server is not None:
                self._action_server.destroy()
                self._action_server = None
                self.get_logger().info('Action server destroyed due to activation failure')
            if self._execute_trajectory_client is not None:
                self._execute_trajectory_client.destroy()
                self._execute_trajectory_client = None
                self.get_logger().info('Execute trajectory client destroyed due to activation failure')
            if self._planner_service is not None:
                self._planner_service.on_deactivate()  # ensure planner service is deactivated if activation fails
                self.get_logger().info('PlanningCoordinator deactivated due to activation failure')
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate the node: stop the planner, destroy the action server and clients.

        Cancels any in-progress planning sequence, publishes a stop signal on
        the ``/trajectory_execution_event`` topic, then destroys the action
        client, action server, and cancellation publisher created during
        :meth:`on_activate`.

        :param state: Lifecycle state being transitioned from.
        :type state: LifecycleState
        :returns: Always returns ``SUCCESS``.
        :rtype: TransitionCallbackReturn
        """
        self.get_logger().info(f'Deactivating from state: {state.label}')
        if self._planner_service is not None:
            self.get_logger().debug('Cancelling planner service and deactivating during on_deactivate')
            self._planner_service.cancel()  # ensure any ongoing planning is stopped if deactivated mid-plan
            self._planner_service.on_deactivate()
        if self._cancellation_pub is not None:
            self._cancellation_pub.publish(String(data="stop"))  # send stop signal just in case
            self.destroy_publisher(self._cancellation_pub)
            self._cancellation_pub = None
            self.get_logger().info('Cancellation publisher destroyed successfully')
        # give time for in-flight messages to be processed. Use a wall-clock
        # sleep here (not self.get_clock().sleep_for): the node clock follows
        # use_sim_time, and a sim-time sleep inside this transition callback
        # deadlocks because the /clock subscription that advances sim time
        # shares this node's default mutually-exclusive callback group and
        # cannot run while on_deactivate is blocked.
        sleep(0.5)
        if self._execute_trajectory_client is not None:
            self._execute_trajectory_client.destroy()
            self._execute_trajectory_client = None
            self.get_logger().info('Execute trajectory client destroyed successfully')
        if self._action_server is not None:
            self._action_server.destroy()
            self._action_server = None
            self.get_logger().info('Action server destroyed successfully')
        
        with self._executing_lock:
            self._is_executing = False

        self._trajectory_goal = None

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Release held references to allow garbage collection.

        Clears the planner service and cached trajectory goal so the node can
        be re-configured cleanly via a subsequent :meth:`on_configure` call.

        :param state: Lifecycle state being transitioned from.
        :type state: LifecycleState
        :returns: Always returns ``SUCCESS``.
        :rtype: TransitionCallbackReturn
        """
        self.get_logger().info(f'Cleaning up from state: {state.label}')
        self._planner_service = None
        self._trajectory_goal = None
        return TransitionCallbackReturn.SUCCESS
    
    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handle lifecycle error transitions.

        Logs the state in which the error occurred and delegates to the
        base-class :meth:`rclpy.lifecycle.LifecycleNode.on_error` implementation.

        :param state: Lifecycle state in which the error occurred.
        :type state: LifecycleState
        :returns: Return value of the base-class ``on_error`` handler.
        :rtype: TransitionCallbackReturn
        """
        self.get_logger().error(f'Error occurred in state: {state.label}')
        return super().on_error(state)

    # endregion: lifecycle callbacks

    # region: private methods
    def _goal_abort(self, goal_handle: ServerGoalHandle) -> None:
        """Helper method to abort the current goal and reset executing state; 
        used in multiple places in _execute_callback to avoid code duplication.
        """
        try:
            goal_handle.abort()
        except Exception:
            self.get_logger().warning('Failed to abort goal handle - it may already be in a terminal state')
            pass
    
    def _call_with_cancellation(self, goal_handle: ServerGoalHandle, goal: MoveItExecuteTrajectory.Goal) -> MoveitExecuteTrajectoryResponse:
        # utility function and event to block until goal request has been responded
        event = Event()
        def unblock(future: Future):
            nonlocal event
            self.get_logger().debug("Future done callback called, unblocking waiting thread")
            event.set()

        # sanity check - we have checked this already so here it is expected to be not None
        if self._execute_trajectory_client is None:
            raise RuntimeError('execute trajectory client should be intialized at this point')
        
        # send goal
        if goal_handle.is_cancel_requested is False:
            self.get_logger().debug("Sending goal to execute_trajectory action server")
            send_goal_future = self._execute_trajectory_client.send_goal_async(goal)
            send_goal_future.add_done_callback(unblock)

            # wait for goal request response and check for exceptions
            self.get_logger().debug("Goal sent, waiting to be accepted")
            event.wait()
            self.get_logger().debug("Goal response received, checking for exceptions")
            exception = send_goal_future.exception()
            if exception is not None:
                raise exception

            # get the goal handle for the sent goal - this is used to cancel and get result
            moveit_goal_handle: ClientGoalHandle = send_goal_future.result()  # type: ignore
            self.get_logger().debug("Goal handle acquired")

            # clear utility event
            event.clear()

            # get the result future
            result_future: Future = moveit_goal_handle.get_result_async()
            result_future.add_done_callback(unblock)

            # wait for goal to complete and check for exceptions
            self.get_logger().debug("Waiting for goal result")
            event.wait()
            exception = result_future.exception()
            if exception is not None:
                raise exception
            
            # return goal result
            self.get_logger().debug("Goal result received")
            return result_future.result()
        else:
            raise ExecutionFailedError('goal was cancelled by the client')

    def _generate_failure_result(
        self,
        goal_handle: ServerGoalHandle,
        error_message: str,
        completed_ids: list[str]
    ) -> ExecuteTrajectory.Result:
        self.get_logger().debug(f'Generating failure result with error message: {error_message}')
        result = ExecuteTrajectory.Result()
        result.success = False
        result.error_message = error_message
        result.trajectory_paths_completed = completed_ids
        self._goal_abort(goal_handle)
        self._trajectory_goal = None  # reset after execution
        with self._executing_lock:
            self._is_executing = False
        with self._goal_handle_lock:
            self._goal_handle = None  # reset goal handle after execution

        return result
    
    # endregion: private methods

    # region: callbacks
    def _goal_callback(self, goal: ExecuteTrajectory.Goal) -> GoalResponse:
        # Only one goal at a time — check and set atomically to prevent TOCTOU race
        with self._executing_lock:
            if self._is_executing:
                self.get_logger().error('Goal rejected: another goal is already executing')
                return GoalResponse.REJECT
            self._is_executing = True

        # Full goal validation via DTO — reset executing flag on failure
        self.get_logger().debug(f'Received goal request, validating {len(goal.paths)} path(s)')
        try:
            self._trajectory_goal = TrajectoryGoalDTO.from_ros_msg(goal)
            if self._constraint_config is not None:
                self.get_logger().debug('Validating goal against active constraint configuration')
                self._constraint_config.validate_goal(self._trajectory_goal)
        except (ValidationError, ValueError) as e:
            self.get_logger().error(f'Goal rejected: {e}')
            self._trajectory_goal = None
            with self._executing_lock:
                self._is_executing = False
            return GoalResponse.REJECT

        self.get_logger().debug(
            f'Goal accepted: {len(self._trajectory_goal.paths)} path(s) — '
            f'ids: {[p.path_id for p in self._trajectory_goal.paths]}'
        )
        return GoalResponse.ACCEPT

    def _execute_callback(
        self, goal_handle: ServerGoalHandle
    ) -> ExecuteTrajectory.Result:
        with self._goal_handle_lock:
            self._goal_handle = goal_handle  # store for cancellation and feedback
        if self._trajectory_goal is None:
            self.get_logger().warning('Trajectory goal is not set, creating from request')
            goal = TrajectoryGoalDTO.from_ros_msg(goal_handle.request)
        else:
            goal = self._trajectory_goal  # use the already validated DTO from the goal callback
        groups = TrajectoryGrouper.group(goal.paths)
        self.get_logger().debug(
            f'Trajectory grouped into {len(groups)} execution group(s): '
            f'{[[p.path_id for p in g] for g in groups]}'
        )
        completed_ids: list[str] = []
        error_message: str = ''

        if (
            self._planner_service is None or
            self._execute_trajectory_client is None or
            self._execute_trajectory_client.wait_for_server(timeout_sec=5.0) is False
        ):
            error_message = 'planner service or move_group client not initialised'
            self.get_logger().error(error_message)
            return self._generate_failure_result(goal_handle, error_message, completed_ids)

        try:
            self.get_logger().debug("Starting planning and execution of trajectories")
            if self._planner_service.plan_all(groups):  # start planning in background
                for plan in self._planner_service.iterate_planned_trajectories():
                    if (
                        goal_handle.is_cancel_requested or
                        not plan.success
                    ):
                        if goal_handle.is_cancel_requested:
                            error_message = "Goal was cancelled"
                            self.get_logger().info(error_message)
                        else:
                            error_message = f'Look-ahead planning failed - {plan.error_message}'
                            self.get_logger().error(error_message)

                        return self._generate_failure_result(goal_handle, error_message, completed_ids)
                    
                    self.get_logger().debug("Sending feedback for executing trajectory paths")
                    fb = ExecuteTrajectory.Feedback()
                    fb.status = FeedbackStatusEnum.EXECUTING.value
                    fb.trajectory_path_ids = plan.path_ids
                    goal_handle.publish_feedback(fb)

                    for traj in plan.motion_plan.planned_trajectories:  # type: ignore[union-attr]
                        self.get_logger().debug(f'Sending trajectory for execution: {traj}')
                        exec_goal = MoveItExecuteTrajectory.Goal()
                        exec_goal.trajectory = traj
                        execution_result: MoveitExecuteTrajectoryResponse = self._call_with_cancellation(
                            goal_handle,
                            exec_goal
                        )
                        
                        self._logger.debug(f'Execution status: {execution_result.status}, result: {execution_result.result}')

                        if execution_result.status in [5, 6]:  # CANCELLED/ABORTED
                            raise ExecutionFailedError("trajectory execution was cancelled")
                        elif (
                            execution_result.status != 4 or  # SUCCEEDED
                            execution_result.result is None or
                            execution_result.result.error_code.val != MoveItErrorCodes.SUCCESS
                        ):
                            self._planner_service.cancel()  # ensure planning is stopped on exec failure
                            err_code = (
                                execution_result.result.error_code.val if execution_result.result else None
                            )
                            raise ExecutionFailedError(
                                f'failed to execute trajectory for paths {plan.path_ids}: '
                                f'error code {err_code}'
                            )
                    
                    fb2 = ExecuteTrajectory.Feedback()
                    fb2.status = FeedbackStatusEnum.COMPLETED.value
                    fb2.trajectory_path_ids = plan.path_ids
                    goal_handle.publish_feedback(fb2)

                    # h. Track completed IDs (D-02)
                    completed_ids.extend(plan.path_ids)

                self.get_logger().info("Finished executing trajectories")
                result = ExecuteTrajectory.Result()
                result.success = True
                result.error_message = ''
                result.trajectory_paths_completed = completed_ids
                goal_handle.succeed()
                with self._executing_lock:
                    self._is_executing = False
                with self._goal_handle_lock:
                    self._goal_handle = None  # reset goal handle after execution
                self.get_logger().info("All trajectories executed successfully, goal succeeded")
                return result
            # else case
            error_message = 'Planning failed or no paths to execute'
        except ExecutionFailedError as e:
            error_message = f'Execution failed - {e}'
        except Exception as e:
            error_message = f'Unknown exception raised - {e}'

        self.get_logger().error(error_message)
        return self._generate_failure_result(goal_handle, error_message, completed_ids)

    def _cancel_callback(self, cancel_request: CancelGoal.Request) -> CancelResponse:
        """Non-blocking cancel: signal planner to stop and return ACCEPT immediately (D-10).

        Does NOT join the planning thread. Does NOT call moveit.stop().
        The background thread terminates when it next checks cancel_event.
        The iterate_planned_trajectories() generator terminates via the StopIteration sentinel.
        """
        self.get_logger().info('Cancel request received')
        if self._planner_service is not None:
            self.get_logger().debug("Signalling planner service to cancel")
            self._planner_service.cancel()

        if self._cancellation_pub is not None:
            self.get_logger().debug("Cancelling moveit execution goal")
            # NOTE: there is a bug and cancellation via goal handle does not work, need to use 
            # this topic as per issue resolution/workaround explained here:
            # https://github.com/moveit/moveit2/issues/2808
            self._cancellation_pub.publish(String(data="stop"))
        
        return CancelResponse.ACCEPT

    # endregion: callbacks


def main(args=None) -> None:
    """Entry point for the ``movement_controller`` executable.

    Initialises rclpy, creates a :class:`MovementController` node, and spins
    it with a :class:`~rclpy.executors.MultiThreadedExecutor` (5 threads) to
    allow concurrent goal, feedback, and cancel callbacks.  Shuts down cleanly
    on exit or keyboard interrupt.

    :param args: Optional command-line arguments forwarded to :func:`rclpy.init`.
    :type args: list[str] | None
    """
    init(args=args)
    node = MovementController()
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        shutdown()
