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
"""OmplPlannerService — plans a single PTP path via OMPL using the GetMotionPlan service."""

from __future__ import annotations

from collections.abc import Callable

from rclpy.lifecycle.node import LifecycleNode
from rclpy.client import Client
from rclpy import Future as RosFuture

from moveit_msgs.msg import (
    MotionPlanRequest,
    MotionSequenceResponse,
    MoveItErrorCodes,
    RobotState,
)
from moveit_msgs.srv import GetMotionPlan

from movement_controller.models import (
    PlanResultDTO,
    TrajectoryPathDTO,
)
from movement_controller.services.base_planner_service import BasePlannerService


class OmplPlannerService(BasePlannerService):
    """Plans a single collision-aware PTP path via OMPL (RRTConnect) + TOTG.

    Unlike :class:`PilzPlannerService`, which plans whole blended groups through
    the PILZ ``MotionSequence`` service, this service plans exactly one PTP path
    per call through the standard MoveIt ``GetMotionPlan`` service
    (``plan_kinematic_path``).  OMPL reroutes around collision objects, and the
    ``ompl`` pipeline's ``AddTimeOptimalParameterization`` response adapter
    re-times the resulting path (TOTG) without changing its geometry.

    Like :class:`PilzPlannerService`, planning is fully asynchronous: a
    ``call_async`` future delivers its result to a caller-supplied callback via
    :meth:`plan_group_async`.  Cross-group chaining and cancellation are owned by
    :class:`~movement_controller.services.planning_coordinator.PlanningCoordinator`.
    """

    # MoveIt error codes that most likely indicate the straight/sampled path could
    # not be routed around obstacles within the planning budget.
    _NO_PATH_ERROR_CODES = frozenset(
        {
            MoveItErrorCodes.PLANNING_FAILED,
            MoveItErrorCodes.INVALID_MOTION_PLAN,
            MoveItErrorCodes.TIMED_OUT,
        }
    )

    def __init__(self, node: LifecycleNode, moveit_group_name: str) -> None:
        """Initialise the OmplPlannerService.

        The ``GetMotionPlan`` service client is created lazily during
        :meth:`on_activate` to respect the lifecycle pattern.

        :param node: Parent lifecycle node; used to create the service client
            and access the logger.
        :type node: LifecycleNode
        :param moveit_group_name: MoveIt 2 planning group name as defined in
            the SRDF (e.g. ``'ur_manipulator'``).
        :type moveit_group_name: str
        """
        super().__init__(node, moveit_group_name)
        self._plan_client: Client | None = None
        self._planning_time: float = 10.0
        self._num_planning_attempts: int = 5

    # region: private methods
    def _build_motion_plan_request(
        self, path_dto: TrajectoryPathDTO, start_state: RobotState
    ) -> MotionPlanRequest:
        """Build an OMPL MotionPlanRequest for a single PTP path.

        :param path_dto: The PTP path to plan.
        :type path_dto: TrajectoryPathDTO
        :param start_state: Robot start state (from the planning scene for the
            first group, or the previous group's end state for chaining).
        :type start_state: RobotState
        :returns: A fully populated OMPL motion plan request.
        :rtype: MotionPlanRequest
        """
        req = MotionPlanRequest()
        req.group_name = self._group_name
        req.pipeline_id = 'ompl'
        req.planner_id = 'RRTConnect'
        req.num_planning_attempts = self._num_planning_attempts
        req.allowed_planning_time = self._planning_time
        req.start_state = start_state
        req.goal_constraints = [
            self._build_pose_goal_constraints(path_dto.tool_frame, path_dto.target_pose)
        ]
        req.path_constraints = self._build_path_constraints(path_dto.tool_frame)
        (
            req.max_velocity_scaling_factor,
            req.max_acceleration_scaling_factor,
        ) = self._calculate_scaling_factors(path_dto)
        self._logger.debug(
            f'Built OMPL MotionPlanRequest for path {path_dto.path_id}: '
            f'planner_id={req.planner_id}, allowed_planning_time={req.allowed_planning_time}, '
            f'num_planning_attempts={req.num_planning_attempts}, '
            f'vel_scale={req.max_velocity_scaling_factor}, acc_scale={req.max_acceleration_scaling_factor}'
        )
        return req

    def _describe_error(self, error_val: int) -> str:
        """Map a MoveIt error code to a human-readable OMPL planning failure message."""
        if error_val in self._NO_PATH_ERROR_CODES:
            return (
                'OMPL found no collision-free path within the planning budget '
                f'(a collision object may block all paths); error code {error_val}'
            )
        if error_val == MoveItErrorCodes.START_STATE_IN_COLLISION:
            return f'OMPL planning failed: start state is in collision; error code {error_val}'
        if error_val == MoveItErrorCodes.GOAL_IN_COLLISION:
            return f'OMPL planning failed: goal state is in collision; error code {error_val}'
        if error_val == MoveItErrorCodes.NO_IK_SOLUTION:
            return f'OMPL planning failed: no IK solution for the goal pose; error code {error_val}'
        return f'OMPL planning failed with error code {error_val}'

    def _wrap_trajectory(self, response: GetMotionPlan.Response) -> MotionSequenceResponse:
        """Wrap the single OMPL trajectory into a MotionSequenceResponse.

        This keeps the :class:`PlanResultDTO` and the node's execution loop
        uniform across the PILZ (sequence) and OMPL (single-plan) planners.
        """
        seq = MotionSequenceResponse()
        seq.error_code.val = MoveItErrorCodes.SUCCESS
        seq.planned_trajectories = [response.motion_plan_response.trajectory]
        return seq

    # endregion: private methods

    # region: callbacks
    def _on_plan_response(
        self,
        future: RosFuture,
        path_id: str,
        on_result: Callable[[PlanResultDTO], None],
    ) -> None:
        """Convert a ``GetMotionPlan`` future into a :class:`PlanResultDTO`.

        Called when the ``plan_kinematic_path`` service future completes.  The
        single OMPL trajectory is wrapped in a
        :class:`~moveit_msgs.msg.MotionSequenceResponse` so downstream consumers
        treat it identically to a PILZ result.  The resulting DTO — success or
        failure — is delivered to the coordinator via ``on_result``.

        :param future: Completed ``GetMotionPlan`` future.
        :type future: rclpy.Future
        :param path_id: Path ID of the PTP path that was planned.
        :type path_id: str
        :param on_result: Callback invoked exactly once with the resulting DTO.
        :type on_result: Callable[[PlanResultDTO], None]
        """
        self._logger.debug(f'Received OMPL GetMotionPlan response for path {path_id}')
        try:
            response = future.result()
            if response is None:
                err_msg = 'ompl GetMotionPlan returned no response'
                self._logger.error(err_msg)
                on_result(PlanResultDTO(error_message=err_msg))
                return

            error_val = response.motion_plan_response.error_code.val
            if error_val != MoveItErrorCodes.SUCCESS:
                err_msg = self._describe_error(error_val)
                self._logger.error(err_msg)
                on_result(PlanResultDTO(error_message=err_msg))
                return

            self._logger.debug(
                f'OMPL planning succeeded for path {path_id}; wrapping trajectory'
            )
            on_result(
                PlanResultDTO(
                    success=True,
                    motion_plan=self._wrap_trajectory(response),
                    path_ids=[path_id],
                    blended=False,
                )
            )
        except Exception as e:
            err_msg = f'exception while processing ompl planning response for path {path_id} - {e}'
            self._logger.error(err_msg)
            on_result(PlanResultDTO(error_message=err_msg))

    # endregion: callbacks

    # region: lifecycle
    def on_activate(self) -> None:
        """Create the ``plan_kinematic_path`` (GetMotionPlan) service client."""
        self._logger.debug('Creating plan_kinematic_path (GetMotionPlan) service client')
        self._plan_client = self._node.create_client(
            srv_type=GetMotionPlan,
            srv_name='plan_kinematic_path',
            callback_group=self._callback_group,
        )
        self._logger.debug('OmplPlannerService service client created')

    def on_deactivate(self) -> None:
        """Destroy the GetMotionPlan service client."""
        if self._plan_client is not None:
            self._logger.debug('Destroying plan_kinematic_path service client')
            self._node.destroy_client(self._plan_client)
            self._plan_client = None
        self._logger.debug('OmplPlannerService deactivated and client destroyed')

    # endregion: lifecycle

    # region: public methods
    def set_tuning(self, planning_time: float, num_planning_attempts: int) -> None:
        """Configure OMPL tuning parameters (typically from node parameters).

        :param planning_time: OMPL ``allowed_planning_time`` in seconds.
        :type planning_time: float
        :param num_planning_attempts: OMPL ``num_planning_attempts``.
        :type num_planning_attempts: int
        """
        self._planning_time = planning_time
        self._num_planning_attempts = num_planning_attempts
        self._logger.debug(
            f'OMPL tuning set: planning_time={planning_time}s, '
            f'num_planning_attempts={num_planning_attempts}'
        )

    def wait_for_service(self, timeout_sec: float) -> bool:
        """Block until the ``plan_kinematic_path`` service is available.

        :param timeout_sec: Maximum number of seconds to wait.
        :type timeout_sec: float
        :returns: ``True`` if the service became available within the timeout;
            ``False`` if it timed out or the client has not been initialised yet
            (i.e. :meth:`on_activate` has not been called).
        :rtype: bool
        """
        if self._plan_client is None:
            self._logger.error('wait_for_service called before activation')
            return False

        self._logger.debug(f'Waiting for plan_kinematic_path service (timeout={timeout_sec}s)')
        available = self._plan_client.wait_for_service(timeout_sec=timeout_sec)
        if available:
            self._logger.debug('plan_kinematic_path service is available')
        else:
            self._logger.debug(f'plan_kinematic_path service not available after {timeout_sec}s')
        return available

    def plan_group_async(
        self,
        group: list[TrajectoryPathDTO],
        start_state: RobotState,
        on_result: Callable[[PlanResultDTO], None],
    ) -> None:
        """Plan a single PTP path asynchronously via OMPL (``plan_kinematic_path``).

        The grouper guarantees every PTP path is isolated into its own
        single-item group, so only ``group[0]`` is planned; any extra
        items would indicate a grouping bug and are ignored with a warning.
        Issues a non-blocking ``call_async`` and arranges for the result to be
        delivered to ``on_result`` exactly once (via :meth:`_on_plan_response`).

        :param group: Single-item group holding the PTP path to plan.
        :type group: list[TrajectoryPathDTO]
        :param start_state: Robot start state for this path (previous group's
            end state, or the live scene state for the first group).
        :type start_state: RobotState
        :param on_result: Callback invoked exactly once with the resulting
            :class:`~movement_controller.models.PlanResultDTO` (success or failure).
        :type on_result: Callable[[PlanResultDTO], None]
        """
        if not group:
            err_msg = 'ompl plan_group_async called with an empty group'
            self._logger.error(err_msg)
            on_result(PlanResultDTO(error_message=err_msg))
            return
        if len(group) > 1:
            self._logger.warning(
                f'ompl plan_group_async received {len(group)} paths; PTP paths must be '
                f'isolated, planning only the first path {group[0].path_id}'
            )

        path_dto = group[0]
        self._logger.debug(f'plan_group_async called for PTP path {path_dto.path_id}')

        if self._plan_client is None:
            err_msg = 'ompl planning client not initialised; did you call on_activate()?'
            self._logger.error(err_msg)
            on_result(PlanResultDTO(error_message=err_msg))
            return

        try:
            if self._start_state_invalid(start_state):
                err_msg = 'invalid start state - violation of constraints'
                self._logger.error(err_msg)
                on_result(PlanResultDTO(error_message=err_msg))
                return

            request = GetMotionPlan.Request()
            request.motion_plan_request = self._build_motion_plan_request(path_dto, start_state)
        except Exception as e:
            err_msg = f'exception while building ompl motion plan request for path {path_dto.path_id} - {e}'
            self._logger.error(err_msg)
            on_result(PlanResultDTO(error_message=err_msg))
            return

        self._logger.debug(f'Sending OMPL GetMotionPlan request for path {path_dto.path_id}')
        future: RosFuture = self._plan_client.call_async(request)
        future.add_done_callback(
            lambda f: self._on_plan_response(f, path_dto.path_id, on_result)
        )

    # endregion: public methods
