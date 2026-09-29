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
"""PilzPlannerService — plans a single blended LIN/CIRC group via the PILZ MotionSequence service."""

from __future__ import annotations

from collections.abc import Callable

from rclpy.lifecycle.node import LifecycleNode
from rclpy.client import Client
from rclpy import Future as RosFuture

from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MoveItErrorCodes,
    MotionSequenceItem,
    MotionSequenceRequest,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetMotionSequence
from geometry_msgs.msg import Pose

from movement_controller.enums.motion_type_enum import MotionTypeEnum
from movement_controller.exceptions import (
    InvalidStartStateError,
)
from movement_controller.models import (
    PlanResultDTO,
    TrajectoryPathDTO
)
from movement_controller.services.base_planner_service import BasePlannerService


class PilzPlannerService(BasePlannerService):
    """Plans a single blended group of LIN/CIRC paths via the PILZ ``MotionSequence`` service.

    This service owns only the ``plan_sequence_path`` client and plans exactly
    one group per :meth:`plan_group_async` call, delivering the result through a
    caller-supplied callback.  Cross-group look-ahead chaining, planning-scene
    retrieval, queueing, and cancellation are owned by
    :class:`~movement_controller.services.planning_coordinator.PlanningCoordinator`.
    """

    def __init__(self, node: LifecycleNode, moveit_group_name: str) -> None:
        """Initialise the PilzPlannerService.

        Stores the parent node reference and planning group name.  The
        ``plan_sequence_path`` service client is created lazily during
        :meth:`on_activate` to respect the lifecycle pattern.

        :param node: Parent lifecycle node; used to create the service client
            and access the logger.
        :type node: LifecycleNode
        :param moveit_group_name: MoveIt 2 planning group name as defined in
            the SRDF (e.g. ``'ur_manipulator'``).
        :type moveit_group_name: str
        """
        super().__init__(node, moveit_group_name)
        self._plan_seq_client: Client | None = None

    # region: private methods
    def _merge_circ_and_path_constraints(self, circ: Constraints, path: Constraints) -> Constraints:
        """Merge CIRC arc central point path constraint with general path constraints

        Preserves the CIRC arc point at position_constraints[0] and appends workspace BOX
        (if any) from path constraints at [1:]. This avoids overwriting the arc definition
        that PILZ reads for CIRC planning.
        """
        merged = Constraints()
        merged.name = circ.name
        merged.position_constraints = [circ.position_constraints[0]] + path.position_constraints  # type: ignore
        merged.joint_constraints = path.joint_constraints
        merged.orientation_constraints = path.orientation_constraints
        self._logger.debug(f'Merged CIRC and path constraints:\n{merged}')
        return merged

    def _build_circ_constraints(self, path_dto: TrajectoryPathDTO) -> Constraints:
        """Build path constraints for PILZ CIRC planner."""
        constraints = Constraints()
        constraints.name = path_dto.circ_type.value  # 'interim' or 'center'

        pos_constraint = PositionConstraint()
        pos_constraint.header = path_dto.target_pose.header
        pos_constraint.link_name = path_dto.tool_frame
        pos_constraint.weight = 0.1

        point_pose = Pose()
        point_pose.position.x = path_dto.circ_point.x
        point_pose.position.y = path_dto.circ_point.y
        point_pose.position.z = path_dto.circ_point.z
        point_pose.orientation.w = 1.0

        bv = BoundingVolume()
        bv.primitives = []
        bv.primitive_poses = [point_pose]
        pos_constraint.constraint_region = bv

        constraints.position_constraints = [pos_constraint]
        self._logger.debug(
            f'Constructed CIRC constraints for path {path_dto.path_id}:\n'
            f'{constraints.position_constraints}'
        )
        return constraints

    def _generate_motion_sequence_request(self, group: list[TrajectoryPathDTO], start_state_msg: RobotState) -> MotionSequenceRequest:
        """Helper method to generate a MotionSequenceRequest from a group of TrajectoryPathDTOs and a start state."""
        self._logger.debug(f'Generating MotionSequenceRequest for group of {len(group)} paths with start state:\n{start_state_msg}')
        
        if self._start_state_invalid(start_state_msg):
            err_msg = 'invalid start state - violation of constraints'
            self._logger.error(err_msg)
            raise InvalidStartStateError(err_msg)

        items: list[MotionSequenceItem] = []
        for i, path_dto in enumerate(group):
            self._logger.debug(f'Generating motion sequence item for path {path_dto.path_id}')
            item = MotionSequenceItem()
            # PILZ constraint: last item in group MUST have blend_radius=0.0
            item.blend_radius = path_dto.blend_radius if i < len(group) - 1 else 0.0
            item.req.group_name = self._group_name
            item.req.pipeline_id = 'pilz_industrial_motion_planner'
            item.req.planner_id = path_dto.motion_type.value  # 'LIN', 'PTP', or 'CIRC'
            item.req.allowed_planning_time = 5.0
            item.req.num_planning_attempts = 1
            (
                item.req.max_velocity_scaling_factor,
                item.req.max_acceleration_scaling_factor
            ) = self._calculate_scaling_factors(path_dto)
            item.req.goal_constraints = [
                self._build_pose_goal_constraints(
                    path_dto.tool_frame, path_dto.target_pose
                )
            ]
            
            if i == 0:
                self._logger.debug(
                    f'Setting start state for first path in group:\n{start_state_msg}'
                )
                item.req.start_state = start_state_msg

            path_constraints = self._build_path_constraints(path_dto.tool_frame)
            if path_dto.motion_type == MotionTypeEnum.CIRC:
                self._logger.debug(
                    f'Building CIRC-specific constraints for path {path_dto.path_id} with CIRC '
                    f'type {path_dto.circ_type}'
                )
                circ_constraints = self._build_circ_constraints(path_dto)
                item.req.path_constraints = self._merge_circ_and_path_constraints(
                    circ_constraints, path_constraints
                )
            else:
                item.req.path_constraints = path_constraints
            
            self._logger.debug(f'Final path constraint:\n{item.req.path_constraints}')

            items.append(item)
        
        self._logger.debug(f'Final list of items into MotionSequenceRequest:\n{items}')

        seq_req = MotionSequenceRequest()
        seq_req.items = items
        return seq_req
    
    # endregion: private methods

    # region: callbacks
    def _on_plan_response(
        self,
        future: RosFuture,
        path_ids: list[str],
        on_result: Callable[[PlanResultDTO], None],
    ) -> None:
        """Convert a ``GetMotionSequence`` future into a :class:`PlanResultDTO`.

        Called when the ``plan_sequence_path`` service future completes.  The
        resulting DTO — success or failure — is delivered to the coordinator via
        ``on_result``; this service performs no queueing or chaining itself.

        :param future: Completed ``GetMotionSequence`` future.
        :type future: rclpy.Future
        :param path_ids: Path IDs of the group that was planned, used to
            populate :attr:`PlanResultDTO.path_ids`.
        :type path_ids: list[str]
        :param on_result: Callback invoked exactly once with the resulting DTO.
        :type on_result: Callable[[PlanResultDTO], None]
        """
        self._logger.debug(f'Received motion sequence response for path IDs {path_ids}')
        try:
            response = future.result()
            if response is None or response.response.error_code.val != MoveItErrorCodes.SUCCESS:
                code = response.response.error_code.val if response is not None else 'None'
                err_msg = f'planning sequence service call failed with error code {code}'
                self._logger.error(err_msg)
                on_result(PlanResultDTO(error_message=err_msg))
                return

            self._logger.debug(
                f'Planned motion sequence for path IDs {path_ids} with '
                f'{len(response.response.planned_trajectories)} trajectory/trajectories'
            )
            on_result(
                PlanResultDTO(
                    success=True,
                    motion_plan=response.response,
                    path_ids=path_ids,
                    blended=len(path_ids) > 1,
                )
            )
        except Exception as e:
            err_msg = f'exception while processing planning sequence response - {e}'
            self._logger.error(err_msg)
            on_result(PlanResultDTO(error_message=err_msg))

    # endregion: callbacks

    # region: public methods
    def on_activate(self) -> None:
        """Create the ``plan_sequence_path`` (GetMotionSequence) service client.

        Called from the parent node's / coordinator's :meth:`on_activate`
        lifecycle transition.
        """
        self._logger.debug('Creating plan_sequence_path service client')
        self._plan_seq_client = self._node.create_client(
            srv_type=GetMotionSequence,
            srv_name='plan_sequence_path',
            callback_group=self._callback_group
        )
        self._logger.debug('PilzPlannerService service client created')

    def on_deactivate(self) -> None:
        """Destroy the ``plan_sequence_path`` service client.

        Called from the parent node's / coordinator's :meth:`on_deactivate`
        lifecycle transition.
        """
        if self._plan_seq_client is not None:
            self._logger.debug('Destroying plan_sequence_path service client')
            self._node.destroy_client(self._plan_seq_client)
            self._plan_seq_client = None
        self._logger.debug('PilzPlannerService deactivated and client destroyed')

    def wait_for_service(self, timeout_sec: float) -> bool:
        """Block until the ``plan_sequence_path`` service is available.

        :param timeout_sec: Maximum number of seconds to wait.
        :type timeout_sec: float
        :returns: ``True`` if the service became available within the timeout;
            ``False`` if it timed out or the client has not been initialised
            yet (i.e. :meth:`on_activate` has not been called).
        :rtype: bool
        """
        if self._plan_seq_client is None:
            self._logger.error('wait_for_service called before activation')
            return False
        
        self._logger.debug(f'Waiting for plan_sequence_path service (timeout={timeout_sec}s)')
        available = self._plan_seq_client.wait_for_service(timeout_sec=timeout_sec)
        if available:
            self._logger.debug('plan_sequence_path service is available')
        else:
            self._logger.debug(f'plan_sequence_path service not available after {timeout_sec}s')
        return available

    def plan_group_async(
        self,
        group: list[TrajectoryPathDTO],
        start_state: RobotState,
        on_result: Callable[[PlanResultDTO], None],
    ) -> None:
        """Plan one blended LIN/CIRC group asynchronously via ``plan_sequence_path``.

        Builds a single :class:`~moveit_msgs.msg.MotionSequenceRequest` for the
        group, issues a non-blocking ``call_async`` on the ``plan_sequence_path``
        service, and arranges for the result to be delivered to ``on_result``
        exactly once (via :meth:`_on_plan_response`).  Look-ahead chaining across
        groups is the coordinator's responsibility — this method plans only the
        group it is given, starting from ``start_state``.

        :param group: Ordered LIN/CIRC paths that make up one blended sequence.
        :type group: list[TrajectoryPathDTO]
        :param start_state: Robot start state for the first path in the group.
        :type start_state: RobotState
        :param on_result: Callback invoked exactly once with the resulting
            :class:`~movement_controller.models.PlanResultDTO` (success or failure).
        :type on_result: Callable[[PlanResultDTO], None]
        """
        path_ids = [p.path_id for p in group]
        self._logger.debug(f'plan_group_async called for LIN/CIRC group with path IDs {path_ids}')

        if self._plan_seq_client is None:
            err_msg = 'plan_sequence_path client not initialised; did you call on_activate()?'
            self._logger.error(err_msg)
            on_result(PlanResultDTO(error_message=err_msg))
            return

        try:
            request = GetMotionSequence.Request()
            request.request = self._generate_motion_sequence_request(group, start_state)
        except InvalidStartStateError:
            err_msg = 'robot start state violates one of the constraints; aborting group planning'
            self._logger.error(err_msg)
            on_result(PlanResultDTO(error_message=err_msg))
            return
        except Exception as e:
            err_msg = f'exception while building motion sequence request - {e}'
            self._logger.error(err_msg)
            on_result(PlanResultDTO(error_message=err_msg))
            return

        self._logger.debug(f'Sending motion sequence request for path IDs {path_ids}')
        future: RosFuture = self._plan_seq_client.call_async(request)
        future.add_done_callback(
            lambda f: self._on_plan_response(f, path_ids, on_result)
        )

    # endregion: public methods
