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
"""PlanningCoordinator — routes trajectory groups to the PILZ or OMPL planner and chains results."""

from __future__ import annotations

from queue import Queue
from threading import Event
from collections.abc import Iterator

from rclpy.lifecycle.node import LifecycleNode
from rclpy.client import Client
from rclpy import Future as RosFuture

from moveit_msgs.msg import PlanningSceneComponents, RobotState
from moveit_msgs.srv import GetPlanningScene

from movement_controller.enums.motion_type_enum import MotionTypeEnum
from movement_controller.models import (
    ConstraintConfigDTO,
    PlanResultDTO,
    PlanningSessionDTO,
    TrajectoryPathDTO,
)
from movement_controller.services.base_planner_service import BasePlannerService
from movement_controller.services.ompl_planner_service import OmplPlannerService
from movement_controller.services.pilz_planner_service import PilzPlannerService


class PlanningCoordinator(BasePlannerService):
    """Routes trajectory groups to the correct planner and chains their end states.

    The coordinator owns the ``get_planning_scene`` client (to obtain the live
    robot state that seeds the first group) plus one :class:`PilzPlannerService`
    (LIN/CIRC) and one :class:`OmplPlannerService` (collision-aware PTP).

    Look-ahead is fully asynchronous: after the scene is fetched, each group is
    routed by its motion type — PTP groups (always isolated) go to OMPL,
    LIN/CIRC groups go to PILZ — via ``plan_group_async``.  Each result is pushed
    onto an internal queue; the successful group's end state is extracted and
    used as the start state for the next group, threading the sequence together
    across planners.
    """

    def __init__(self, node: LifecycleNode, moveit_group_name: str) -> None:
        """Initialise the coordinator and its two sub-planner services.

        The ``get_planning_scene`` client and the sub-services' clients are all
        created lazily during :meth:`on_activate` to respect the lifecycle
        pattern.

        :param node: Parent lifecycle node; used to create service clients and
            access the logger.
        :type node: LifecycleNode
        :param moveit_group_name: MoveIt 2 planning group name as defined in
            the SRDF (e.g. ``'ur_manipulator'``).
        :type moveit_group_name: str
        """
        super().__init__(node, moveit_group_name)
        self._pilz = PilzPlannerService(node, moveit_group_name)
        self._ompl = OmplPlannerService(node, moveit_group_name)
        self._scene_client: Client | None = None
        self._plan_queue: Queue[PlanResultDTO | StopIteration] | None = None
        self._cancel_event: Event | None = None
        self._session: PlanningSessionDTO | None = None

    # region: private helpers
    def _abort(self, err_msg: str) -> None:
        """Push a failure DTO and a StopIteration sentinel onto the plan queue."""
        if self._plan_queue is not None:
            self._plan_queue.put(PlanResultDTO(error_message=err_msg))
            self._plan_queue.put(StopIteration())

    def _select_service(self, group: list[TrajectoryPathDTO]) -> PilzPlannerService | OmplPlannerService:
        """Select the planner for a group based on its (leading) motion type.

        PTP paths are always isolated into single-item groups by the grouper,
        so the group's motion type is fully determined by ``group[0]``.

        :param group: The group about to be planned.
        :type group: list[TrajectoryPathDTO]
        :returns: The OMPL service for PTP groups, else the PILZ service.
        :rtype: PilzPlannerService | OmplPlannerService
        """
        if group[0].motion_type == MotionTypeEnum.PTP:
            self._logger.debug(f'Routing PTP path {group[0].path_id} to OMPL planner')
            return self._ompl
        self._logger.debug(
            f'Routing LIN/CIRC group {[p.path_id for p in group]} to PILZ planner'
        )
        return self._pilz

    def _schedule_next(self, start_state: RobotState) -> None:
        """Pop the next group and dispatch it to its planner, or finish the sequence.

        :param start_state: Robot start state for the next group (live scene
            state for the first group, or the previous group's end state).
        :type start_state: RobotState
        """
        if self._cancel_event is not None and self._cancel_event.is_set():
            self._logger.info('planning cancelled; not scheduling next group')
            return

        group = self._session.get_next_group() if self._session is not None else None
        if not group:
            self._logger.info('all groups planned successfully; finishing planning sequence')
            if self._plan_queue is not None:
                self._plan_queue.put(StopIteration())
            return

        self._logger.debug(
            f'Scheduling planning for group of {len(group)} path(s): '
            f'{[p.path_id for p in group]}'
        )
        service = self._select_service(group)
        service.plan_group_async(group, start_state, self._on_group_planned)

    # endregion: private helpers

    # region: callbacks
    def _on_scene(self, future: RosFuture) -> None:
        """Handle the ``get_planning_scene`` response and start the planning chain."""
        if self._cancel_event is not None and self._cancel_event.is_set():
            self._logger.info('planning cancelled; skipping scene processing')
            return

        self._logger.debug('Planning scene response received; initiating first planning call')
        try:
            response = future.result()
            if response is None or response.scene.robot_state is None:
                err_msg = 'failed to retrieve planning scene or robot state; aborting planning'
                self._logger.error(err_msg)
                self._abort(err_msg)
                return

            robot_state = response.scene.robot_state
            self._logger.debug(
                f'Planning scene retrieved; robot joint state names: '
                f'{list(robot_state.joint_state.name)}'
            )
            self._schedule_next(robot_state)
        except Exception as e:
            err_msg = f'exception while retrieving planning scene - {e}'
            self._logger.error(err_msg)
            self._abort(err_msg)

    def _on_group_planned(self, plan_dto: PlanResultDTO) -> None:
        """Handle one group's plan result: enqueue it, then chain or stop.

        Pushes the result onto the queue.  On failure, pushes a StopIteration
        sentinel to end the sequence.  On success, extracts the group's end
        state and schedules the next group from there.

        :param plan_dto: Result delivered by a sub-planner's ``plan_group_async``.
        :type plan_dto: PlanResultDTO
        """
        if self._cancel_event is not None and self._cancel_event.is_set():
            self._logger.info('planning cancelled; discarding group result and not chaining')
            return

        self._logger.debug(
            f'Group planned: success={plan_dto.success}, path_ids={plan_dto.path_ids}, '
            f'blended={plan_dto.blended}'
        )
        if self._plan_queue is not None:
            self._plan_queue.put(plan_dto)

        if not plan_dto.success:
            self._logger.error(
                f'group planning failed ({plan_dto.error_message}); stopping planning sequence'
            )
            if self._plan_queue is not None:
                self._plan_queue.put(StopIteration())
            return

        try:
            end_state = self._extract_end_state(plan_dto.motion_plan)  # type: ignore[arg-type]
        except Exception as e:
            err_msg = f'failed to extract end state from planned group - {e}'
            self._logger.error(err_msg)
            self._abort(err_msg)
            return

        self._schedule_next(end_state)

    # endregion: callbacks

    # region: lifecycle
    def on_activate(self) -> None:
        """Create the ``get_planning_scene`` client and activate both sub-services."""
        self._logger.debug('Creating get_planning_scene service client')
        self._scene_client = self._node.create_client(
            srv_type=GetPlanningScene,
            srv_name='get_planning_scene',
            callback_group=self._callback_group,
        )
        self._pilz.on_activate()
        self._ompl.on_activate()
        self._cancel_event = None
        self._plan_queue = None
        self._session = None
        self._logger.debug('PlanningCoordinator activated')

    def on_deactivate(self) -> None:
        """Destroy the ``get_planning_scene`` client and deactivate both sub-services."""
        if self._scene_client is not None:
            self._logger.debug('Destroying get_planning_scene service client')
            self._node.destroy_client(self._scene_client)
            self._scene_client = None
        self._pilz.on_deactivate()
        self._ompl.on_deactivate()
        self._session = None
        self._logger.debug('PlanningCoordinator deactivated')

    # endregion: lifecycle

    # region: public methods
    def set_constraints(self, dto: ConstraintConfigDTO) -> None:
        """Store the active constraint configuration on the coordinator and both sub-services.

        :param dto: Validated constraint configuration to apply.
        :type dto: ConstraintConfigDTO
        """
        super().set_constraints(dto)
        self._pilz.set_constraints(dto)
        self._ompl.set_constraints(dto)

    def set_ompl_tuning(self, planning_time: float, num_planning_attempts: int) -> None:
        """Forward OMPL tuning parameters to the OMPL sub-service.

        :param planning_time: OMPL ``allowed_planning_time`` in seconds.
        :type planning_time: float
        :param num_planning_attempts: OMPL ``num_planning_attempts``.
        :type num_planning_attempts: int
        """
        self._ompl.set_tuning(planning_time, num_planning_attempts)

    def wait_for_service(self, timeout_sec: float) -> bool:
        """Block until the scene service and both planner services are available.

        :param timeout_sec: Maximum number of seconds to wait for each service.
        :type timeout_sec: float
        :returns: ``True`` only if the ``get_planning_scene`` service and both
            sub-planner services became available within the timeout; ``False``
            otherwise (including before :meth:`on_activate` has been called).
        :rtype: bool
        """
        if self._scene_client is None:
            self._logger.error('wait_for_service called before activation')
            return False

        self._logger.debug(f'Waiting for get_planning_scene service (timeout={timeout_sec}s)')
        scene_available = self._scene_client.wait_for_service(timeout_sec=timeout_sec)
        if not scene_available:
            self._logger.debug(f'get_planning_scene service not available after {timeout_sec}s')
            return False

        pilz_available = self._pilz.wait_for_service(timeout_sec=timeout_sec)
        ompl_available = self._ompl.wait_for_service(timeout_sec=timeout_sec)
        return pilz_available and ompl_available

    def plan_all(self, groups: list[list[TrajectoryPathDTO]]) -> bool:
        """Start asynchronous look-ahead planning across all trajectory groups.

        Fetches the current robot state via ``get_planning_scene`` and then
        chains ``plan_group_async`` calls — one per group, routed to PILZ or
        OMPL — through ROS 2 future callbacks.  Results are pushed onto an
        internal queue as :class:`~movement_controller.models.PlanResultDTO`
        objects, terminated by a :class:`StopIteration` sentinel.  Consume them
        incrementally with :meth:`iterate_planned_trajectories`.

        :param groups: Ordered list of trajectory-path groups as produced by
            :class:`~movement_controller.utils.trajectory_grouper.TrajectoryGrouper`.
        :type groups: list[list[TrajectoryPathDTO]]
        :returns: ``True`` if planning was successfully started (scene service
            reachable); ``False`` if ``get_planning_scene`` was unavailable.
        :rtype: bool
        """
        total_paths = sum(len(g) for g in groups)
        self._logger.debug(
            f'plan_all called: {len(groups)} group(s), {total_paths} total path(s)'
        )
        self._cancel_event = Event()
        self._plan_queue = Queue()
        self._session = PlanningSessionDTO(groups=groups)

        self._logger.debug('Waiting for get_planning_scene service (timeout=5.0s)')
        if self._scene_client is not None and self._scene_client.wait_for_service(timeout_sec=5.0):
            self._logger.debug('get_planning_scene service available; requesting current robot state')
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.ROBOT_STATE
            future: RosFuture = self._scene_client.call_async(request)
            future.add_done_callback(self._on_scene)
            return True

        self._logger.error('planning scene service not available; cannot start planning')
        self._cancel_event = None
        self._plan_queue = None
        self._session = None
        return False

    def iterate_planned_trajectories(self) -> Iterator[PlanResultDTO]:
        """Yield one :class:`~movement_controller.models.PlanResultDTO` per group as planning completes.

        Blocks on the internal queue until each result is ready.  Terminates
        when the planning callback chain pushes the :class:`StopIteration`
        sentinel.  Yields a single failure DTO immediately if :meth:`plan_all`
        was not called first.

        :returns: Generator of :class:`~movement_controller.models.PlanResultDTO`
            objects in the order groups were submitted to :meth:`plan_all`.
        :rtype: Iterator[PlanResultDTO]
        """
        self._logger.debug('Starting iteration over planned trajectories')
        while True:
            if self._plan_queue:
                self._logger.debug('Waiting for next plan result from planning queue')
                item = self._plan_queue.get()
                if isinstance(item, StopIteration):
                    self._logger.debug('Received StopIteration sentinel; all trajectories planned')
                    self._plan_queue = None
                    self._cancel_event = None
                    self._session = None
                    return

                self._logger.debug(
                    f'Dequeued plan result: success={item.success}, '
                    f'path_ids={item.path_ids}, blended={item.blended}'
                )
                yield item
            else:
                self._logger.warning('iterate_planned_trajectories called but plan queue is not initialized')
                yield PlanResultDTO(error_message='plan queue not initialized; did you call plan_all()?')
                return

    def cancel(self) -> None:
        """Non-blocking cancellation of the active planning sequence.

        Sets the cancel event flag so pending future callbacks are skipped,
        drains the result queue, and pushes a failure
        :class:`~movement_controller.models.PlanResultDTO` so any blocked
        :meth:`iterate_planned_trajectories` call unblocks immediately.

        Idempotent — safe to call before :meth:`plan_all` or multiple times.
        """
        self._logger.debug('cancel() called on PlanningCoordinator')
        if self._cancel_event is not None and not self._cancel_event.is_set():
            self._logger.debug('Setting cancel event flag')
            self._cancel_event.set()
        if self._plan_queue is not None:
            self._logger.debug('Draining plan queue and pushing cancellation sentinel')
            with self._plan_queue.mutex:
                self._plan_queue.queue.clear()
            self._plan_queue.put(PlanResultDTO(error_message='planning cancelled by requester'))
            self._plan_queue.put(StopIteration())

    # endregion: public methods
