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
"""Unit tests for PlanningCoordinator — sub-planners and ROS 2 deps mocked."""

from typing import Tuple
from unittest.mock import MagicMock

from rclpy.lifecycle import LifecycleNode
from rclpy.impl.rcutils_logger import RcutilsLogger
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import (
    MoveItErrorCodes,
    MotionSequenceResponse,
    RobotState,
    RobotTrajectory,
)
from trajectory_msgs.msg import JointTrajectoryPoint

from movement_controller.enums.motion_type_enum import MotionTypeEnum
from movement_controller.models.constraint_config_dto import ConstraintConfigDTO
from movement_controller.models.plan_result_dto import PlanResultDTO
from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO
from movement_controller.services.ompl_planner_service import OmplPlannerService
from movement_controller.services.pilz_planner_service import PilzPlannerService
from movement_controller.services.planning_coordinator import PlanningCoordinator


# Resolve TYPE_CHECKING-only forward references so Pydantic can instantiate PlanResultDTO in tests.
PlanResultDTO.model_rebuild(_types_namespace={'MotionSequenceResponse': object})

_UUID_A = 'aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa'
_UUID_B = 'bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb'
_SCENE_SRV = 'get_planning_scene'
_UNSET = object()  # sentinel so an explicit scene_robot_state=None can be tested


def _make_path_dto(motion_type: MotionTypeEnum, path_id: str = _UUID_A) -> TrajectoryPathDTO:
    """Build a minimal valid TrajectoryPathDTO of the given motion type."""
    return TrajectoryPathDTO(
        path_id=path_id,
        motion_type=motion_type,
        target_pose=PoseStamped(),
        tool_frame='tool0',
    )


def _make_sync_future(response):
    """Return a mock ROS future that invokes add_done_callback synchronously."""
    future = MagicMock()
    future.result.return_value = response

    def add_done_callback(cb):
        cb(future)

    future.add_done_callback = add_done_callback
    return future


def _success_dto(path_ids: list[str], positions: list[float]) -> PlanResultDTO:
    """Build a success PlanResultDTO whose motion_plan carries a single trajectory."""
    seq = MotionSequenceResponse()
    seq.error_code.val = MoveItErrorCodes.SUCCESS
    traj = RobotTrajectory()
    traj.joint_trajectory.joint_names = ['joint_1']
    point = JointTrajectoryPoint()
    point.positions = positions
    traj.joint_trajectory.points = [point]
    seq.planned_trajectories = [traj]
    return PlanResultDTO(
        success=True,
        motion_plan=seq,
        path_ids=path_ids,
        blended=len(path_ids) > 1,
    )


def _sync_planner(dtos: list[PlanResultDTO], calls: list) -> MagicMock:
    """Build a mock sub-planner whose plan_group_async synchronously delivers preset DTOs.

    Each call records ``(group, start_state)`` into ``calls`` and delivers the next
    DTO from ``dtos`` to the ``on_result`` callback.
    """
    planner = MagicMock()
    dto_iter = iter(dtos)

    def plan_group_async(group, start_state, on_result):
        calls.append((group, start_state))
        on_result(next(dto_iter))

    planner.plan_group_async.side_effect = plan_group_async
    return planner


def _build_coordinator(
    pilz_dtos: list[PlanResultDTO] | None = None,
    ompl_dtos: list[PlanResultDTO] | None = None,
    scene_available: bool = True,
    scene_robot_state=_UNSET,
) -> Tuple[PlanningCoordinator, list, list]:
    """Build a coordinator with mocked sub-planners and a synchronous scene client."""
    scene_state = RobotState() if scene_robot_state is _UNSET else scene_robot_state
    scene_resp = MagicMock()
    scene_resp.scene.robot_state = scene_state
    scene_client = MagicMock()
    scene_client.wait_for_service.return_value = scene_available
    scene_client.call_async.side_effect = lambda req: _make_sync_future(scene_resp)

    mock_node = MagicMock(spec=LifecycleNode)
    mock_node.get_logger.return_value = MagicMock(spec=RcutilsLogger)
    mock_node.create_client.return_value = scene_client

    coord = PlanningCoordinator(mock_node, 'ur_manipulator')

    pilz_calls: list = []
    ompl_calls: list = []
    coord._pilz = _sync_planner(pilz_dtos or [], pilz_calls)
    coord._ompl = _sync_planner(ompl_dtos or [], ompl_calls)
    coord._scene_client = scene_client
    return coord, pilz_calls, ompl_calls


# region: plan_all / routing tests
def test_plan_all_returns_false_when_scene_unavailable():
    """plan_all() returns False and does not start planning if scene service is unavailable."""
    coord, _, _ = _build_coordinator(scene_available=False)
    result = coord.plan_all([[_make_path_dto(MotionTypeEnum.PTP)]])
    assert result is False
    assert coord._plan_queue is None


def test_plan_all_single_ptp_routes_to_ompl():
    """A single PTP group is routed to the OMPL sub-planner (D-2)."""
    coord, pilz_calls, ompl_calls = _build_coordinator(
        ompl_dtos=[_success_dto([_UUID_A], [0.1])]
    )
    result = coord.plan_all([[_make_path_dto(MotionTypeEnum.PTP)]])
    assert result is True

    results = list(coord.iterate_planned_trajectories())
    assert len(results) == 1
    assert results[0].success is True
    assert len(ompl_calls) == 1
    assert len(pilz_calls) == 0


def test_plan_all_single_lin_routes_to_pilz():
    """A single LIN group is routed to the PILZ sub-planner."""
    coord, pilz_calls, ompl_calls = _build_coordinator(
        pilz_dtos=[_success_dto([_UUID_A], [0.1])]
    )
    coord.plan_all([[_make_path_dto(MotionTypeEnum.LIN)]])

    results = list(coord.iterate_planned_trajectories())
    assert len(results) == 1
    assert results[0].success is True
    assert len(pilz_calls) == 1
    assert len(ompl_calls) == 0


def test_chaining_threads_end_state_across_planners():
    """The first group's end state seeds the next group's start state, across planners."""
    scene_state = RobotState()
    coord, pilz_calls, ompl_calls = _build_coordinator(
        pilz_dtos=[_success_dto([_UUID_A], [0.5])],
        ompl_dtos=[_success_dto([_UUID_B], [0.9])],
        scene_robot_state=scene_state,
    )
    groups = [
        [_make_path_dto(MotionTypeEnum.LIN, _UUID_A)],
        [_make_path_dto(MotionTypeEnum.PTP, _UUID_B)],
    ]
    coord.plan_all(groups)
    results = list(coord.iterate_planned_trajectories())

    assert len(results) == 2
    assert all(r.success for r in results)
    # First (PILZ) group starts from the live scene state.
    assert pilz_calls[0][1] is scene_state
    # Second (OMPL) group starts from the PILZ group's extracted end state.
    ompl_start_state = ompl_calls[0][1]
    assert list(ompl_start_state.joint_state.position) == [0.5]


def test_failure_stops_chain():
    """A failed group ends the sequence; subsequent groups are not planned."""
    coord, pilz_calls, ompl_calls = _build_coordinator(
        pilz_dtos=[PlanResultDTO(error_message='pilz failed')],
    )
    groups = [
        [_make_path_dto(MotionTypeEnum.LIN, _UUID_A)],
        [_make_path_dto(MotionTypeEnum.PTP, _UUID_B)],
    ]
    coord.plan_all(groups)
    results = list(coord.iterate_planned_trajectories())

    assert len(results) == 1
    assert results[0].success is False
    assert len(pilz_calls) == 1
    assert len(ompl_calls) == 0


def test_scene_robot_state_none_yields_failure():
    """A scene response with no robot_state yields a failure DTO and stops."""
    coord, pilz_calls, _ = _build_coordinator(scene_robot_state=None)
    coord.plan_all([[_make_path_dto(MotionTypeEnum.LIN)]])
    results = list(coord.iterate_planned_trajectories())

    assert len(results) == 1
    assert results[0].success is False
    assert len(pilz_calls) == 0

# endregion: plan_all / routing tests


# region: iterate / cancel tests
def test_iterate_yields_error_before_plan_all():
    """iterate_planned_trajectories() yields a single failure DTO when called before plan_all()."""
    coord, _, _ = _build_coordinator()
    results = list(coord.iterate_planned_trajectories())
    assert len(results) == 1
    assert results[0].success is False
    assert 'plan queue not initialized' in results[0].error_message


def test_cancel_unblocks_iterator():
    """cancel() after a planner that never responds pushes a failure and terminates cleanly."""
    coord, _, _ = _build_coordinator()
    # Sub-planner that never invokes on_result → queue stays empty after plan_all().
    coord._pilz = MagicMock()
    coord._pilz.plan_group_async.side_effect = lambda group, start_state, on_result: None

    coord.plan_all([[_make_path_dto(MotionTypeEnum.LIN)]])
    coord.cancel()

    results = list(coord.iterate_planned_trajectories())
    assert len(results) == 1
    assert results[0].success is False
    assert 'cancelled' in results[0].error_message.lower()

# endregion: iterate / cancel tests


# region: delegation / lifecycle tests
def test_set_constraints_delegates_to_both_subservices():
    """set_constraints() stores the dto and forwards it to both sub-planners."""
    coord, _, _ = _build_coordinator()
    dto = ConstraintConfigDTO()
    coord.set_constraints(dto)

    assert coord._constraint_config is dto
    coord._pilz.set_constraints.assert_called_once_with(dto)
    coord._ompl.set_constraints.assert_called_once_with(dto)


def test_set_ompl_tuning_forwards_to_ompl():
    """set_ompl_tuning() forwards tuning parameters to the OMPL sub-planner."""
    coord, _, _ = _build_coordinator()
    coord.set_ompl_tuning(7.5, 4)
    coord._ompl.set_tuning.assert_called_once_with(7.5, 4)


def test_wait_for_service_requires_all_available():
    """wait_for_service() returns True only if scene and both sub-planners are available."""
    coord, _, _ = _build_coordinator()
    coord._scene_client.wait_for_service.return_value = True
    coord._pilz.wait_for_service.return_value = True
    coord._ompl.wait_for_service.return_value = True
    assert coord.wait_for_service(1.0) is True

    coord._ompl.wait_for_service.return_value = False
    assert coord.wait_for_service(1.0) is False


def test_wait_for_service_false_when_scene_unavailable():
    """wait_for_service() short-circuits to False when the scene service is unavailable."""
    coord, _, _ = _build_coordinator()
    coord._scene_client.wait_for_service.return_value = False
    assert coord.wait_for_service(1.0) is False
    coord._pilz.wait_for_service.assert_not_called()


def test_wait_for_service_false_before_activate():
    """wait_for_service() returns False before on_activate() (no scene client)."""
    mock_node = MagicMock(spec=LifecycleNode)
    mock_node.get_logger.return_value = MagicMock(spec=RcutilsLogger)
    coord = PlanningCoordinator(mock_node, 'ur_manipulator')
    assert coord.wait_for_service(1.0) is False


def test_on_activate_creates_scene_client_and_activates_subservices():
    """on_activate() creates the scene client and activates both sub-planners."""
    mock_node = MagicMock(spec=LifecycleNode)
    mock_node.get_logger.return_value = MagicMock(spec=RcutilsLogger)
    mock_node.create_client.return_value = MagicMock()

    coord = PlanningCoordinator(mock_node, 'ur_manipulator')
    coord._pilz = MagicMock()
    coord._ompl = MagicMock()
    coord.on_activate()

    assert coord._scene_client is not None
    srv_names = [c.kwargs.get('srv_name') for c in mock_node.create_client.call_args_list]
    assert _SCENE_SRV in srv_names
    coord._pilz.on_activate.assert_called_once()
    coord._ompl.on_activate.assert_called_once()


def test_on_deactivate_destroys_scene_client_and_deactivates_subservices():
    """on_deactivate() destroys the scene client and deactivates both sub-planners."""
    coord, _, _ = _build_coordinator()
    scene_client = coord._scene_client

    coord.on_deactivate()

    coord._node.destroy_client.assert_any_call(scene_client)  # type: ignore
    assert coord._scene_client is None
    coord._pilz.on_deactivate.assert_called_once()
    coord._ompl.on_deactivate.assert_called_once()

# endregion: delegation / lifecycle tests
