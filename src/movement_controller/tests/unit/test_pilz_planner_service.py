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
"""Unit tests for the slimmed PilzPlannerService.plan_group_async — all ROS 2 deps mocked."""

from typing import Tuple
from unittest.mock import MagicMock

from rclpy.lifecycle import LifecycleNode
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.client import Client
from geometry_msgs.msg import PoseStamped

from moveit_msgs.msg import (
    MoveItErrorCodes,
    MotionSequenceResponse,
    RobotState,
    RobotTrajectory,
)
from moveit_msgs.action import MoveGroupSequence
from trajectory_msgs.msg import JointTrajectoryPoint

from movement_controller.enums.motion_type_enum import MotionTypeEnum
from movement_controller.models.constraint_config_dto import ConstraintConfigDTO
from movement_controller.models.plan_result_dto import PlanResultDTO
from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO
from movement_controller.services.pilz_planner_service import PilzPlannerService


# Resolve TYPE_CHECKING-only forward references so Pydantic can instantiate these models in tests.
PlanResultDTO.model_rebuild(_types_namespace={'MotionSequenceResponse': object})

_UUID = 'aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa'
_UUID2 = 'bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb'

_SEQ_SRV = 'plan_sequence_path'


def _make_path_dto(**overrides) -> TrajectoryPathDTO:
    """Build a minimal valid TrajectoryPathDTO with sensible defaults."""
    defaults: dict = {
        'path_id': _UUID,
        'motion_type': MotionTypeEnum.LIN,
        'target_pose': PoseStamped(),
    }
    return TrajectoryPathDTO(**{**defaults, **overrides})


def _make_sync_future(response):
    """Return a mock ROS future that invokes add_done_callback synchronously."""
    future = MagicMock()
    future.result.return_value = response

    def add_done_callback(cb):
        cb(future)

    future.add_done_callback = add_done_callback
    return future


def _make_default_seq_response():
    """Build a default successful GetMotionSequence response mock."""
    real_motion_response = MotionSequenceResponse()
    real_motion_response.error_code.val = MoveItErrorCodes.SUCCESS
    mock_traj = MagicMock(spec=RobotTrajectory)
    mock_traj.joint_trajectory.joint_names = ['joint_1']
    joint_traj_point = MagicMock(spec=JointTrajectoryPoint)
    joint_traj_point.positions = [0.0]
    mock_traj.joint_trajectory.points = [joint_traj_point]

    resp = MagicMock()
    resp.response = real_motion_response
    return resp


def _build_service(
    seq_response=None,
    seq_available=True,
) -> Tuple[PilzPlannerService, MagicMock]:
    """Build an activated PilzPlannerService backed by a synchronous mock service client.

    Sync futures cause plan_group_async's done-callback to fire immediately, so the
    on_result callback is invoked before plan_group_async() returns.
    """
    seq_resp = seq_response if seq_response is not None else _make_default_seq_response()

    mock_seq_client = MagicMock()
    mock_seq_client.wait_for_service.return_value = seq_available
    mock_seq_client.call_async.side_effect = lambda req: _make_sync_future(seq_resp)

    mock_node = MagicMock(spec=LifecycleNode)
    mock_node.get_logger.return_value = MagicMock(spec=RcutilsLogger)
    mock_node.create_client.return_value = mock_seq_client

    svc = PilzPlannerService(mock_node, 'ur_manipulator')
    svc.on_activate()
    return svc, mock_seq_client


# region: Constructor and lifecycle tests
def test_constructor_stores_group_name():
    """Constructor stores the planning group name for use in service requests."""
    mock_node = MagicMock(spec=LifecycleNode)
    mock_node.get_logger.return_value = MagicMock(spec=RcutilsLogger)
    svc = PilzPlannerService(mock_node, 'ur_manipulator')
    assert svc._group_name == 'ur_manipulator'
    mock_node.get_logger.assert_called_once()


def test_constructor_no_client_before_activate():
    """The service client is NOT created in __init__; it is created in on_activate()."""
    mock_node = MagicMock(spec=LifecycleNode)
    mock_node.get_logger.return_value = MagicMock(spec=RcutilsLogger)
    svc = PilzPlannerService(mock_node, 'ur_manipulator')
    assert svc._plan_seq_client is None
    mock_node.create_client.assert_not_called()


def test_on_activate_creates_single_client():
    """on_activate() creates only the plan_sequence_path client via node.create_client."""
    mock_node = MagicMock(spec=LifecycleNode)
    mock_node.get_logger.return_value = MagicMock(spec=RcutilsLogger)
    mock_node.create_client.return_value = MagicMock(spec=Client)

    svc = PilzPlannerService(mock_node, 'ur_manipulator')
    svc.on_activate()

    assert mock_node.create_client.call_count == 1
    assert mock_node.create_client.call_args.kwargs['srv_name'] == _SEQ_SRV
    assert svc._plan_seq_client is not None


def test_on_deactivate_destroys_client():
    """on_deactivate() destroys the service client and clears the reference."""
    svc, _ = _build_service()
    seq_client = svc._plan_seq_client

    svc.on_deactivate()

    svc._node.destroy_client.assert_any_call(seq_client)  # type: ignore
    assert svc._plan_seq_client is None

# endregion: Constructor and lifecycle tests


# region: wait_for_service tests
def test_wait_for_service_returns_false_before_activate():
    """wait_for_service() returns False when called before on_activate() (client is None)."""
    mock_node = MagicMock(spec=LifecycleNode)
    mock_node.get_logger.return_value = MagicMock(spec=RcutilsLogger)
    svc = PilzPlannerService(mock_node, 'ur_manipulator')
    assert svc.wait_for_service(timeout_sec=1.0) is False


def test_wait_for_service_delegates_to_client():
    """wait_for_service() delegates to the plan_seq_client with the given timeout."""
    svc, mock_seq_client = _build_service()
    mock_seq_client.wait_for_service.return_value = True

    result = svc.wait_for_service(timeout_sec=3.0)

    assert result is True
    mock_seq_client.wait_for_service.assert_called_with(timeout_sec=3.0)

# endregion: wait_for_service tests


# region: plan_group_async tests
def test_plan_group_async_success_invokes_callback_with_success_dto():
    """plan_group_async() delivers a successful PlanResultDTO to on_result."""
    svc, _ = _build_service()
    results: list[PlanResultDTO] = []

    svc.plan_group_async([_make_path_dto()], RobotState(), results.append)

    assert len(results) == 1
    assert results[0].success is True
    assert results[0].path_ids == [_UUID]
    assert results[0].blended is False


def test_plan_group_async_multi_path_group_sets_blended_and_path_ids():
    """A two-path group yields blended=True and both path IDs in order."""
    svc, _ = _build_service()
    results: list[PlanResultDTO] = []
    group = [_make_path_dto(path_id=_UUID), _make_path_dto(path_id=_UUID2)]

    svc.plan_group_async(group, RobotState(), results.append)

    assert len(results) == 1
    assert results[0].success is True
    assert results[0].blended is True
    assert results[0].path_ids == [_UUID, _UUID2]


def test_plan_group_async_client_not_initialised_yields_failure():
    """plan_group_async() before on_activate() delivers a failure DTO (no client)."""
    mock_node = MagicMock(spec=LifecycleNode)
    mock_node.get_logger.return_value = MagicMock(spec=RcutilsLogger)
    svc = PilzPlannerService(mock_node, 'ur_manipulator')
    results: list[PlanResultDTO] = []

    svc.plan_group_async([_make_path_dto()], RobotState(), results.append)

    assert len(results) == 1
    assert results[0].success is False
    assert 'not initialised' in results[0].error_message


def test_plan_group_async_planning_failure_yields_failure_dto():
    """A GetMotionSequence response with error_code != SUCCESS yields a failure DTO."""
    fail_resp = MagicMock(spec=MoveGroupSequence.Result)
    fail_resp.response.error_code.val = MoveItErrorCodes.PLANNING_FAILED
    svc, _ = _build_service(seq_response=fail_resp)
    results: list[PlanResultDTO] = []

    svc.plan_group_async([_make_path_dto()], RobotState(), results.append)

    assert len(results) == 1
    assert results[0].success is False
    assert results[0].error_message != ''


# endregion: plan_group_async tests


# region: _generate_motion_sequence_request tests
def test_last_item_blend_radius_forced_to_zero():
    """In a 2-path group the last MotionSequenceItem always has blend_radius=0.0 (PILZ constraint)."""
    svc, _ = _build_service()
    path1 = _make_path_dto(path_id=_UUID, blend_radius=0.05)
    path2 = _make_path_dto(path_id=_UUID2, blend_radius=0.05)

    seq_req = svc._generate_motion_sequence_request([path1, path2], RobotState())  # type: ignore

    assert len(seq_req.items) == 2
    assert seq_req.items[0].blend_radius == 0.05, 'First item should keep its original blend_radius'  # type: ignore
    assert seq_req.items[-1].blend_radius == 0.0, 'Last item blend_radius must be forced to 0.0'  # type: ignore


def test_generate_request_maps_motion_type_to_planner_id():
    """_generate_motion_sequence_request maps MotionTypeEnum to the correct PILZ planner_id."""
    svc, _ = _build_service()
    for motion_type, expected_planner_id in [
        (MotionTypeEnum.LIN, 'LIN'),
        (MotionTypeEnum.PTP, 'PTP'),
    ]:
        path = _make_path_dto(motion_type=motion_type)
        seq_req = svc._generate_motion_sequence_request([path], RobotState())
        assert seq_req.items[0].req.planner_id == expected_planner_id  # type: ignore
        assert seq_req.items[0].req.pipeline_id == 'pilz_industrial_motion_planner'  # type: ignore


def test_generate_request_sets_start_state_on_first_item_only():
    """start_state is assigned only to the first MotionSequenceItem."""
    svc, _ = _build_service()
    start = RobotState()
    path1 = _make_path_dto(path_id=_UUID)
    path2 = _make_path_dto(path_id=_UUID2)

    seq_req = svc._generate_motion_sequence_request([path1, path2], start)  # type: ignore

    assert seq_req.items[0].req.start_state is start  # type: ignore
    assert seq_req.items[1].req.start_state is not start  # type: ignore

# endregion: _generate_motion_sequence_request tests


# region: set_constraints and constraint injection tests
def test_set_constraints_stores_dto():
    """set_constraints() stores the dto as _constraint_config."""
    svc, _ = _build_service()
    dto = ConstraintConfigDTO()
    svc.set_constraints(dto)
    assert svc._constraint_config is dto


def test_constraints_injected_into_every_sequence_item():
    """Active workspace constraint is injected into every MotionSequenceItem."""
    from shape_msgs.msg import SolidPrimitive

    svc, _ = _build_service()
    dto = ConstraintConfigDTO(x_min=-1.0, x_max=1.0)
    svc.set_constraints(dto)

    path1 = _make_path_dto(path_id=_UUID)
    path2 = _make_path_dto(path_id=_UUID2)
    seq_req = svc._generate_motion_sequence_request([path1, path2], RobotState())

    assert len(seq_req.items) == 2
    for item in seq_req.items:
        pc_list = item.req.path_constraints.position_constraints
        assert len(pc_list) > 0, 'Each item must have at least one position constraint'
        assert pc_list[0].constraint_region.primitives[0].type == SolidPrimitive.BOX


def test_constraints_not_injected_when_all_disabled():
    """All-sentinel ConstraintConfigDTO → no constraints in generated items."""
    svc, _ = _build_service()
    dto = ConstraintConfigDTO()  # all at sentinel = all disabled
    svc.set_constraints(dto)

    path = _make_path_dto()
    seq_req = svc._generate_motion_sequence_request([path], RobotState())

    item = seq_req.items[0]  # type: ignore
    assert len(item.req.path_constraints.position_constraints) == 0
    assert len(item.req.path_constraints.joint_constraints) == 0
    assert len(item.req.path_constraints.orientation_constraints) == 0

# endregion: set_constraints and constraint injection tests
