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
"""Unit tests for the async OmplPlannerService.plan_group_async — all ROS 2 deps mocked."""

from typing import Tuple
from unittest.mock import MagicMock

import pytest
from rclpy.lifecycle import LifecycleNode
from rclpy.impl.rcutils_logger import RcutilsLogger
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetMotionPlan

from movement_controller.enums.motion_type_enum import MotionTypeEnum
from movement_controller.models.constraint_config_dto import ConstraintConfigDTO
from movement_controller.models.plan_result_dto import PlanResultDTO
from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO
from movement_controller.services.ompl_planner_service import OmplPlannerService


# Resolve TYPE_CHECKING-only forward references so Pydantic can instantiate PlanResultDTO in tests.
PlanResultDTO.model_rebuild(_types_namespace={'MotionSequenceResponse': object})

_UUID = 'aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa'
_UUID2 = 'bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb'
_PLAN_SRV = 'plan_kinematic_path'


def _make_path_dto(**overrides) -> TrajectoryPathDTO:
    """Build a minimal valid PTP TrajectoryPathDTO with sensible defaults."""
    defaults: dict = {
        'path_id': _UUID,
        'motion_type': MotionTypeEnum.PTP,
        'target_pose': PoseStamped(),
        'tool_frame': 'tool0',
    }
    return TrajectoryPathDTO(**{**defaults, **overrides})


def _make_response(error_val: int = MoveItErrorCodes.SUCCESS) -> GetMotionPlan.Response:
    """Build a real GetMotionPlan.Response with the given error code."""
    resp = GetMotionPlan.Response()
    resp.motion_plan_response.error_code.val = error_val
    return resp


def _make_sync_future(response):
    """Return a mock ROS future that invokes add_done_callback synchronously."""
    future = MagicMock()
    future.result.return_value = response

    def add_done_callback(cb):
        cb(future)

    future.add_done_callback = add_done_callback
    return future


def _build_service(
    response=None,
    service_available: bool = True,
) -> Tuple[OmplPlannerService, MagicMock, dict]:
    """Build an activated OmplPlannerService backed by a synchronous mock client.

    Sync futures cause plan_group_async's done-callback to fire immediately, so the
    on_result callback is invoked before plan_group_async() returns.
    """
    resp = response if response is not None else _make_response()
    captured: dict = {}

    mock_client = MagicMock()
    mock_client.wait_for_service.return_value = service_available

    def call_async(req):
        captured['req'] = req
        return _make_sync_future(resp)

    mock_client.call_async.side_effect = call_async

    mock_node = MagicMock(spec=LifecycleNode)
    mock_node.get_logger.return_value = MagicMock(spec=RcutilsLogger)
    mock_node.create_client.return_value = mock_client

    svc = OmplPlannerService(mock_node, 'ur_manipulator')
    svc.on_activate()
    return svc, mock_client, captured


# region: constructor and lifecycle tests
def test_constructor_stores_group_name():
    """Constructor stores the planning group name."""
    mock_node = MagicMock(spec=LifecycleNode)
    mock_node.get_logger.return_value = MagicMock(spec=RcutilsLogger)
    svc = OmplPlannerService(mock_node, 'ur_manipulator')
    assert svc._group_name == 'ur_manipulator'


def test_constructor_no_client_before_activate():
    """The service client is NOT created in __init__; it is created in on_activate()."""
    mock_node = MagicMock(spec=LifecycleNode)
    mock_node.get_logger.return_value = MagicMock(spec=RcutilsLogger)
    svc = OmplPlannerService(mock_node, 'ur_manipulator')
    assert svc._plan_client is None
    mock_node.create_client.assert_not_called()


def test_on_activate_creates_client():
    """on_activate() creates the plan_kinematic_path GetMotionPlan client."""
    svc, mock_client, _ = _build_service()
    assert svc._plan_client is mock_client
    _, kwargs = svc._node.create_client.call_args
    assert kwargs['srv_name'] == _PLAN_SRV
    assert kwargs['srv_type'] is GetMotionPlan


def test_on_deactivate_destroys_client():
    """on_deactivate() destroys the client and clears the reference."""
    svc, mock_client, _ = _build_service()
    svc.on_deactivate()
    svc._node.destroy_client.assert_called_once_with(mock_client)
    assert svc._plan_client is None


def test_wait_for_service_before_activate_returns_false():
    """wait_for_service() returns False if called before activation."""
    mock_node = MagicMock(spec=LifecycleNode)
    mock_node.get_logger.return_value = MagicMock(spec=RcutilsLogger)
    svc = OmplPlannerService(mock_node, 'ur_manipulator')
    assert svc.wait_for_service(1.0) is False


def test_wait_for_service_true_and_false():
    """wait_for_service() reflects the underlying client availability."""
    svc, _, _ = _build_service(service_available=True)
    assert svc.wait_for_service(1.0) is True

    svc2, _, _ = _build_service(service_available=False)
    assert svc2.wait_for_service(1.0) is False


# endregion: constructor and lifecycle tests


# region: tuning tests
def test_set_tuning_updates_fields():
    """set_tuning() updates the planning time and attempts."""
    svc, _, _ = _build_service()
    svc.set_tuning(12.5, 8)
    assert svc._planning_time == 12.5
    assert svc._num_planning_attempts == 8


# endregion: tuning tests


# region: plan_group_async tests
def test_plan_group_async_success_wraps_single_trajectory():
    """A successful plan delivers success with the single trajectory wrapped in motion_plan."""
    svc, _, _ = _build_service(response=_make_response(MoveItErrorCodes.SUCCESS))
    results: list[PlanResultDTO] = []

    svc.plan_group_async([_make_path_dto()], RobotState(), results.append)

    assert len(results) == 1
    result = results[0]
    assert result.success is True
    assert result.error_message == ''
    assert result.path_ids == [_UUID]
    assert result.blended is False
    assert result.motion_plan is not None
    assert len(result.motion_plan.planned_trajectories) == 1


def test_plan_group_async_builds_ompl_request():
    """The request uses the ompl pipeline, RRTConnect planner, group, tuning, and start state."""
    svc, _, captured = _build_service()
    svc.set_tuning(9.0, 3)
    start = RobotState()
    svc.plan_group_async([_make_path_dto()], start, lambda _dto: None)

    mpr = captured['req'].motion_plan_request
    assert mpr.pipeline_id == 'ompl'
    assert mpr.planner_id == 'RRTConnect'
    assert mpr.group_name == 'ur_manipulator'
    assert mpr.allowed_planning_time == 9.0
    assert mpr.num_planning_attempts == 3
    assert len(mpr.goal_constraints) == 1
    # No constraint config → scaling factors default to 1.0
    assert mpr.max_velocity_scaling_factor == 1.0
    assert mpr.max_acceleration_scaling_factor == 1.0


def test_plan_group_async_uses_joint_scaling_factors():
    """PTP scaling factors are derived from joint_speed/joint_acceleration (D-7/#7)."""
    svc, _, captured = _build_service()
    svc.set_constraints(
        ConstraintConfigDTO(max_joint_speed=2.0, max_joint_acceleration=4.0)
    )
    svc.plan_group_async(
        [_make_path_dto(joint_speed=1.0, joint_acceleration=1.0)], RobotState(), lambda _dto: None
    )

    mpr = captured['req'].motion_plan_request
    assert mpr.max_velocity_scaling_factor == pytest.approx(0.5)
    assert mpr.max_acceleration_scaling_factor == pytest.approx(0.25)


def test_plan_group_async_client_not_initialised():
    """plan_group_async() fails cleanly if called before activation."""
    mock_node = MagicMock(spec=LifecycleNode)
    mock_node.get_logger.return_value = MagicMock(spec=RcutilsLogger)
    svc = OmplPlannerService(mock_node, 'ur_manipulator')
    results: list[PlanResultDTO] = []

    svc.plan_group_async([_make_path_dto()], RobotState(), results.append)

    assert len(results) == 1
    assert results[0].success is False
    assert 'not initialised' in results[0].error_message


def test_plan_group_async_empty_group_yields_failure():
    """An empty group is rejected with a failure DTO before any call."""
    svc, mock_client, _ = _build_service()
    results: list[PlanResultDTO] = []

    svc.plan_group_async([], RobotState(), results.append)

    assert len(results) == 1
    assert results[0].success is False
    mock_client.call_async.assert_not_called()


def test_plan_group_async_multi_path_plans_only_first():
    """A non-isolated group (>1 path) plans only the first path (grouper guarantees isolation, D-2)."""
    svc, _, _ = _build_service()
    results: list[PlanResultDTO] = []
    group = [_make_path_dto(path_id=_UUID), _make_path_dto(path_id=_UUID2)]

    svc.plan_group_async(group, RobotState(), results.append)

    assert len(results) == 1
    assert results[0].success is True
    assert results[0].path_ids == [_UUID]


def test_plan_group_async_planning_failed_reports_no_path():
    """PLANNING_FAILED is surfaced as a 'no collision-free path' message."""
    svc, _, _ = _build_service(response=_make_response(MoveItErrorCodes.PLANNING_FAILED))
    results: list[PlanResultDTO] = []

    svc.plan_group_async([_make_path_dto()], RobotState(), results.append)

    assert results[0].success is False
    assert 'no collision-free path' in results[0].error_message.lower()


def test_plan_group_async_timed_out_reports_no_path():
    """TIMED_OUT is surfaced as a 'no collision-free path' message."""
    svc, _, _ = _build_service(response=_make_response(MoveItErrorCodes.TIMED_OUT))
    results: list[PlanResultDTO] = []

    svc.plan_group_async([_make_path_dto()], RobotState(), results.append)

    assert results[0].success is False
    assert 'no collision-free path' in results[0].error_message.lower()


def test_plan_group_async_goal_in_collision_message():
    """GOAL_IN_COLLISION is surfaced with a goal-specific message."""
    svc, _, _ = _build_service(response=_make_response(MoveItErrorCodes.GOAL_IN_COLLISION))
    results: list[PlanResultDTO] = []

    svc.plan_group_async([_make_path_dto()], RobotState(), results.append)

    assert results[0].success is False
    assert 'goal state is in collision' in results[0].error_message.lower()


# endregion: plan_group_async tests
