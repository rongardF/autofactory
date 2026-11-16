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
"""Unit tests for the session services and session-aware goal handling of MovementController."""

from unittest.mock import MagicMock, patch

import pytest
import rclpy
from rclpy.action import GoalResponse, CancelResponse
from rclpy.lifecycle.node import TransitionCallbackReturn, LifecycleState
from rclpy.parameter import Parameter
from geometry_msgs.msg import Point, PoseStamped

from movement_controller.action import ExecuteTrajectory
from movement_controller.msg import TrajectoryPath
from movement_controller.services import PlanningCoordinator
from movement_controller.srv import CloseSession, InSession
from movement_controller.movement_controller import MovementController

_UUID1 = '00000000-0000-4000-8000-000000000001'


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros_context):
    n = MovementController()
    yield n
    n.destroy_node()


def _goal(session_id: str = '') -> MagicMock:
    path = MagicMock(spec=TrajectoryPath)
    path.path_id = _UUID1
    path.motion_type = 'LIN'
    path.blend_radius = 0.0
    path.target_pose = PoseStamped()
    path.circ_point = Point()
    path.cartesian_speed = 0.0
    path.cartesian_acceleration = 0.0
    path.joint_speed = 0.0
    path.joint_acceleration = 0.0
    path.tool_frame = ''
    path.circ_type = 'interim'
    goal = MagicMock(spec=ExecuteTrajectory.Goal)
    goal.paths = [path]
    goal.session_id = session_id
    return goal


def _configure(node: MovementController) -> None:
    with patch('movement_controller.movement_controller.PlanningCoordinator') as coordinator:
        coordinator.return_value = MagicMock(spec=PlanningCoordinator)
        assert node.on_configure(LifecycleState(label='unconfigured', state_id=1)) == TransitionCallbackReturn.SUCCESS


def _acquire(node: MovementController, session_id: str = '', timeout: float = -1.0) -> InSession.Response:
    request = InSession.Request()
    request.session_id = session_id
    request.session_timeout_s = timeout
    return node._in_session_callback(request, InSession.Response())


def _close(node: MovementController, session_id: str) -> CloseSession.Response:
    request = CloseSession.Request()
    request.session_id = session_id
    return node._close_session_callback(request, CloseSession.Response())


def test_default_session_timeout_parameter(node):
    assert node.get_parameter('session_timeout_s').get_parameter_value().double_value == 10.0


def test_configure_fails_for_non_positive_session_timeout(node):
    node.set_parameters([Parameter('session_timeout_s', Parameter.Type.DOUBLE, 0.0)])
    with patch('movement_controller.movement_controller.PlanningCoordinator'):
        result = node.on_configure(LifecycleState(label='unconfigured', state_id=1))
    assert result == TransitionCallbackReturn.FAILURE


def test_services_fail_gracefully_when_not_configured(node):
    assert _acquire(node).success is False
    assert _close(node, 'x').success is False


def test_in_session_acquire_uses_parameter_default(node):
    node.set_parameters([Parameter('session_timeout_s', Parameter.Type.DOUBLE, 7.0)])
    _configure(node)
    response = _acquire(node)
    assert response.success is True
    assert response.session_id != ''
    assert response.session_timeout_s == 7.0


def test_in_session_busy_returns_error_message(node):
    _configure(node)
    _acquire(node)
    response = _acquire(node)
    assert response.success is False
    assert response.error_message != ''


def test_in_session_extend_returns_same_id(node):
    _configure(node)
    sid = _acquire(node).session_id
    response = _acquire(node, sid, 30.0)
    assert response.success is True
    assert response.session_id == sid
    assert response.session_timeout_s == 30.0


def test_close_session_releases_lock(node):
    _configure(node)
    sid = _acquire(node).session_id
    assert _close(node, sid).success is True
    assert _acquire(node).success is True


def test_close_session_wrong_id_fails(node):
    _configure(node)
    _acquire(node)
    assert _close(node, 'wrong').success is False


def test_goal_without_session_accepted_for_any_id(node):
    _configure(node)
    assert node._goal_callback(_goal('')) == GoalResponse.ACCEPT
    node._is_executing = False
    assert node._goal_callback(_goal('garbage')) == GoalResponse.ACCEPT
    node._is_executing = False


def test_goal_rejected_when_locked_and_id_empty_or_wrong(node):
    _configure(node)
    _acquire(node)
    assert node._goal_callback(_goal('')) == GoalResponse.REJECT
    assert node._goal_callback(_goal('wrong')) == GoalResponse.REJECT
    assert node._is_executing is False


def test_goal_accepted_when_locked_and_id_correct(node):
    _configure(node)
    sid = _acquire(node).session_id
    assert node._goal_callback(_goal(sid)) == GoalResponse.ACCEPT
    node._is_executing = False


def test_goal_with_correct_session_still_rejected_while_executing(node):
    _configure(node)
    sid = _acquire(node).session_id
    node._is_executing = True
    assert node._goal_callback(_goal(sid)) == GoalResponse.REJECT
    node._is_executing = False


def test_session_can_be_acquired_while_session_less_goal_executes(node):
    _configure(node)
    node._is_executing = True
    assert _acquire(node).success is True
    node._is_executing = False


def test_cancel_allowed_for_non_owner(node):
    _configure(node)
    _acquire(node)
    assert node._cancel_callback(MagicMock()) == CancelResponse.ACCEPT


def test_on_activate_creates_private_session_services_and_deactivate_clears(node):
    _configure(node)
    node._planner_service.wait_for_service.return_value = True
    with patch('movement_controller.movement_controller.ActionServer') as action_server, \
         patch('movement_controller.movement_controller.ActionClient'):
        result = node.on_activate(LifecycleState(label='inactive', state_id=2))
    assert result == TransitionCallbackReturn.SUCCESS
    assert action_server.call_args.args[2] == '~/execute_trajectory'
    assert node._in_session_service is not None
    assert node._close_session_service is not None
    names = {name for name, _ in node.get_service_names_and_types() if 'session' in name}
    assert names == {'/in_session'.replace('/', '/' + node.get_name() + '/', 1),
                     '/close_session'.replace('/', '/' + node.get_name() + '/', 1)}

    _acquire(node)
    node.on_deactivate(LifecycleState(label='active', state_id=3))
    assert node._in_session_service is None
    assert node._close_session_service is None
    assert _acquire(node).success is True  # previous session was cleared by deactivate
