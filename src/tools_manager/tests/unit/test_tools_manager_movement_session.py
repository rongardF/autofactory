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
"""Unit tests for the movement-session handling of ToolsManager."""

import threading
from unittest.mock import MagicMock

import pytest
import rclpy

from movement_controller.action import ExecuteTrajectory
from movement_controller.srv import CloseSession, InSession
from tools_manager.action import MountTool
from movement_controller.services.movement_session import MovementSession
from tools_manager.tools_manager import ToolsManager


class FakeMovementController:
    """Stand-in for the in_session / close_session services."""

    def __init__(self) -> None:
        self.in_session_requests: list[InSession.Request] = []
        self.close_requests: list[CloseSession.Request] = []
        self.accept = True

    def in_session(self, request):
        self.in_session_requests.append(request)
        response = InSession.Response()
        response.success = self.accept
        response.error_message = '' if self.accept else 'locked'
        response.session_id = request.session_id or 'sid-1'
        response.session_timeout_s = request.session_timeout_s
        return response

    def close(self, request):
        self.close_requests.append(request)
        response = CloseSession.Response()
        response.success = True
        return response


@pytest.fixture
def controller():
    return FakeMovementController()


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros_context):
    n = ToolsManager()
    yield n
    n.destroy_node()


def test_call_action_fails_without_session(node):
    assert node._call_action(MagicMock(), ExecuteTrajectory.Goal()) is False


def test_call_action_fails_when_session_cannot_be_opened(node, controller):
    controller.accept = False
    node._movement_session = MovementSession(controller.in_session, controller.close, lambda: 0.0)
    client = MagicMock()
    assert node._call_action(client, ExecuteTrajectory.Goal()) is False
    client.send_goal_async.assert_not_called()


def test_call_action_sets_session_id_on_goal(node, controller):
    node._movement_session = MovementSession(controller.in_session, controller.close, lambda: 0.0)
    client = MagicMock()
    client.wait_for_server.return_value = False  # stop right after the session handling
    goal = ExecuteTrajectory.Goal()
    node._call_action(client, goal)
    assert goal.session_id == 'sid-1'


def test_operation_session_is_closed_on_success_and_exception(node, controller):
    node._in_session_client = MagicMock()
    node._close_session_client = MagicMock()
    node._call_service = lambda client, request, timeout=5.0: (
        controller.in_session(request) if client is node._in_session_client else controller.close(request))

    def operation(goal_handle):
        node._movement_session.ensure_valid()
        return 'result'

    def failing_operation(goal_handle):
        node._movement_session.ensure_valid()
        raise RuntimeError('boom')

    assert node._run_in_movement_session(MagicMock(), MountTool.Result, operation) == 'result'
    assert len(controller.close_requests) == 1
    assert node._movement_session is None

    with pytest.raises(RuntimeError):
        node._run_in_movement_session(MagicMock(), MountTool.Result, failing_operation)
    assert len(controller.close_requests) == 2
    assert node._movement_session is None
    assert node._tool_action_lock.acquire(blocking=False)  # lock was released again
    node._tool_action_lock.release()


def test_operation_aborts_when_another_operation_is_running(node):
    goal_handle = MagicMock()
    operation = MagicMock()
    results = []
    node._tool_action_lock.acquire()
    try:
        # RLock is re-entrant, so the competing request must come from another thread
        thread = threading.Thread(
            target=lambda: results.append(node._run_in_movement_session(goal_handle, MountTool.Result, operation)))
        thread.start()
        thread.join()
    finally:
        node._tool_action_lock.release()
    assert results[0].success is False
    goal_handle.abort.assert_called_once()
    operation.assert_not_called()
