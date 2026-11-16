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
"""Unit tests for the movement-session handling of the localization package."""

import threading
from unittest.mock import MagicMock, patch

import pytest
from geometry_msgs.msg import PoseStamped

from movement_controller.srv import CloseSession, InSession

from localization.exceptions import LocalizationCancelledError, MotionError
from localization.models.localizer_config_dto import LocalizerConfigDTO
from localization.services.mock_localization_service import MockLocalizationService
from localization.services.motion_client import MotionClient


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
        return response

    def close(self, request):
        self.close_requests.append(request)
        response = CloseSession.Response()
        response.success = True
        return response


@pytest.fixture
def controller():
    return FakeMovementController()


@pytest.fixture
def motion(controller):
    node = MagicMock()
    node.get_clock.return_value.now.return_value.nanoseconds = 0
    tf_lookup = MagicMock()
    tf_lookup.transform_pose.return_value = PoseStamped()
    with patch('localization.services.motion_client.ActionClient'), \
            patch('localization.services.motion_client.call_service') as call_service:
        call_service.side_effect = lambda client, request, timeout_s: (
            controller.in_session(request) if isinstance(request, InSession.Request)
            else controller.close(request))
        yield MotionClient(node, '/mc/execute_trajectory', 'tcp', tf_lookup, MagicMock(),
                           '/mc/in_session', '/mc/close_session')


def test_move_to_without_session_raises(motion):
    with pytest.raises(MotionError, match='no movement session'):
        motion.move_to((0, 0, 0, 0, 0, 0), 'base_link', threading.Event())


def test_open_session_failure_raises(motion, controller):
    controller.accept = False
    with pytest.raises(MotionError, match='locked'):
        motion.open_session()


def test_move_to_sends_session_id_in_goal(motion):
    motion.open_session()
    with patch('localization.services.motion_client.execute_goal') as execute_goal, \
            patch('localization.services.motion_client.pose6_to_pose_stamped', return_value=PoseStamped()):
        execute_goal.return_value = MagicMock(success=True)
        motion.move_to((0, 0, 0, 0, 0, 0), 'base_link', threading.Event())
    assert execute_goal.call_args.args[1].session_id == 'sid-1'


def test_close_session_releases_and_destroy_closes_open_session(motion, controller):
    motion.open_session()
    motion.close_session()
    assert len(controller.close_requests) == 1
    motion.open_session()
    motion.destroy()
    assert len(controller.close_requests) == 2


def test_open_session_twice_raises(motion):
    motion.open_session()
    with pytest.raises(MotionError):
        motion.open_session()


def test_config_service_names():
    config = LocalizerConfigDTO(
        movement_controller_node_name='movement_controller',
        initial_localization_pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        calibration_poses_file='/tmp/poses.yaml',
    )
    assert config.in_session_service == '/movement_controller/in_session'
    assert config.close_session_service == '/movement_controller/close_session'


def _service_with_mocks():
    service = MockLocalizationService(MagicMock(), MagicMock(poses=[(1,) * 6, (2,) * 6]))
    service._motion = MagicMock()
    service._camera = MagicMock()
    service._tf_publisher = MagicMock()
    service._tf_lookup = MagicMock()
    return service


def test_collect_images_runs_all_moves_in_one_session():
    service = _service_with_mocks()
    with patch.object(service, '_settle_and_capture', return_value=MagicMock()):
        images = service._collect_images(threading.Event())
    assert len(images) == 3
    service._motion.open_session.assert_called_once()
    assert service._motion.move_to.call_count == 3
    service._motion.close_session.assert_called_once()


def test_collect_images_closes_session_on_cancel_and_failure():
    service = _service_with_mocks()
    with patch.object(service, '_settle_and_capture', side_effect=LocalizationCancelledError('cancelled')):
        with pytest.raises(LocalizationCancelledError):
            service._collect_images(threading.Event())
    service._motion.close_session.assert_called_once()

    service = _service_with_mocks()
    service._motion.move_to.side_effect = MotionError('boom')
    with pytest.raises(MotionError):
        service._collect_images(threading.Event())
    service._motion.close_session.assert_called_once()


def test_collect_images_does_not_move_when_session_cannot_be_opened():
    service = _service_with_mocks()
    service._motion.open_session.side_effect = MotionError('locked')
    with pytest.raises(MotionError):
        service._collect_images(threading.Event())
    service._motion.move_to.assert_not_called()
    service._motion.close_session.assert_not_called()
