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
"""Unit tests for the client-side MovementSession (fake services and clock, no ROS required)."""

import math

import pytest

from movement_controller.services.movement_session import (
    DEFAULT_SESSION_EXTEND_THRESHOLD_S,
    DEFAULT_SESSION_TIMEOUT_S,
    MovementSession,
)
from movement_controller.srv import CloseSession, InSession


class FakeMovementController:
    """Stand-in for the in_session / close_session services."""

    def __init__(self) -> None:
        self.in_session_requests: list[InSession.Request] = []
        self.close_requests: list[CloseSession.Request] = []
        self.accept = True
        self.close_accept = True

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
        response.success = self.close_accept
        response.error_message = '' if self.close_accept else 'wrong id'
        return response


@pytest.fixture
def controller():
    return FakeMovementController()


@pytest.fixture
def clock():
    return [1000.0]


@pytest.fixture
def session(controller, clock):
    return MovementSession(controller.in_session, controller.close, lambda: clock[0])


def test_first_ensure_valid_acquires_with_60s_timeout(session, controller):
    assert session.ensure_valid() == ''
    assert session.session_id == 'sid-1'
    request = controller.in_session_requests[0]
    assert request.session_id == ''
    assert request.session_timeout_s == DEFAULT_SESSION_TIMEOUT_S == 60.0


def test_no_extension_while_enough_time_remains(session, controller, clock):
    session.ensure_valid()
    clock[0] += DEFAULT_SESSION_TIMEOUT_S - DEFAULT_SESSION_EXTEND_THRESHOLD_S - 1.0  # 31 s remain
    assert session.ensure_valid() == ''
    assert len(controller.in_session_requests) == 1


def test_extends_with_session_id_when_less_than_30s_remain(session, controller, clock):
    session.ensure_valid()
    clock[0] += 31.0  # 29 s remain
    assert session.ensure_valid() == ''
    extension = controller.in_session_requests[1]
    assert extension.session_id == 'sid-1'
    assert extension.session_timeout_s == 60.0
    clock[0] += 29.0  # 31 s remain after the restart: no further extension
    session.ensure_valid()
    assert len(controller.in_session_requests) == 2


def test_failed_acquire_returns_error_and_no_session(session, controller):
    controller.accept = False
    error = session.ensure_valid()
    assert 'locked' in error
    assert session.session_id == ''


def test_unavailable_service_returns_error():
    session = MovementSession(lambda r: None, lambda r: None, lambda: 0.0)
    assert 'not available' in session.ensure_valid()


def test_close_releases_and_is_idempotent(session, controller):
    session.ensure_valid()
    assert session.close() == ''
    assert controller.close_requests[0].session_id == 'sid-1'
    assert session.close() == ''
    assert len(controller.close_requests) == 1


def test_defaults_are_60s_timeout_and_30s_threshold():
    assert DEFAULT_SESSION_TIMEOUT_S == 60.0
    assert DEFAULT_SESSION_EXTEND_THRESHOLD_S == 30.0


def test_custom_timeout_and_threshold_are_used(controller, clock):
    session = MovementSession(
        controller.in_session, controller.close, lambda: clock[0],
        timeout_s=20.0, extend_threshold_s=5.0)
    session.ensure_valid()
    assert controller.in_session_requests[0].session_timeout_s == 20.0

    clock[0] += 14.0  # 6 s remain, above the 5 s threshold
    session.ensure_valid()
    assert len(controller.in_session_requests) == 1

    clock[0] += 2.0  # 4 s remain
    session.ensure_valid()
    assert len(controller.in_session_requests) == 2
    assert controller.in_session_requests[1].session_timeout_s == 20.0

    clock[0] += 14.0  # deadline was restarted with the custom timeout: 6 s remain
    session.ensure_valid()
    assert len(controller.in_session_requests) == 2


@pytest.mark.parametrize('kwargs', [
    {'timeout_s': 0.0}, {'timeout_s': -1.0}, {'timeout_s': math.nan}, {'timeout_s': math.inf},
    {'extend_threshold_s': -1.0}, {'extend_threshold_s': math.nan}, {'extend_threshold_s': math.inf},
])
def test_invalid_timeout_or_threshold_raises(controller, kwargs):
    with pytest.raises(ValueError):
        MovementSession(controller.in_session, controller.close, lambda: 0.0, **kwargs)


def test_close_without_session_does_not_call_service(session, controller):
    assert session.close() == ''
    assert controller.close_requests == []


def test_close_reports_rejection_and_forgets_session(session, controller):
    session.ensure_valid()
    controller.close_accept = False
    assert 'wrong id' in session.close()
    assert session.session_id == ''


def test_close_reports_unavailable_service():
    session = MovementSession(lambda r: InSession.Response(success=True, session_id='s'), lambda r: None, lambda: 0.0)
    session.ensure_valid()
    assert 'not available' in session.close()
