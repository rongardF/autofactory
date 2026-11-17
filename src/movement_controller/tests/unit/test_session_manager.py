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
"""Unit tests for SessionManager (fake clock, no ROS required)."""

import math

import pytest

from movement_controller.services.session_manager import SessionManager


class FakeClock:
    def __init__(self) -> None:
        self.t = 100.0

    def __call__(self) -> float:
        return self.t


@pytest.fixture
def clock():
    return FakeClock()


@pytest.fixture
def manager(clock):
    return SessionManager(default_timeout_s=10.0, now=clock)


def test_acquire_when_free_returns_uuid_and_default_timeout(manager):
    result = manager.acquire_or_extend('', -1.0)
    assert result.success is True
    assert len(result.session_id) == 36
    assert result.session_timeout_s == 10.0


def test_acquire_with_custom_timeout(manager):
    result = manager.acquire_or_extend('', 42.5)
    assert result.success is True
    assert result.session_timeout_s == 42.5


def test_acquire_when_locked_is_rejected(manager):
    manager.acquire_or_extend('', -1.0)
    result = manager.acquire_or_extend('', -1.0)
    assert result.success is False
    assert result.error_message != ''
    assert result.session_id == ''


@pytest.mark.parametrize('timeout', [0.0, -2.0, -0.5, math.nan, math.inf, -math.inf])
def test_invalid_timeouts_are_rejected_without_state_change(manager, timeout):
    result = manager.acquire_or_extend('', timeout)
    assert result.success is False
    assert manager.is_goal_allowed('anything') is True  # no session was created


def test_large_timeout_is_accepted(manager):
    assert manager.acquire_or_extend('', 1e6).success is True


def test_extend_keeps_id_and_restarts_timeout(manager, clock):
    sid = manager.acquire_or_extend('', 10.0).session_id
    clock.t += 8.0
    result = manager.acquire_or_extend(sid, 10.0)
    assert result.success is True
    assert result.session_id == sid
    clock.t += 8.0  # 16 s after acquire: would be expired if not restarted
    assert manager.is_goal_allowed('other') is False
    clock.t += 2.0  # 10 s after the extension
    assert manager.is_goal_allowed('other') is True


def test_extend_does_not_accumulate_remaining_time(manager, clock):
    sid = manager.acquire_or_extend('', 100.0).session_id
    manager.acquire_or_extend(sid, 5.0)
    clock.t += 5.0
    assert manager.is_goal_allowed('other') is True


def test_extend_with_default_timeout(manager):
    sid = manager.acquire_or_extend('', 50.0).session_id
    result = manager.acquire_or_extend(sid, -1.0)
    assert result.success is True
    assert result.session_timeout_s == 10.0


def test_extend_with_wrong_id_is_rejected(manager):
    manager.acquire_or_extend('', -1.0)
    result = manager.acquire_or_extend('wrong', -1.0)
    assert result.success is False


def test_extend_without_active_session_is_rejected(manager):
    result = manager.acquire_or_extend('stale-id', -1.0)
    assert result.success is False


def test_extend_after_expiry_is_rejected(manager, clock):
    sid = manager.acquire_or_extend('', 5.0).session_id
    clock.t += 5.0
    assert manager.acquire_or_extend(sid, 5.0).success is False
    assert manager.acquire_or_extend('', 5.0).success is True


def test_expiry_releases_lock_for_new_acquire_and_notifies(clock):
    expired = []
    manager = SessionManager(10.0, clock, on_expired=expired.append)
    sid = manager.acquire_or_extend('', 5.0).session_id
    clock.t += 5.0
    assert manager.acquire_or_extend('', 5.0).success is True
    assert expired == [sid]


def test_close_with_matching_id_releases(manager):
    sid = manager.acquire_or_extend('', -1.0).session_id
    assert manager.close(sid).success is True
    assert manager.acquire_or_extend('', -1.0).success is True


@pytest.mark.parametrize('wrong_id', ['', 'wrong'])
def test_close_with_wrong_id_fails_and_keeps_lock(manager, wrong_id):
    sid = manager.acquire_or_extend('', -1.0).session_id
    assert manager.close(wrong_id).success is False
    assert manager.is_goal_allowed(sid) is True
    assert manager.is_goal_allowed('') is False


def test_close_without_session_is_idempotent_success(manager):
    assert manager.close('whatever').success is True
    assert manager.close('').success is True


def test_close_after_expiry_succeeds(manager, clock):
    sid = manager.acquire_or_extend('', 5.0).session_id
    clock.t += 6.0
    assert manager.close(sid).success is True


def test_goal_allowed_without_session_for_any_id(manager):
    assert manager.is_goal_allowed('') is True
    assert manager.is_goal_allowed('garbage') is True


def test_goal_with_lock_requires_matching_id(manager):
    sid = manager.acquire_or_extend('', -1.0).session_id
    assert manager.is_goal_allowed(sid) is True
    assert manager.is_goal_allowed('') is False
    assert manager.is_goal_allowed('wrong') is False


def test_goal_allowed_again_after_expiry(manager, clock):
    manager.acquire_or_extend('', 5.0)
    clock.t += 5.0
    assert manager.is_goal_allowed('') is True


def test_clear_drops_session(manager):
    manager.acquire_or_extend('', -1.0)
    manager.clear()
    assert manager.is_goal_allowed('') is True


@pytest.mark.parametrize('bad', [0.0, -1.0, math.nan, math.inf])
def test_invalid_default_timeout_raises(clock, bad):
    with pytest.raises(ValueError):
        SessionManager(default_timeout_s=bad, now=clock)
