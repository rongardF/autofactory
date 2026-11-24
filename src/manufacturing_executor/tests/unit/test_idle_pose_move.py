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
"""Tests of the non-blocking move to the idle pose."""

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from action_msgs.msg import GoalStatus

from manufacturing_executor.exceptions import SystemFailureError
from manufacturing_executor.services.abort_context import AbortContext
from manufacturing_executor.services.async_calls import ActionOutcome
from manufacturing_executor.services.idle_pose_service import IdlePoseMove


class FakeGoal:
    """Stands in for a ``ChildGoal``; ``finish`` delivers the result like the executor thread would."""

    def __init__(self) -> None:
        self.callbacks = []
        self.outcome: ActionOutcome | None = None
        self.cancelled = False

    def add_done_callback(self, callback) -> None:
        self.callbacks.append(callback)

    def finish(self, status: int, success: bool, error: str = '') -> None:
        self.outcome = ActionOutcome(
            status=status, result=SimpleNamespace(success=success, error_message=error)
        )
        for callback in self.callbacks:
            callback()

    def wait(self, timeout_s) -> ActionOutcome:
        assert self.outcome is not None
        return self.outcome

    def cancel(self, wait: bool = False) -> None:
        self.cancelled = self.outcome is None

    def release(self) -> None:
        pass


def _move(goal: FakeGoal, ctx: AbortContext) -> tuple[IdlePoseMove, MagicMock]:
    session = MagicMock()
    return IdlePoseMove(session, goal, ctx, MagicMock()), session


def test_failed_move_aborts_the_context() -> None:
    goal, ctx = FakeGoal(), AbortContext()
    move, _ = _move(goal, ctx)
    goal.finish(GoalStatus.STATUS_ABORTED, False, 'unreachable')
    assert isinstance(ctx.error, SystemFailureError)
    assert 'unreachable' in str(ctx.error)
    with pytest.raises(SystemFailureError):
        move.wait()


def test_successful_move_leaves_the_context_alone() -> None:
    goal, ctx = FakeGoal(), AbortContext()
    move, _ = _move(goal, ctx)
    goal.finish(GoalStatus.STATUS_SUCCEEDED, True)
    move.wait()
    assert ctx.error is None


def test_close_cancels_a_running_move_without_aborting_and_releases_the_session() -> None:
    goal, ctx = FakeGoal(), AbortContext()
    move, session = _move(goal, ctx)
    move.close()
    goal.finish(GoalStatus.STATUS_CANCELED, False)
    assert goal.cancelled
    assert ctx.error is None
    session.close.assert_called_once()
