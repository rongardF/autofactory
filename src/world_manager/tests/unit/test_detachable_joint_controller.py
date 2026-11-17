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
"""Unit tests for DetachableJointController."""

from pytest import raises

from world_manager.enums import JointStateEnum
from world_manager.exceptions import (
    ConfirmationTimeoutError,
    GzRequestError,
    ModelNotFoundError,
    ModelValidationError,
)
from conftest import make_sdf, spec


def _spawn(stack, *models):
    stack.service.spawn(list(models), 'owner')


def _publishes(stack, topic):
    return [c for c in stack.adapter.calls if c == ('publish', topic)]


def test_detach_waits_for_state_event(stack):
    """A detach request completes once the state topic reports 'detached'."""
    _spawn(stack, spec('tool'))
    stack.controller.set_attached('tool', 'mount', False, 1.0)
    assert stack.registry.joint_state('tool', 'mount').state == JointStateEnum.DETACHED


def test_request_is_republished_until_confirmed(stack):
    """The plugin may not be ready for the first request, so it is published again."""
    _spawn(stack, spec('tool'))
    stack.adapter.swallow_publishes = 2
    stack.controller.set_attached('tool', 'mount', False, 1.0)
    assert len(_publishes(stack, '/mount/tool/detach')) == 3


def test_detach_of_detached_joint_returns_without_publishing(stack):
    """Detaching an already detached joint is a no-op."""
    _spawn(stack, spec('tool'))
    stack.controller.set_attached('tool', 'mount', False, 1.0)
    stack.adapter.calls.clear()
    stack.controller.set_attached('tool', 'mount', False, 1.0)
    assert stack.adapter.calls == []


def test_unconfirmed_request_times_out(stack):
    """A request that is never confirmed raises ConfirmationTimeoutError."""
    _spawn(stack, spec('tool'))
    stack.adapter.ignore_topics.add('/mount/tool/detach')
    with raises(ConfirmationTimeoutError, match='mount'):
        stack.controller.set_attached('tool', 'mount', False, 0.2)


def test_unknown_joint_is_rejected(stack):
    """Addressing a joint that does not exist raises ModelNotFoundError."""
    _spawn(stack, spec('tool'))
    with raises(ModelNotFoundError):
        stack.controller.set_attached('tool', 'nope', False, 0.1)


def test_uncontrollable_joint_is_rejected_naming_the_topic(stack):
    """A joint with a shared topic cannot be controlled; the error names the topic."""
    sdf = make_sdf('pallet', [('conveyor', 'slot')])
    _spawn(stack, spec('pallet', sdf), spec('pallet2', make_sdf('pallet2', [('conveyor', 'slot')])))
    with raises(ModelValidationError, match='/model/conveyor/detachable_joint/state'):
        stack.controller.set_attached('pallet', 'slot', False, 0.1)


def test_watch_failure_raises(stack):
    """A subscription that cannot be created aborts with GzRequestError."""
    stack.adapter.subscribe_ok = False
    with raises(GzRequestError, match='/a/state'):
        stack.controller.watch(['/a/state'])


def test_unwatch_keeps_topics_still_in_use(stack):
    """Subscriptions needed by a remaining joint are kept."""
    _spawn(stack, spec('tool'))
    stack.controller.unwatch(['/mount/tool/state', '/unused/state'])
    assert ('unsubscribe', '/mount/tool/state') not in stack.adapter.calls
    assert ('unsubscribe', '/unused/state') in stack.adapter.calls
