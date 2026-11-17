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
"""Unit tests for ModelRegistry."""

from pytest import raises

from world_manager.enums import JointStateEnum
from world_manager.exceptions import ModelValidationError
from world_manager.models import DetachableJointDTO
from world_manager.services.model_registry import ModelRegistry


def joint(link: str, attach: str = None, state: str = None, child: str = 'station') -> DetachableJointDTO:
    return DetachableJointDTO(
        link_name=link, child_model=child,
        attach_topic=attach or f'/{link}/attach', detach_topic=f'/{link}/detach',
        state_topic=state or f'/{link}/state')


def test_pending_models_are_invisible_until_committed():
    """Pending entries are not part of the snapshot and do not notify."""
    changes = []
    registry = ModelRegistry(on_change=lambda: changes.append(1))
    registry.add_pending('a', 'owner', [joint('l1')])
    assert registry.snapshot() == []
    assert changes == []
    registry.commit(['a'])
    assert [m.model_id for m in registry.snapshot()] == ['a']
    assert changes == [1]


def test_discarded_pending_models_leave_no_trace():
    """A rolled-back batch never notifies and never becomes visible."""
    changes = []
    registry = ModelRegistry(on_change=lambda: changes.append(1))
    registry.add_pending('a', 'owner', [joint('l1')])
    registry.discard(['a'])
    assert not registry.contains('a')
    assert changes == []


def test_duplicate_id_is_rejected():
    """The same id cannot be registered twice."""
    registry = ModelRegistry()
    registry.add_pending('a', 'o', [])
    with raises(ModelValidationError):
        registry.add_pending('a', 'o', [])


def test_unique_topics_are_controllable():
    """Joints with unique topics are controllable."""
    registry = ModelRegistry()
    registry.add_pending('a', 'o', [joint('l1'), joint('l2')])
    assert all(info.controllable for _, info in registry.joints('a'))


def test_shared_state_topic_makes_joints_uncontrollable():
    """Two joints reporting on one state topic cannot be told apart."""
    registry = ModelRegistry()
    registry.add_pending('a', 'o', [joint('l1', state='/shared'), joint('l2', state='/shared')])
    assert not any(info.controllable for _, info in registry.joints('a'))
    assert registry.collision_topic('a', 'l1') == '/shared'


def test_pending_batch_does_not_flip_committed_joints():
    """A colliding pending model must not change what a committed model reports."""
    registry = ModelRegistry()
    registry.add_pending('a', 'o', [joint('l1', state='/shared')])
    registry.commit(['a'])
    registry.add_pending('b', 'o', [joint('l2', state='/shared')])
    assert registry.snapshot()[0].joints[0].controllable is True
    assert registry.joints('b')[0][1].controllable is False
    registry.commit(['b'])
    assert not any(j.controllable for m in registry.snapshot() for j in m.joints)


def test_event_is_attributed_and_confirmed():
    """A state event updates the joint and marks the state as observed."""
    registry = ModelRegistry()
    registry.add_pending('a', 'o', [joint('l1')])
    registry.commit(['a'])
    assert registry.apply_state_event('/l1/state', 'detached') is True
    info = registry.joint_state('a', 'l1')
    assert (info.state, info.state_confirmed) == (JointStateEnum.DETACHED, True)


def test_event_on_shared_topic_is_not_attributed():
    """Ambiguous events are dropped instead of being given to a random joint."""
    registry = ModelRegistry()
    registry.add_pending('a', 'o', [joint('l1', state='/shared')])
    registry.add_pending('b', 'o', [joint('l2', state='/shared')])
    assert registry.apply_state_event('/shared', 'attached') is False
    assert registry.joint_state('a', 'l1').state == JointStateEnum.UNKNOWN


def test_unknown_state_string_is_ignored():
    """Garbage on a state topic changes nothing."""
    registry = ModelRegistry()
    registry.add_pending('a', 'o', [joint('l1')])
    assert registry.apply_state_event('/l1/state', 'banana') is False


def test_repeated_identical_event_does_not_notify():
    """Only changes are published."""
    changes = []
    registry = ModelRegistry(on_change=lambda: changes.append(1))
    registry.add_pending('a', 'o', [joint('l1')])
    registry.commit(['a'])
    registry.apply_state_event('/l1/state', 'attached')
    registry.apply_state_event('/l1/state', 'attached')
    assert len(changes) == 2  # commit + first event


def test_assumed_attached_is_unconfirmed():
    """Without an event the state is assumed, flagged unconfirmed."""
    registry = ModelRegistry()
    registry.add_pending('a', 'o', [joint('l1')])
    assert registry.assume_attached('a') == ['l1']
    info = registry.joint_state('a', 'l1')
    assert (info.state, info.state_confirmed) == (JointStateEnum.ATTACHED, False)


def test_event_before_assumption_is_preserved():
    """An event that arrived before the create reply wins over the assumption."""
    registry = ModelRegistry()
    registry.add_pending('a', 'o', [joint('l1')])
    registry.apply_state_event('/l1/state', 'attached')
    assert registry.assume_attached('a') == []
    assert registry.joint_state('a', 'l1').state_confirmed is True


def test_dependents_ignore_detached_joints():
    """A model is depended on only through joints that are not detached."""
    registry = ModelRegistry()
    registry.add_pending('station', 'o', [])
    registry.add_pending('tool', 'o', [joint('l1', child='station')])
    registry.commit(['station', 'tool'])
    registry.assume_attached('tool')
    assert registry.dependents_of('station') == ['tool']
    registry.apply_state_event('/l1/state', 'detached')
    assert registry.dependents_of('station') == []


def test_drop_missing_removes_and_notifies():
    """Entries whose model is gone are dropped and the list is republished."""
    changes = []
    registry = ModelRegistry(on_change=lambda: changes.append(1))
    registry.add_pending('a', 'o', [])
    registry.add_pending('b', 'o', [])
    registry.commit(['a', 'b'])
    assert registry.drop_missing(['a']) == ['b']
    assert registry.committed_ids() == ['a']
    assert len(changes) == 2
