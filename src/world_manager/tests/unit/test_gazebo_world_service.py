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
"""Unit tests for GazeboWorldService against a scripted fake gz."""

from threading import Event, Thread
from time import sleep

from pytest import raises

from world_manager.enums import JointStateEnum
from world_manager.exceptions import (
    ActivationError,
    ConfirmationTimeoutError,
    GzRequestError,
    ModelNotFoundError,
    ModelValidationError,
    OperationTimeoutError,
)
from world_manager.models import PoseDTO
from conftest import make_sdf, spec, tool_sdf


def states(stack, model):
    return {info.link_name: info.state for _, info in stack.registry.joints(model)}


# ---------- spawn ----------

def test_spawn_registers_models_and_returns_ids(stack):
    """A successful spawn returns the ids in order and registers owner and joints."""
    ids = stack.service.spawn([spec('a'), spec('b')], 'endtools')
    assert ids == ['a', 'b']
    listed = stack.service.list_models('endtools')
    assert [m.model_id for m in listed] == ['a', 'b']
    assert all(m.owner == 'endtools' for m in listed)


def test_spawn_creates_in_list_order(stack):
    """Models are created in the order given so a child can precede its holder."""
    stack.service.spawn([spec('b'), spec('a')], 'o')
    assert stack.adapter.names('create') == ['b', 'a']


def test_spawn_subscribes_before_creating(stack):
    """State topics are subscribed before create so the first event is not lost."""
    stack.service.spawn([spec('a')], 'o')
    calls = stack.adapter.calls
    assert calls.index(('subscribe', '/mount/a/state')) < calls.index(('create', 'a'))


def test_spawn_failure_rolls_back_in_reverse_order(stack):
    """If the third model fails, the first two are removed, last first, and nothing is registered."""
    stack.adapter.fail_create.add('c')
    with raises(GzRequestError, match="'c'"):
        stack.service.spawn([spec('a'), spec('b'), spec('c')], 'o')
    assert stack.adapter.names('remove') == ['b', 'a']
    assert stack.service.list_models() == []
    assert stack.adapter.world == {'station'}


def test_rollback_detaches_joints_before_removing(stack):
    """Rolled-back models have their joints detached first."""
    stack.adapter.fail_create.add('b')
    with raises(GzRequestError):
        stack.service.spawn([spec('a'), spec('b')], 'o')
    calls = stack.adapter.calls
    assert calls.index(('publish', '/mount/a/detach')) < calls.index(('remove', 'a'))


def test_rolled_back_spawn_does_not_notify(stack):
    """A rolled-back spawn leaves the committed list unchanged and signals nothing."""
    stack.adapter.fail_create.add('b')
    with raises(GzRequestError):
        stack.service.spawn([spec('a'), spec('b')], 'o')
    assert not stack.dirty.is_set()


def test_successful_spawn_notifies_once_after_commit(stack):
    """The list is marked dirty by the commit of the batch."""
    stack.service.spawn([spec('a')], 'o')
    assert stack.dirty.is_set()


def test_spawn_rejects_duplicate_ids_in_call(stack):
    """Ids must be unique within a call; nothing is created."""
    with raises(ModelValidationError, match='unique'):
        stack.service.spawn([spec('a'), spec('a')], 'o')
    assert stack.adapter.names('create') == []


def test_spawn_rejects_already_registered_id(stack):
    """Spawning a registered id again is rejected before any gz call."""
    stack.service.spawn([spec('a')], 'o')
    with raises(ModelValidationError, match='already registered'):
        stack.service.spawn([spec('a')], 'o')


def test_spawn_rejects_id_that_differs_from_sdf_name(stack):
    """model_id must equal the SDF model name."""
    with raises(ModelValidationError, match='does not match'):
        stack.service.spawn([spec('a', tool_sdf('other'))], 'o')


def test_spawn_rejects_empty_request(stack):
    """An empty batch is a request error."""
    with raises(ModelValidationError):
        stack.service.spawn([], 'o')


def test_spawn_rejects_unknown_attach_link(stack):
    """attach_link must name the child_link of one joint."""
    with raises(ModelValidationError, match='attach_link|child_link'):
        stack.service.spawn([spec('a', attach_link='nope')], 'o')


def test_spawn_rejects_attach_link_when_joints_not_controllable(stack):
    """attach_link needs every joint of the model to be controllable."""
    sdf = make_sdf('t', [('station', 'a'), ('station', 'b')])  # both share the default state topic
    with raises(ModelValidationError, match='not controllable'):
        stack.service.spawn([spec('t', sdf, attach_link='a')], 'o')
    assert stack.adapter.names('create') == []


def test_spawn_rejects_unresolvable_tf_frame(stack):
    """A pose whose frame TF cannot resolve is rejected before creation."""
    with raises(ModelValidationError, match='cannot resolve'):
        stack.service.spawn([spec('a', frame_id='nowhere')], 'o')
    assert stack.adapter.names('create') == []


def test_spawn_with_attach_link_keeps_that_joint_and_detaches_others(stack):
    """Only the joint holding attach_link stays attached after spawn."""
    stack.service.spawn([spec('t', attach_link='mount')], 'o')
    assert states(stack, 't') == {'mount': JointStateEnum.ATTACHED, 'rack': JointStateEnum.DETACHED}


def test_spawn_detach_failure_rolls_back(stack):
    """If the detach required by attach_link is not confirmed, the spawn fails and is rolled back."""
    stack.adapter.ignore_topics.add('/rack/t/detach')
    with raises(ConfirmationTimeoutError):
        stack.service.spawn([spec('t', attach_link='mount')], 'o')
    assert 't' not in stack.adapter.world
    assert stack.service.list_models() == []


def test_spawn_without_initial_event_succeeds_with_unconfirmed_state(stack):
    """A missing initial 'attached' event is not a failure; the state is assumed and flagged."""
    stack.adapter.send_initial_event = False
    stack.service.spawn([spec('t')], 'o')
    joints = stack.service.list_models()[0].joints
    assert all(j.state == JointStateEnum.ATTACHED and j.state_confirmed is False for j in joints)


def test_initial_event_gives_confirmed_state(stack):
    """An observed initial event confirms the state."""
    stack.service.spawn([spec('t')], 'o')
    assert all(j.state_confirmed for j in stack.service.list_models()[0].joints)


def test_event_before_create_reply_is_preserved(stack):
    """An event delivered before create returns is kept, not overwritten by the assumption."""
    stack.adapter.send_initial_event = False
    stack.adapter.on_create = lambda model: stack.adapter.emit('/mount/t/state', 'detached')
    stack.service.spawn([spec('t', tool_sdf('t', ('mount',)))], 'o')
    joint = stack.service.list_models()[0].joints[0]
    assert (joint.state, joint.state_confirmed) == (JointStateEnum.DETACHED, True)


def test_later_event_confirms_assumed_state(stack):
    """After an assumed spawn, a later observed event confirms the state."""
    stack.adapter.send_initial_event = False
    stack.service.spawn([spec('t', tool_sdf('t', ('mount',)))], 'o')
    stack.adapter.emit('/mount/t/state', 'attached')
    assert stack.service.list_models()[0].joints[0].state_confirmed is True


def test_ambiguous_event_is_not_attributed(stack):
    """An event on a shared state topic is attributed to no joint."""
    stack.adapter.send_initial_event = False
    a = make_sdf('p1', [('conveyor', 'slot')])
    b = make_sdf('p2', [('conveyor', 'slot')])
    stack.service.spawn([spec('p1', a), spec('p2', b)], 'o')
    stack.adapter.emit('/model/conveyor/detachable_joint/state', 'detached')
    assert all(
        j.state == JointStateEnum.ATTACHED and not j.state_confirmed
        for m in stack.service.list_models() for j in m.joints)


def test_unmanaged_model_with_same_id_is_rejected_before_creating(stack):
    """A world model of the same name that the manager does not own is not adopted and nothing is created."""
    stack.adapter.world.add('a')
    with raises(ModelValidationError, match='not managed'):
        stack.service.spawn([spec('a')], 'o')
    assert stack.adapter.names('create') == []
    assert stack.adapter.world == {'station', 'a'}


def test_create_without_reply_that_still_creates_the_model_is_rolled_back(stack):
    """A create that times out but takes effect is removed again, so no orphan stays in the world."""
    stack.adapter.late_create.add('b')
    with raises(GzRequestError, match='rolling back'):
        stack.service.spawn([spec('a'), spec('b')], 'o')
    assert stack.adapter.names('remove') == ['b', 'a']
    assert stack.adapter.world == {'station'}
    assert stack.service.list_models() == []


def test_spawn_deadline_expiry_rolls_back_and_names_step(stack):
    """When the deadline expires between creates, earlier models are rolled back and the step is named."""
    original = stack.adapter.create_model

    def slow_create(world, model_id, sdf, pose, timeout_sec):
        result = original(world, model_id, sdf, pose, timeout_sec)
        sleep(0.05)
        return result

    stack.adapter.create_model = slow_create
    stack.service._config = stack.config.model_copy(update={'operation_timeout_sec': 0.02})
    with raises(OperationTimeoutError, match="creating 'b'"):
        stack.service.spawn([spec('a'), spec('b')], 'o')
    assert stack.adapter.names('remove') == ['a']
    assert stack.service.list_models() == []


# ---------- delete ----------

def test_delete_by_ids_removes_and_unregisters(stack):
    """Listed ids are removed from the world and the registry."""
    stack.service.spawn([spec('a'), spec('b')], 'o')
    result, error = stack.service.delete(['a'])
    assert (result.deleted_ids, error) == (('a',), '')
    assert [m.model_id for m in stack.service.list_models()] == ['b']


def test_delete_detaches_joints_first(stack):
    """Joints are detached before the model is removed."""
    stack.service.spawn([spec('a')], 'o')
    stack.adapter.calls.clear()
    stack.service.delete(['a'])
    calls = stack.adapter.calls
    assert calls.index(('publish', '/mount/a/detach')) < calls.index(('remove', 'a'))


def test_delete_by_owner_removes_all_of_that_owner(stack):
    """An empty id list deletes everything registered to the owner."""
    stack.service.spawn([spec('a')], 'x')
    stack.service.spawn([spec('b')], 'y')
    result, _ = stack.service.delete(owner='x')
    assert result.deleted_ids == ('a',)
    assert [m.model_id for m in stack.service.list_models()] == ['b']


def test_delete_with_nothing_given_is_rejected(stack):
    """Both inputs empty is rejected."""
    with raises(ModelValidationError):
        stack.service.delete()


def test_delete_reports_unknown_ids_and_continues(stack):
    """Unknown ids are returned and do not stop the rest."""
    stack.service.spawn([spec('a')], 'o')
    result, error = stack.service.delete(['ghost', 'a'])
    assert (result.deleted_ids, result.unknown_ids, error) == (('a',), ('ghost',), '')


def test_delete_with_owner_guard_rejects_foreign_ids(stack):
    """With both given, owner guards: ids of another owner reject the whole call."""
    stack.service.spawn([spec('a')], 'x')
    with raises(ModelValidationError, match='not owned'):
        stack.service.delete(['a'], 'y')
    assert stack.adapter.names('remove') == []


def _holder_setup(stack):
    """'station2' is held by 'tool2' through an attached joint."""
    holder = make_sdf('tool2', [('station2', 'link', '/l/a', '/l/d', '/l/s')])
    stack.service.spawn([spec('station2', make_sdf('station2')), spec('tool2', holder)], 'o')


def test_delete_rejected_when_outside_holder_exists(stack):
    """A model with an attached holder outside the call cannot be deleted; nothing is deleted."""
    _holder_setup(stack)
    with raises(ModelValidationError, match="'tool2'"):
        stack.service.delete(['station2'])
    assert stack.adapter.names('remove') == []


def test_delete_accepted_dependents_first_when_both_in_call(stack):
    """When holder and held are both targeted, the holder goes first."""
    _holder_setup(stack)
    stack.service.delete(['station2', 'tool2'])
    assert stack.adapter.names('remove') == ['tool2', 'station2']


def test_delete_after_holder_detached_is_allowed(stack):
    """Once the holder's joint is detached the held model may be deleted alone."""
    _holder_setup(stack)
    stack.service.set_attachment('tool2', '')
    result, error = stack.service.delete(['station2'])
    assert (result.deleted_ids, error) == (('station2',), '')


def _putty(model: str, held: str = 'station2'):
    """A model that holds ``held`` and is flagged to be deleted with it."""
    sdf = make_sdf(model, [(held, 'link', f'/p/{model}/a', f'/p/{model}/d', f'/p/{model}/s')])
    return spec(model, sdf, delete_with_child=True)


def test_delete_cascades_to_flagged_holders_of_other_owner(stack):
    """Flagged holders are detached and removed before the held model, whatever their owner."""
    stack.service.spawn([spec('station2', make_sdf('station2'))], 'workplace')
    stack.service.spawn([_putty('p1'), _putty('p2')], 'endtool')
    result, error = stack.service.delete(owner='workplace')
    assert (set(result.deleted_ids), error) == ({'station2', 'p1', 'p2'}, '')
    assert stack.adapter.names('remove')[-1] == 'station2'
    calls = stack.adapter.calls
    assert calls.index(('publish', '/p/p1/d')) < calls.index(('remove', 'p1'))
    assert stack.service.list_models() == []


def test_delete_cascade_is_transitive(stack):
    """A flagged holder of a flagged holder is deleted too, outermost first."""
    stack.service.spawn([spec('station2', make_sdf('station2'))], 'o')
    stack.service.spawn([_putty('p1')], 'o')
    stack.service.spawn([_putty('p2', held='p1')], 'o')
    result, error = stack.service.delete(['station2'])
    assert error == ''
    assert stack.adapter.names('remove') == ['p2', 'p1', 'station2']


def test_delete_cascade_ignores_unflagged_holders(stack):
    """An unflagged holder still blocks the delete."""
    stack.service.spawn([spec('station2', make_sdf('station2'))], 'o')
    stack.service.spawn([_putty('p1')], 'o')
    holder = make_sdf('tool2', [('station2', 'link', '/l/a', '/l/d', '/l/s')])
    stack.service.spawn([spec('tool2', holder)], 'o')
    with raises(ModelValidationError, match="'tool2'"):
        stack.service.delete(['station2'])
    assert stack.adapter.names('remove') == []


def test_delete_keeps_held_model_when_cascaded_holder_stays_attached(stack):
    """If a cascaded holder cannot be detached and removed, the model it holds stays too."""
    stack.service.spawn([spec('station2', make_sdf('station2'))], 'o')
    stack.service.spawn([_putty('p1'), _putty('p2')], 'o')
    stack.adapter.fail_remove.add('p1')
    stack.adapter.ignore_topics.add('/p/p1/d')
    result, error = stack.service.delete(['station2'])
    assert result.deleted_ids == ('p2',)
    assert 'p1' in error and 'station2' in error
    assert 'station2' not in stack.adapter.names('remove')


def test_delete_proceeds_when_cascaded_holder_was_detached_but_not_removed(stack):
    """A holder whose joint is detached no longer blocks the held model, even if its removal failed."""
    stack.service.spawn([spec('station2', make_sdf('station2'))], 'o')
    stack.service.spawn([_putty('p1')], 'o')
    stack.adapter.fail_remove.add('p1')
    result, error = stack.service.delete(['station2'])
    assert result.deleted_ids == ('station2',)
    assert 'p1' in error


def test_cascaded_deletions_do_not_use_up_the_deadline(stack):
    """The requested model is still deleted when the cascaded holders took longer than the timeout."""
    stack.service.spawn([spec('station2', make_sdf('station2'))], 'o')
    stack.service.spawn([_putty('p1')], 'o')
    original = stack.service._delete_one

    def slow(model_id):
        sleep(0.2 if model_id == 'p1' else 0)
        return original(model_id)

    stack.service._delete_one = slow
    stack.service._config = stack.service._config.model_copy(update={'operation_timeout_sec': 0.1})
    result, error = stack.service.delete(['station2'])
    assert (set(result.deleted_ids), error) == ({'p1', 'station2'}, '')


def test_delete_failure_of_known_model_is_reported(stack):
    """A failed remove of a registered model marks the call failed but deletes the others."""
    stack.service.spawn([spec('a'), spec('b')], 'o')
    stack.adapter.fail_remove.add('a')
    result, error = stack.service.delete(['a', 'b'])
    assert result.deleted_ids == ('b',)
    assert 'a' in error


def test_delete_of_externally_removed_model_counts_as_deleted(stack):
    """If the world no longer has the model, the delete goal is met and the registry is cleaned."""
    stack.service.spawn([spec('a')], 'o')
    stack.adapter.world.discard('a')
    result, error = stack.service.delete(['a'])
    assert (result.deleted_ids, error) == (('a',), '')
    assert stack.service.list_models() == []


# ---------- set_pose ----------

def test_set_pose_calls_gz_with_world_pose(stack):
    """The resolved world pose is sent to gz."""
    stack.service.spawn([spec('a')], 'o')
    stack.service.set_pose('a', PoseDTO(x=1.0))
    assert stack.adapter.calls[-1][:2] == ('set_pose', 'a')
    assert stack.adapter.calls[-1][2].x == 1.0


def test_set_pose_unknown_model_fails_without_gz_call(stack):
    """Unmanaged ids fail up front instead of waiting for the 5 s gz timeout."""
    with raises(ModelNotFoundError):
        stack.service.set_pose('ghost', PoseDTO())
    assert stack.adapter.names('set_pose') == []


def test_set_pose_reconciles_and_fails_fast_for_externally_removed_model(stack):
    """A model removed behind the manager's back fails with 'no longer exists' and leaves the registry."""
    stack.service.spawn([spec('a')], 'o')
    stack.adapter.world.discard('a')
    with raises(ModelNotFoundError, match='no longer exists'):
        stack.service.set_pose('a', PoseDTO())
    assert stack.adapter.names('set_pose') == []
    assert stack.service.list_models() == []


def test_set_pose_gz_refusal_raises(stack):
    """A refused set_pose is reported."""
    stack.service.spawn([spec('a')], 'o')
    stack.adapter.fail_set_pose = True
    with raises(GzRequestError):
        stack.service.set_pose('a', PoseDTO())


def test_set_pose_fails_when_scene_info_does_not_answer(stack):
    """If the world cannot be read, the call fails instead of guessing."""
    stack.service.spawn([spec('a')], 'o')
    stack.adapter.scene_answers = False
    with raises(GzRequestError, match='scene/info'):
        stack.service.set_pose('a', PoseDTO())


# ---------- set_attachment ----------

def test_set_attachment_attaches_target_then_detaches_others(stack):
    """The target link is attached before others are detached so the model is never unheld."""
    stack.service.spawn([spec('t', attach_link='mount')], 'o')
    stack.adapter.calls.clear()
    stack.service.set_attachment('t', 'rack')
    publishes = [c[1] for c in stack.adapter.calls if c[0] == 'publish']
    assert publishes.index('/rack/t/attach') < publishes.index('/mount/t/detach')
    assert states(stack, 't') == {'mount': JointStateEnum.DETACHED, 'rack': JointStateEnum.ATTACHED}


def test_set_attachment_empty_detaches_all(stack):
    """An empty link name releases the model."""
    stack.service.spawn([spec('t')], 'o')
    stack.service.set_attachment('t', '')
    assert set(states(stack, 't').values()) == {JointStateEnum.DETACHED}


def test_set_attachment_is_idempotent(stack):
    """Repeating a request that matches the tracked state issues no gz traffic."""
    stack.service.spawn([spec('t', attach_link='mount')], 'o')
    stack.adapter.calls.clear()
    stack.service.set_attachment('t', 'mount')
    assert stack.adapter.calls == []


def test_set_attachment_unknown_model_or_link(stack):
    """Unknown models and links are rejected."""
    stack.service.spawn([spec('t')], 'o')
    with raises(ModelNotFoundError):
        stack.service.set_attachment('ghost', 'mount')
    with raises(ModelValidationError):
        stack.service.set_attachment('t', 'nope')


def test_set_attachment_timeout_when_not_confirmed(stack):
    """A change that is never confirmed times out; the timeout is not recorded as a state."""
    stack.service.spawn([spec('t', attach_link='mount')], 'o')
    stack.adapter.ignore_topics.add('/rack/t/attach')
    with raises(ConfirmationTimeoutError):
        stack.service.set_attachment('t', 'rack')
    assert states(stack, 't')['rack'] == JointStateEnum.DETACHED


def test_set_attachment_on_uncontrollable_joint_names_topic(stack):
    """A joint with a shared topic fails with a clear error."""
    stack.service.spawn([spec('p1', make_sdf('p1', [('conveyor', 'slot')])),
                         spec('p2', make_sdf('p2', [('conveyor', 'slot')]))], 'o')
    stack.adapter.emit('/model/conveyor/detachable_joint/state', 'attached')
    with raises(ModelValidationError, match='not controllable'):
        stack.service.set_attachment('p1', '')


# ---------- activation / reconcile ----------

def test_activate_succeeds_when_scene_info_answers(stack):
    """Activation passes if the world answers scene/info."""
    stack.service.activate()
    assert ('scene_info',) in stack.adapter.calls


def test_activate_fails_when_scene_info_does_not_answer(stack):
    """Activation fails with a clear error if the world lacks the SceneBroadcaster."""
    stack.adapter.scene_answers = False
    with raises(ActivationError, match='scene/info'):
        stack.service.activate()


def test_registry_survives_reactivation(stack):
    """Re-activation keeps registered models that still exist."""
    stack.service.spawn([spec('a')], 'o')
    stack.service.activate()
    assert [m.model_id for m in stack.service.list_models()] == ['a']


def test_activation_drops_models_gone_from_the_world(stack):
    """Reconcile on activation removes entries whose model vanished and marks the list dirty."""
    stack.service.spawn([spec('a')], 'o')
    stack.dirty.clear()
    stack.adapter.world.discard('a')
    stack.service.activate()
    assert stack.service.list_models() == []
    assert stack.dirty.is_set()


def test_failed_remove_reconciles_registry(stack):
    """A refused remove triggers a scene/info read."""
    stack.service.spawn([spec('a')], 'o')
    stack.adapter.fail_remove.add('a')
    stack.adapter.calls.clear()
    stack.service.delete(['a'])
    assert ('scene_info',) in stack.adapter.calls


def test_failed_create_reconciles_registry(stack):
    """A refused create triggers a scene/info read."""
    stack.adapter.fail_create.add('a')
    with raises(GzRequestError):
        stack.service.spawn([spec('a')], 'o')
    assert ('scene_info',) in stack.adapter.calls


# ---------- listing ----------

def test_list_models_filters_by_owner(stack):
    """The owner filter narrows the list; empty means all."""
    stack.service.spawn([spec('a')], 'x')
    stack.service.spawn([spec('b')], 'y')
    assert [m.model_id for m in stack.service.list_models('y')] == ['b']
    assert len(stack.service.list_models()) == 2


def test_list_models_is_not_blocked_by_running_spawn(stack):
    """ListModels uses only the registry lock, so it answers while a spawn holds the operation lock."""
    in_create = Event()
    release = Event()
    original = stack.adapter.create_model

    def blocking_create(*args):
        in_create.set()
        release.wait(2.0)
        return original(*args)

    stack.adapter.create_model = blocking_create
    worker = Thread(target=lambda: stack.service.spawn([spec('a')], 'o'))
    worker.start()
    try:
        assert in_create.wait(2.0)
        assert stack.service.list_models() == []
    finally:
        release.set()
        worker.join(2.0)
    assert [m.model_id for m in stack.service.list_models()] == ['a']
