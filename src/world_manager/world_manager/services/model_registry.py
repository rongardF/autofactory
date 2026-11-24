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
"""Thread-safe registry of the models spawned by the manager."""

from collections import Counter
from threading import RLock
from typing import Callable, Iterable, Sequence

from world_manager.enums import JointStateEnum
from world_manager.exceptions import ModelValidationError
from world_manager.models import DetachableJointDTO, JointInfoDTO, ModelInfoDTO


class _JointEntry:
    """Mutable bookkeeping of one joint; only touched under the registry lock."""

    __slots__ = ('config', 'state', 'confirmed', 'controllable')

    def __init__(self, config: DetachableJointDTO) -> None:
        self.config = config
        self.state = JointStateEnum.UNKNOWN
        self.confirmed = False
        self.controllable = True

    def info(self) -> JointInfoDTO:
        return JointInfoDTO(
            link_name=self.config.link_name,
            child_model=self.config.child_model,
            state=self.state,
            state_confirmed=self.confirmed,
            controllable=self.controllable,
        )


class _ModelEntry:
    __slots__ = ('model_id', 'owner', 'joints', 'committed', 'delete_with_child')

    def __init__(
        self, model_id: str, owner: str, joints: Sequence[DetachableJointDTO], delete_with_child: bool,
    ) -> None:
        self.model_id = model_id
        self.owner = owner
        self.delete_with_child = delete_with_child
        self.joints = {joint.link_name: _JointEntry(joint) for joint in joints}
        self.committed = False


class ModelRegistry:
    """Models and joint states known to the manager.

    Models are first added as *pending* so that state events arriving during a spawn batch are stored.
    Only committed models are visible to ``snapshot`` and trigger ``on_change``; a discarded batch
    therefore leaves no trace. The lock is short-lived and never held across a gz request or a wait.
    """

    def __init__(self, on_change: Callable[[], None] | None = None) -> None:
        """``on_change`` is called (outside the lock) when the committed list may have changed."""
        self._lock = RLock()
        self._models: dict[str, _ModelEntry] = {}
        self._on_change = on_change

    def _notify(self) -> None:
        if self._on_change is not None:
            self._on_change()

    def _joints(self, committed_only: bool) -> list[_JointEntry]:
        return [
            joint for entry in self._models.values() if entry.committed or not committed_only
            for joint in entry.joints.values()
        ]

    def _recompute_controllable(self) -> bool:
        """Recompute the controllable flags; return True if a committed joint changed.

        Committed joints are judged against committed joints only, so a pending batch can never flip
        them; pending joints are judged against committed and pending joints.
        """
        committed = [entry for entry in self._models.values() if entry.committed]
        pending = [entry for entry in self._models.values() if not entry.committed]
        changed = False
        for group, scope in ((committed, committed), (pending, committed + pending)):
            joints = [joint for entry in scope for joint in entry.joints.values()]
            attach = Counter(j.config.attach_topic for j in joints)
            detach = Counter(j.config.detach_topic for j in joints)
            state = Counter(j.config.state_topic for j in joints)
            for entry in group:
                for joint in entry.joints.values():
                    value = (
                        attach[joint.config.attach_topic] == 1
                        and detach[joint.config.detach_topic] == 1
                        and state[joint.config.state_topic] == 1
                    )
                    if value != joint.controllable:
                        joint.controllable = value
                        changed = changed or entry.committed
        return changed

    def collision_topic(self, model_id: str, link_name: str) -> str:
        """Name a topic of the joint that is shared with another joint, or '' if it is unique."""
        with self._lock:
            entry = self._models.get(model_id)
            if entry is None or link_name not in entry.joints:
                return ''
            joint = entry.joints[link_name].config
            others = [
                other.config for other in self._joints(committed_only=entry.committed)
                if other.config is not joint
            ]
            for topic, attr in (
                (joint.attach_topic, 'attach_topic'),
                (joint.detach_topic, 'detach_topic'),
                (joint.state_topic, 'state_topic'),
            ):
                if any(getattr(other, attr) == topic for other in others):
                    return topic
            return ''

    def add_pending(
        self, model_id: str, owner: str, joints: Sequence[DetachableJointDTO], delete_with_child: bool = False,
    ) -> None:
        """Register a model that is about to be created."""
        with self._lock:
            if model_id in self._models:
                raise ModelValidationError(f"model '{model_id}' is already registered")
            self._models[model_id] = _ModelEntry(model_id, owner, joints, delete_with_child)
            self._recompute_controllable()

    def commit(self, model_ids: Iterable[str]) -> None:
        """Make pending models visible; publishes once for the whole batch."""
        with self._lock:
            for model_id in model_ids:
                self._models[model_id].committed = True
            self._recompute_controllable()
        self._notify()

    def discard(self, model_ids: Iterable[str]) -> None:
        """Forget pending models (rolled-back spawn); nothing is published."""
        with self._lock:
            for model_id in model_ids:
                entry = self._models.get(model_id)
                if entry is not None and not entry.committed:
                    del self._models[model_id]
            self._recompute_controllable()

    def remove(self, model_id: str) -> None:
        """Forget a deleted model."""
        with self._lock:
            entry = self._models.pop(model_id, None)
            self._recompute_controllable()
        if entry is not None and entry.committed:
            self._notify()

    def drop_missing(self, present: Iterable[str]) -> list[str]:
        """Drop committed models that are no longer in the world; returns the dropped ids."""
        present_set = set(present)
        with self._lock:
            dropped = [
                model_id for model_id, entry in self._models.items()
                if entry.committed and model_id not in present_set
            ]
            for model_id in dropped:
                del self._models[model_id]
            if dropped:
                self._recompute_controllable()
        if dropped:
            self._notify()
        return dropped

    def is_registered(self, model_id: str) -> bool:
        """True for committed models."""
        with self._lock:
            entry = self._models.get(model_id)
            return entry is not None and entry.committed

    def contains(self, model_id: str) -> bool:
        """True for committed and pending models."""
        with self._lock:
            return model_id in self._models

    def owner_of(self, model_id: str) -> str | None:
        """Owner of a committed model, None if unknown."""
        with self._lock:
            entry = self._models.get(model_id)
            return entry.owner if entry is not None and entry.committed else None

    def committed_ids(self, owner: str = '') -> list[str]:
        """Ids of committed models, optionally of one owner, in registration order."""
        with self._lock:
            return [
                model_id for model_id, entry in self._models.items()
                if entry.committed and (not owner or entry.owner == owner)
            ]

    def joints(self, model_id: str) -> list[tuple[DetachableJointDTO, JointInfoDTO]]:
        """Joint configuration and current state of a committed or pending model."""
        with self._lock:
            entry = self._models.get(model_id)
            if entry is None:
                return []
            return [(joint.config, joint.info()) for joint in entry.joints.values()]

    def joint_state(self, model_id: str, link_name: str) -> JointInfoDTO | None:
        """Current state of one joint, None if unknown."""
        with self._lock:
            entry = self._models.get(model_id)
            if entry is None or link_name not in entry.joints:
                return None
            return entry.joints[link_name].info()

    def state_topics(self, model_id: str) -> list[str]:
        """State topics of a model's joints."""
        with self._lock:
            entry = self._models.get(model_id)
            return [] if entry is None else sorted({j.config.state_topic for j in entry.joints.values()})

    def topic_in_use(self, topic: str) -> bool:
        """True if any registered or pending joint reports on ``topic``."""
        with self._lock:
            return any(j.config.state_topic == topic for j in self._joints(committed_only=False))

    def dependents_of(self, model_id: str) -> list[str]:
        """Committed models holding a joint on ``model_id`` that is not detached."""
        with self._lock:
            return [
                other.model_id for other in self._models.values()
                if other.committed and other.model_id != model_id and any(
                    joint.config.child_model == model_id and joint.state != JointStateEnum.DETACHED
                    for joint in other.joints.values())
            ]

    def cascade_holders_of(self, model_id: str) -> list[str]:
        """Committed models flagged ``delete_with_child`` that have a joint on ``model_id``, in any state."""
        with self._lock:
            return [
                other.model_id for other in self._models.values()
                if other.committed and other.delete_with_child and other.model_id != model_id and any(
                    joint.config.child_model == model_id for joint in other.joints.values())
            ]

    def apply_state_event(self, topic: str, data: str) -> bool:
        """Attribute a state event to its joint; returns True if the stored state changed.

        An event on a topic shared by several joints is ambiguous and is not attributed to any.
        """
        try:
            state = JointStateEnum(data.strip().lower())
        except ValueError:
            return False
        if state == JointStateEnum.UNKNOWN:
            return False
        with self._lock:
            candidates = [
                (entry, joint) for entry in self._models.values() for joint in entry.joints.values()
                if joint.config.state_topic == topic
            ]
            if len(candidates) != 1:
                return False
            entry, joint = candidates[0]
            if joint.state == state and joint.confirmed:
                return False
            joint.state = state
            joint.confirmed = True
            committed = entry.committed
        if committed:
            self._notify()
        return True

    def assume_attached(self, model_id: str) -> list[str]:
        """Record ``attached`` (unconfirmed) for joints without any observed event; returns their links."""
        with self._lock:
            entry = self._models.get(model_id)
            if entry is None:
                return []
            assumed = []
            for link_name, joint in entry.joints.items():
                if joint.state == JointStateEnum.UNKNOWN:
                    joint.state = JointStateEnum.ATTACHED
                    joint.confirmed = False
                    assumed.append(link_name)
            return assumed

    def snapshot(self, owner: str = '') -> list[ModelInfoDTO]:
        """Committed models with joint states."""
        with self._lock:
            return [
                ModelInfoDTO(
                    model_id=entry.model_id,
                    owner=entry.owner,
                    joints=tuple(joint.info() for joint in entry.joints.values()),
                )
                for entry in self._models.values()
                if entry.committed and (not owner or entry.owner == owner)
            ]
