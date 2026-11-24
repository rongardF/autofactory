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
"""Operations of the Gazebo world manager, independent of ROS."""

from threading import Lock
from typing import Sequence

from rclpy.impl.rcutils_logger import RcutilsLogger

from world_manager.exceptions import (
    ActivationError,
    GzRequestError,
    ModelNotFoundError,
    ModelValidationError,
    OperationTimeoutError,
)
from world_manager.models import (
    DeleteResultDTO,
    GazeboConfigDTO,
    ModelInfoDTO,
    ModelSpecDTO,
    PoseDTO,
    SdfModelDTO,
)
from world_manager.services.detachable_joint_controller import DetachableJointController
from world_manager.services.gz_transport_adapter import GzTransportAdapter
from world_manager.services.model_registry import ModelRegistry
from world_manager.services.pose_resolver import PoseResolver
from world_manager.services.sdf_inspector import SdfInspector
from world_manager.utils import Deadline

class GazeboWorldService:
    """Spawns, deletes, moves and (de)attaches models. Failures are raised as WorldManagerError.

    The operation lock serializes the mutating calls and may be held for seconds; ``list_models``
    only takes the registry lock so it never waits behind a long spawn.
    """

    def __init__(
        self,
        adapter: GzTransportAdapter,
        registry: ModelRegistry,
        controller: DetachableJointController,
        inspector: SdfInspector,
        resolver: PoseResolver,
        config: GazeboConfigDTO,
        logger: RcutilsLogger,
    ) -> None:
        """Store collaborators."""
        self._adapter = adapter
        self._registry = registry
        self._controller = controller
        self._inspector = inspector
        self._resolver = resolver
        self._config = config
        self._logger = logger
        self._operation_lock = Lock()

    def activate(self) -> None:
        """Reconcile the registry with the world; raises ActivationError if the world does not answer."""
        with self._operation_lock:
            if self.reconcile() is None:
                raise ActivationError(
                    f'/world/{self._config.world_name}/scene/info did not answer; the world must run the '
                    'SceneBroadcaster and UserCommands systems')

    def reconcile(self) -> list[str] | None:
        """Drop registry entries whose model is gone; returns the model names in the world, None if unanswered."""
        names = self._adapter.list_models(self._config.world_name, self._config.service_timeout_sec)
        if names is None:
            return None
        for model_id in self._registry.drop_missing(names):
            self._logger.warning(f"model '{model_id}' no longer exists in the world; dropped from the registry")
        return names

    def list_models(self, owner: str = '') -> list[ModelInfoDTO]:
        """Registered models with joint states."""
        return self._registry.snapshot(owner)

    def spawn(self, models: Sequence[ModelSpecDTO], owner: str = '') -> list[str]:
        """Spawn all models or none; returns the spawned ids."""
        with self._operation_lock:
            return self._spawn(models, owner, Deadline(self._config.operation_timeout_sec))

    def _validate_spawn(self, models: Sequence[ModelSpecDTO]) -> dict[str, tuple[SdfModelDTO, PoseDTO]]:
        if not models:
            raise ModelValidationError('no models to spawn')
        ids = [spec.model_id for spec in models]
        if len(set(ids)) != len(ids):
            raise ModelValidationError('model ids in one spawn call must be unique')
        parsed: dict[str, tuple[SdfModelDTO, PoseDTO]] = {}
        for spec in models:
            if self._registry.contains(spec.model_id):
                raise ModelValidationError(f"model '{spec.model_id}' is already registered")
            sdf_model = self._inspector.inspect(spec.sdf)
            if sdf_model.model_name != spec.model_id:
                raise ModelValidationError(
                    f"model_id '{spec.model_id}' does not match the SDF model name '{sdf_model.model_name}'")
            if spec.attach_link and spec.attach_link not in {joint.link_name for joint in sdf_model.joints}:
                raise ModelValidationError(
                    f"model '{spec.model_id}' has no DetachableJoint with child_link '{spec.attach_link}'")
            parsed[spec.model_id] = (sdf_model, self._resolver.to_world(spec.pose))
        return parsed

    def _unmanaged_conflicts(self, models: Sequence[ModelSpecDTO]) -> bool:
        """Reject ids that already exist in the world; returns False if the world could not be read."""
        names = self.reconcile()
        if names is None:
            return False
        for spec in models:
            if spec.model_id in names:
                raise ModelValidationError(
                    f"cannot spawn '{spec.model_id}': model exists in the world but is not managed")
        return True

    def _spawn(self, models: Sequence[ModelSpecDTO], owner: str, deadline: Deadline) -> list[str]:
        parsed = self._validate_spawn(models)
        world_checked = self._unmanaged_conflicts(models)
        ids = [spec.model_id for spec in models]
        topics = sorted({joint.state_topic for sdf_model, _ in parsed.values() for joint in sdf_model.joints})
        created: list[str] = []
        try:
            for spec in models:
                self._registry.add_pending(spec.model_id, owner, parsed[spec.model_id][0].joints, spec.delete_with_child)
            for spec in models:
                if spec.attach_link:
                    self._require_controllable(spec.model_id)
            # Subscribe before creating so the first-step 'attached' event is not lost.
            self._controller.watch(topics)

            for spec in models:
                deadline.check(f"creating '{spec.model_id}'")
                if not self._adapter.create_model(
                        self._config.world_name, spec.model_id, spec.sdf, parsed[spec.model_id][1],
                        self._config.service_timeout_sec):
                    raise GzRequestError(self._describe_create_failure(spec.model_id, world_checked, created))
                created.append(spec.model_id)
                if self._registry.assume_attached(spec.model_id):
                    self._logger.warning(
                        f"model '{spec.model_id}' spawned with assumed (unconfirmed) joint state 'attached'")

            for spec in models:
                if spec.attach_link:
                    self._detach_others(spec.model_id, spec.attach_link, deadline)
        except Exception:
            self._rollback(created)
            self._registry.discard(ids)
            self._controller.unwatch(topics)
            raise
        self._registry.commit(ids)
        return ids

    def _require_controllable(self, model_id: str) -> None:
        for config, info in self._registry.joints(model_id):
            if not info.controllable:
                topic = self._registry.collision_topic(model_id, config.link_name)
                raise ModelValidationError(
                    f"model '{model_id}' has attach_link set but joint '{config.link_name}' is not "
                    f'controllable: topic {topic} is shared with another joint')

    def _describe_create_failure(self, model_id: str, world_checked: bool, created: list[str]) -> str:
        """Explain a failed create and register a model that appeared despite it for rollback.

        The id was absent before the call (``world_checked``), so a model that is present now was created
        by this request even though it did not answer in time.
        """
        names = self.reconcile()
        if names is not None and model_id in names:
            if world_checked:
                created.append(model_id)
                return f"gz did not confirm the create request for '{model_id}' but the model exists; rolling back"
            return f"cannot spawn '{model_id}': model exists in the world but is not managed"
        return f"gz refused or did not answer the create request for '{model_id}'"

    def _detach_others(self, model_id: str, keep_link: str, deadline: Deadline) -> None:
        for config, _ in self._registry.joints(model_id):
            if config.link_name != keep_link:
                self._controller.set_attached(
                    model_id, config.link_name, False, self._config.confirm_timeout_sec, deadline)

    def _rollback(self, created: Sequence[str]) -> None:
        """Delete the models created by a failed spawn, in reverse order."""
        for model_id in reversed(created):
            self._controller.detach_all_best_effort(model_id, self._config.rollback_timeout_sec)
            if not self._adapter.remove_model(
                    self._config.world_name, model_id, self._config.service_timeout_sec):
                self._logger.error(f"rollback could not remove '{model_id}' from the world")

    def delete(self, model_ids: Sequence[str] = (), owner: str = '') -> tuple[DeleteResultDTO, str]:
        """Delete models; returns the result and an error message that is empty on full success.

        Models spawned with ``delete_with_child`` that hold a joint on a deleted model are deleted
        with it, first, whatever their owner. Any other attached holder outside the call is rejected
        up front with ModelValidationError, before anything is deleted.
        """
        with self._operation_lock:
            return self._delete(model_ids, owner)

    def _plan_delete(self, model_ids: Sequence[str], owner: str) -> tuple[list[str], list[str]]:
        if not model_ids and not owner:
            raise ModelValidationError('either model_ids or owner must be given')
        if not model_ids:
            return self._registry.committed_ids(owner), []
        targets: list[str] = []
        unknown: list[str] = []
        for model_id in dict.fromkeys(model_ids):
            registered_owner = self._registry.owner_of(model_id)
            if registered_owner is None:
                unknown.append(model_id)
            elif owner and registered_owner != owner:
                raise ModelValidationError(f"model '{model_id}' is not owned by '{owner}'")
            else:
                targets.append(model_id)
        return targets, unknown

    def _with_cascade(self, targets: list[str]) -> tuple[list[str], set[str]]:
        """Add the ``delete_with_child`` holders of every target, transitively; returns the targets and the added ids."""
        expanded = list(targets)
        cascaded: set[str] = set()
        queue = list(targets)
        while queue:
            for holder in self._registry.cascade_holders_of(queue.pop(0)):
                if holder not in expanded:
                    expanded.append(holder)
                    cascaded.add(holder)
                    queue.append(holder)
        return expanded, cascaded

    def _delete_order(self, targets: list[str]) -> list[str]:
        """Order the targets so that a model is deleted only after the models holding a joint on it."""
        target_set = set(targets)
        holders = {
            model_id: {other for other in self._registry.dependents_of(model_id) if other in target_set}
            for model_id in targets
        }
        for model_id in targets:
            outside = [other for other in self._registry.dependents_of(model_id) if other not in target_set]
            if outside:
                raise ModelValidationError(
                    f"cannot delete '{model_id}': '{outside[0]}' is still attached to it and is not part of the call")
        ordered: list[str] = []
        remaining = list(targets)
        while remaining:
            eligible = [m for m in remaining if not holders[m] & set(remaining)]
            chosen = eligible[0] if eligible else remaining[0]
            ordered.append(chosen)
            remaining.remove(chosen)
        return ordered

    def _delete_one(self, model_id: str) -> bool:
        topics = self._registry.state_topics(model_id)
        self._controller.detach_all_best_effort(model_id, self._config.rollback_timeout_sec)
        if self._adapter.remove_model(self._config.world_name, model_id, self._config.service_timeout_sec):
            self._registry.remove(model_id)
        else:
            names = self.reconcile()
            if names is None or model_id in names:
                return False
            self._logger.warning(f"model '{model_id}' was already gone from the world")
        self._controller.unwatch(topics)
        return True

    def _delete(self, model_ids: Sequence[str], owner: str) -> tuple[DeleteResultDTO, str]:
        targets, unknown = self._plan_delete(model_ids, owner)
        targets, cascaded = self._with_cascade(targets)
        ordered = self._delete_order(targets)
        deadline = Deadline(self._config.operation_timeout_sec)
        deleted: list[str] = []
        failed: list[str] = []
        skipped: list[str] = []
        error = ''
        for model_id in ordered:
            if model_id in cascaded:
                # Cascaded models are not part of the request: they neither expire the deadline nor
                # use up the time left for the requested models.
                if self._delete_one(model_id):
                    deleted.append(model_id)
                else:
                    failed.append(model_id)
                deadline = Deadline(self._config.operation_timeout_sec)
                continue
            try:
                deadline.check(f"deleting '{model_id}'")
            except OperationTimeoutError as e:
                error = str(e)
                break
            if any(holder in failed for holder in self._registry.dependents_of(model_id)):
                # A failed holder may still keep its joint on this model in the physics engine.
                skipped.append(model_id)
            elif self._delete_one(model_id):
                deleted.append(model_id)
            else:
                failed.append(model_id)
        if failed:
            error = f"failed to delete: {', '.join(failed)}" + (f'; {error}' if error else '')
        if skipped:
            error = (f"not deleted because an attached holder could not be deleted: {', '.join(skipped)}"
                     + (f'; {error}' if error else ''))
        if unknown:
            self._logger.warning(f"delete: unknown model ids: {', '.join(unknown)}")
        return DeleteResultDTO(deleted_ids=tuple(deleted), unknown_ids=tuple(unknown)), error

    def set_pose(self, model_id: str, pose: PoseDTO) -> None:
        """Move a registered model; a model removed externally fails fast."""
        with self._operation_lock:
            if not self._registry.is_registered(model_id):
                raise ModelNotFoundError(f"model '{model_id}' is not managed")
            if self.reconcile() is None:
                raise GzRequestError('scene/info did not answer')
            if not self._registry.is_registered(model_id):
                raise ModelNotFoundError(f"model '{model_id}' no longer exists")
            world_pose = self._resolver.to_world(pose)
            if not self._adapter.set_model_pose(
                    self._config.world_name, model_id, world_pose, self._config.service_timeout_sec):
                raise GzRequestError(f"gz refused or did not answer set_pose for '{model_id}'")

    def set_attachment(self, model_id: str, link_name: str) -> None:
        """Attach ``link_name`` and then detach all others; an empty name detaches everything."""
        with self._operation_lock:
            deadline = Deadline(self._config.operation_timeout_sec)
            if not self._registry.is_registered(model_id):
                raise ModelNotFoundError(f"model '{model_id}' is not managed")
            links = [config.link_name for config, _ in self._registry.joints(model_id)]
            if link_name and link_name not in links:
                raise ModelValidationError(f"model '{model_id}' has no joint with child link '{link_name}'")
            if link_name:
                # Attach first so the model is never held by zero joints while it moves between links.
                self._controller.set_attached(model_id, link_name, True, self._config.confirm_timeout_sec, deadline)
            for other in links:
                if other != link_name:
                    self._controller.set_attached(model_id, other, False, self._config.confirm_timeout_sec, deadline)
