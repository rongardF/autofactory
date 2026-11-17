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
"""Atomic operations on the MoveIt planning scene."""

from copy import deepcopy
from threading import Lock
from typing import Callable

from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from world_manager.exceptions import ObjectNotFoundError, PlanningSceneError
from world_manager.models import (
    ObjectInfoDTO,
    PoseDTO,
    SceneObjectDTO,
    SolidPrimitiveDTO,
    WorldChangesDTO,
)
from world_manager.services.mesh_loader import load_mesh
from world_manager.services.planning_scene_gateway import PlanningSceneGateway


class PlanningSceneService:
    """Read-modify-write planning-scene operations serialized against each other."""

    _APPLY_COMPONENTS = (
        PlanningSceneComponents.WORLD_OBJECT_NAMES
        | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    )
    _ATTACH_COMPONENTS = (
        PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
    )
    _LIST_COMPONENTS = (
        PlanningSceneComponents.WORLD_OBJECT_NAMES
        | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
    )

    def __init__(
        self,
        gateway: PlanningSceneGateway,
        planning_frame: str = 'world',
        mesh_loader: Callable[[str, str], object] = load_mesh,
    ) -> None:
        """Store collaborators; mesh loading can be replaced in tests."""
        self._gateway = gateway
        self._planning_frame = planning_frame
        self._mesh_loader = mesh_loader
        self._lock = Lock()

    @staticmethod
    def _attached(scene: PlanningScene) -> dict[str, AttachedCollisionObject]:
        return {entry.object.id: entry for entry in scene.robot_state.attached_collision_objects}

    @staticmethod
    def _world(scene: PlanningScene) -> dict[str, CollisionObject]:
        return {entry.id: entry for entry in scene.world.collision_objects}

    def apply_world_changes(self, changes: WorldChangesDTO) -> None:
        """Apply removals, additions/replacements and ACM changes in exactly one diff.

        Meshes are loaded before locking or querying MoveIt, so a bad mesh leaves the scene untouched.
        """
        prepared = {
            obj.object_id: self._make_collision_object(obj, self._mesh_loader)
            for obj in changes.add_or_replace
        }
        with self._lock:
            current = self._gateway.get_scene(self._APPLY_COMPONENTS)
            attached = self._attached(current)
            world = self._world(current)
            current_ids = set(attached) | set(world)
            replaced = set(prepared) & current_ids
            actual_removals = set(changes.remove_ids) & current_ids
            affected = replaced | actual_removals

            diff = self._new_diff()
            self._append_removals(diff, affected, attached, world)

            acm = deepcopy(current.allowed_collision_matrix)
            acm_changed = self._drop_acm_entries(acm, affected)
            for pair in changes.acm_changes:
                self._set_acm_pair(acm, pair.name_a, pair.name_b, pair.allowed)
                acm_changed = True
            if acm_changed:
                diff.allowed_collision_matrix = acm

            for dto in changes.add_or_replace:
                obj = prepared[dto.object_id]
                if dto.attach_link:
                    attached_obj = AttachedCollisionObject()
                    attached_obj.link_name = dto.attach_link
                    attached_obj.object = obj
                    attached_obj.touch_links = list(dict.fromkeys((dto.attach_link, *dto.touch_links)))
                    diff.robot_state.attached_collision_objects.append(attached_obj)
                else:
                    diff.world.collision_objects.append(obj)

            self._gateway.apply_scene(diff)

    def attach_object(
        self,
        object_id: str,
        link_name: str,
        pose: PoseDTO,
        touch_links: tuple[str, ...] | list[str],
    ) -> None:
        """Move existing collision geometry between the world and a robot link."""
        expected_frame = link_name if link_name else self._planning_frame
        if pose.frame_id and pose.frame_id != expected_frame:
            raise PlanningSceneError(
                f"pose frame '{pose.frame_id}' must be empty or '{expected_frame}'")

        with self._lock:
            scene = self._gateway.get_scene(self._ATTACH_COMPONENTS)
            attached = self._attached(scene)
            world = self._world(scene)
            source = attached.get(object_id)
            world_object = world.get(object_id)
            if source is None and world_object is None:
                raise ObjectNotFoundError(f"object '{object_id}' does not exist in the planning scene")

            diff = self._new_diff()
            if source is not None:
                detach = AttachedCollisionObject()
                detach.link_name = source.link_name
                detach.object.id = object_id
                detach.object.operation = CollisionObject.REMOVE
                diff.robot_state.attached_collision_objects.append(detach)
                geometry = deepcopy(source.object)
            else:
                remove = CollisionObject()
                remove.id = object_id
                remove.operation = CollisionObject.REMOVE
                diff.world.collision_objects.append(remove)
                geometry = deepcopy(world_object)

            geometry.id = object_id
            geometry.header.frame_id = expected_frame
            geometry.pose = PoseDTO.to_msg(pose)
            geometry.operation = CollisionObject.ADD

            if link_name:
                attached_obj = AttachedCollisionObject()
                attached_obj.link_name = link_name
                attached_obj.object = geometry
                attached_obj.touch_links = list(dict.fromkeys((link_name, *touch_links)))
                diff.robot_state.attached_collision_objects.append(attached_obj)
            else:
                diff.world.collision_objects.append(geometry)

            self._gateway.apply_scene(diff)

    def list_objects(self) -> list[ObjectInfoDTO]:
        """Return the world and attached object ids from the scene."""
        with self._lock:
            scene = self._gateway.get_scene(self._LIST_COMPONENTS)
        attached = self._attached(scene)
        world = self._world(scene)
        result = [ObjectInfoDTO(object_id=object_id, attach_link=attached[object_id].link_name)
                  for object_id in attached]
        result.extend(ObjectInfoDTO(object_id=object_id) for object_id in world if object_id not in attached)
        return sorted(result, key=lambda item: item.object_id)

    @staticmethod
    def _new_diff() -> PlanningScene:
        diff = PlanningScene()
        diff.is_diff = True
        diff.robot_state.is_diff = True
        return diff

    def _make_collision_object(self, dto: SceneObjectDTO, mesh_loader: Callable) -> CollisionObject:
        if dto.attach_link and dto.pose.frame_id and dto.pose.frame_id != dto.attach_link:
            raise PlanningSceneError(
                f"object '{dto.object_id}' pose frame '{dto.pose.frame_id}' must match attach_link "
                f"'{dto.attach_link}'")
        obj = CollisionObject()
        obj.id = dto.object_id
        obj.header.frame_id = dto.attach_link or dto.pose.frame_id or self._planning_frame
        obj.pose = PoseDTO.to_msg(dto.pose)
        obj.operation = CollisionObject.ADD
        if dto.mesh_path:
            mesh = mesh_loader(dto.mesh_path, dto.mesh_file_type)
            obj.meshes = [mesh]
            identity = Pose()
            identity.orientation.w = 1.0
            obj.mesh_poses = [identity]
        else:
            obj.primitives = [SolidPrimitiveDTO.to_msg(item) for item in dto.primitives]
            obj.primitive_poses = [PoseDTO.to_msg(item) for item in dto.primitive_poses]
        return obj

    @staticmethod
    def _append_removals(
        diff: PlanningScene,
        object_ids: set[str],
        attached: dict[str, AttachedCollisionObject],
        world: dict[str, CollisionObject],
    ) -> None:
        """Remove attached copies first and their world copies from the same diff."""
        for object_id in sorted(object_ids):
            if object_id in attached:
                remove_attached = AttachedCollisionObject()
                remove_attached.link_name = attached[object_id].link_name
                remove_attached.object.id = object_id
                remove_attached.object.operation = CollisionObject.REMOVE
                diff.robot_state.attached_collision_objects.append(remove_attached)
            if object_id in world or object_id in attached:
                remove_world = CollisionObject()
                remove_world.id = object_id
                remove_world.operation = CollisionObject.REMOVE
                diff.world.collision_objects.append(remove_world)

    @staticmethod
    def _drop_acm_entries(acm: AllowedCollisionMatrix, names: set[str]) -> bool:
        """Remove matrix rows/columns and default entries for object names."""
        names = set(names)
        old_names = list(acm.entry_names)
        drop = {index for index, name in enumerate(old_names) if name in names}
        changed = bool(drop)
        if drop:
            keep = [index for index in range(len(old_names)) if index not in drop]
            old_rows = [list(entry.enabled) for entry in acm.entry_values]
            acm.entry_names = [old_names[index] for index in keep]
            acm.entry_values = [
                AllowedCollisionEntry(enabled=[old_rows[row][column] for column in keep])
                for row in keep
            ]
        defaults = [
            (name, value) for name, value in zip(acm.default_entry_names, acm.default_entry_values)
            if name not in names
        ]
        if len(defaults) != len(acm.default_entry_names):
            changed = True
            acm.default_entry_names = [name for name, _ in defaults]
            acm.default_entry_values = [value for _, value in defaults]
        return changed

    @staticmethod
    def _set_acm_pair(acm: AllowedCollisionMatrix, name_a: str, name_b: str, allowed: bool) -> None:
        """Set a symmetric ACM pair, adding names with collision-checked defaults."""
        names = list(acm.entry_names)
        rows = [list(entry.enabled) for entry in acm.entry_values]
        for name in (name_a, name_b):
            if name not in names:
                names.append(name)
                for row in rows:
                    row.append(False)
                rows.append([False] * len(names))
        i, j = names.index(name_a), names.index(name_b)
        rows[i][j] = allowed
        rows[j][i] = allowed
        acm.entry_names = names
        acm.entry_values = [AllowedCollisionEntry(enabled=row) for row in rows]
