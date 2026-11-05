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
"""MoveIt 2 planning-scene manager for the workplace objects."""

import threading
from collections.abc import Iterable
from pathlib import Path

import trimesh
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.callback_groups import CallbackGroup
from rclpy.client import Client
from rclpy.node import Node
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive

from station_workplace.enums.constants import CARRIER_OBJECT_ID, PRODUCT_OBJECT_ID, SAFETY_OBJECT_ID
from station_workplace.exceptions.planning_scene_error import PlanningSceneError
from station_workplace.interface.planning_scene_manager import PlanningSceneManager
from station_workplace.models.common_node_config_dto import SafetyObjectConfigDTO
from station_workplace.utils.async_service_caller import call_service
from station_workplace.utils.transformations import identity_pose


class _SceneObjects:
    """Snapshot of the collision objects and ACM in the planning scene."""

    def __init__(
        self,
        attached: dict[str, str],
        world: set[str],
        acm: AllowedCollisionMatrix,
    ) -> None:
        self.attached = attached  # object ID -> link it is attached to
        self.world = world
        self.acm = acm

    @property
    def ids(self) -> set[str]:
        return set(self.attached) | self.world


class Moveit2PlanningSceneManager(PlanningSceneManager):
    """Manage the workplace objects through ``GetPlanningScene``/``ApplyPlanningScene``.

    All objects are attached collision objects on ``attach_link``. Every update
    is sent as a single planning-scene diff, so the scene never shows a
    half-applied state.
    """

    def __init__(
        self,
        node: Node,
        attach_link: str,
        collision_allowed_link: str,
        safety_size: SafetyObjectConfigDTO,
        timeout_sec: float,
        callback_group: CallbackGroup,
        get_scene_service: str = '/get_planning_scene',
        apply_scene_service: str = '/apply_planning_scene',
    ) -> None:
        """Store the configuration; clients are created in :meth:`activate`.

        :param node: Node used to create clients and access the logger/clock.
        :param attach_link: Link the objects are attached to.
        :param collision_allowed_link: Link the carrier model and safety object may touch.
        :param safety_size: Size of the safety box.
        :param timeout_sec: Timeout for service availability and responses.
        :param callback_group: Callback group of the service clients.
        :param get_scene_service: ``GetPlanningScene`` service name.
        :param apply_scene_service: ``ApplyPlanningScene`` service name.
        """
        self._node = node
        self._logger = node.get_logger()
        self._attach_link = attach_link
        self._collision_allowed_link = collision_allowed_link
        self._safety_size = safety_size
        self._timeout_sec = timeout_sec
        self._callback_group = callback_group
        self._get_scene_service = get_scene_service
        self._apply_scene_service = apply_scene_service
        self._get_client: Client | None = None
        self._apply_client: Client | None = None
        self._lock = threading.Lock()

    # region lifecycle
    def activate(self) -> None:
        """Create the planning-scene service clients."""
        self._get_client = self._node.create_client(
            GetPlanningScene, self._get_scene_service, callback_group=self._callback_group)
        self._apply_client = self._node.create_client(
            ApplyPlanningScene, self._apply_scene_service, callback_group=self._callback_group)

    def deactivate(self) -> None:
        """Destroy the planning-scene service clients."""
        if self._get_client is not None:
            self._node.destroy_client(self._get_client)
            self._get_client = None
        if self._apply_client is not None:
            self._node.destroy_client(self._apply_client)
            self._apply_client = None
    # endregion lifecycle

    # region public API
    def get_present_object_ids(self, cancel: threading.Event) -> set[str]:
        """Return the IDs of all attached and world collision objects."""
        return self._query(cancel).ids

    def attach_safety_object(self, cancel: threading.Event) -> None:
        """Attach (or replace) the safety box on the attach link."""
        x, y, z = self._safety_size.x, self._safety_size.y, self._safety_size.z
        primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[x, y, z])
        # The box origin is its bottom outer edge; it spans -X, is centred on Y and spans +Z.
        primitive_pose = identity_pose()
        primitive_pose.position.x = -x / 2.0
        primitive_pose.position.z = z / 2.0

        safety = self._new_object(SAFETY_OBJECT_ID, identity_pose())
        safety.primitives = [primitive]
        safety.primitive_poses = [primitive_pose]

        scene = self._new_diff()
        scene.robot_state.attached_collision_objects = [
            self._attach(safety, [self._attach_link, self._collision_allowed_link]),
        ]
        with self._lock:
            self._apply(scene, 'attach safety object', cancel)
        self._logger.info(f"Attached '{SAFETY_OBJECT_ID}' to '{self._attach_link}'")

    def attach_product_and_carrier(
        self,
        product_mesh_path: Path,
        carrier_mesh_path: Path,
        product_pose: Pose,
        cancel: threading.Event,
    ) -> None:
        """Replace the safety object with the carrier and product models in one diff."""
        product_mesh = self._load_mesh(product_mesh_path)
        carrier_mesh = self._load_mesh(carrier_mesh_path)

        carrier = self._new_object(CARRIER_OBJECT_ID, identity_pose())
        carrier.meshes = [carrier_mesh]
        carrier.mesh_poses = [identity_pose()]

        product = self._new_object(PRODUCT_OBJECT_ID, product_pose)
        product.meshes = [product_mesh]
        product.mesh_poses = [identity_pose()]

        with self._lock:
            present = self._query(cancel)
            scene = self._new_diff()
            # Remove the safety object first; an ADD with geometry replaces any existing
            # attached or world object with the same ID, so product/carrier need no removal.
            self._add_removal(scene, present, [SAFETY_OBJECT_ID])
            scene.robot_state.attached_collision_objects = [
                *scene.robot_state.attached_collision_objects,
                self._attach(carrier, [self._attach_link, self._collision_allowed_link]),
                self._attach(product, [self._attach_link]),
            ]
            acm = present.acm
            self._drop_acm_entries(acm, [SAFETY_OBJECT_ID])
            self._set_acm_pair(acm, PRODUCT_OBJECT_ID, CARRIER_OBJECT_ID, True)
            scene.allowed_collision_matrix = acm
            self._apply(scene, 'attach product and carrier models', cancel)
        self._logger.info(
            f"Attached '{CARRIER_OBJECT_ID}' and '{PRODUCT_OBJECT_ID}' to '{self._attach_link}'")

    def remove_objects(self, object_ids: Iterable[str], cancel: threading.Event) -> None:
        """Remove the given objects; IDs that are not present are ignored."""
        requested = list(object_ids)
        with self._lock:
            present = self._query(cancel)
            to_remove = [object_id for object_id in requested if object_id in present.ids]
            if not to_remove:
                return
            scene = self._new_diff()
            self._add_removal(scene, present, to_remove)
            acm = present.acm
            if self._drop_acm_entries(acm, to_remove):
                scene.allowed_collision_matrix = acm
            try:
                self._apply(scene, f'remove {", ".join(to_remove)}', cancel)
            except PlanningSceneError:
                # A concurrent change between query and apply makes MoveIt reject the
                # diff (removing a missing object fails); the goal may still be met.
                if self._query(cancel).ids.intersection(to_remove):
                    raise
                self._logger.warning(
                    f'Removal diff rejected but {", ".join(to_remove)} are already gone')
                return
        self._logger.info(f'Removed {", ".join(to_remove)} from the planning scene')
    # endregion public API

    # region helpers
    def _query(self, cancel: threading.Event) -> _SceneObjects:
        """Fetch the object names, attached objects and ACM of the planning scene."""
        client = self._require(self._get_client)
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        response = call_service(
            client, request, self._timeout_sec, cancel, PlanningSceneError,
            'get planning scene')
        scene = response.scene
        attached = {
            aco.object.id: aco.link_name for aco in scene.robot_state.attached_collision_objects
        }
        world = {obj.id for obj in scene.world.collision_objects}
        return _SceneObjects(attached, world, scene.allowed_collision_matrix)

    def _apply(self, scene: PlanningScene, description: str, cancel: threading.Event) -> None:
        """Apply a planning-scene diff; raise ``PlanningSceneError`` if rejected."""
        client = self._require(self._apply_client)
        request = ApplyPlanningScene.Request()
        request.scene = scene
        response = call_service(
            client, request, self._timeout_sec, cancel, PlanningSceneError, description)
        if not response.success:
            raise PlanningSceneError(f'{description}: planning scene update rejected')

    @staticmethod
    def _require(client: Client | None) -> Client:
        if client is None:
            raise PlanningSceneError('planning scene manager is not active')
        return client

    @staticmethod
    def _new_diff() -> PlanningScene:
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        return scene

    def _new_object(self, object_id: str, pose: Pose) -> CollisionObject:
        obj = CollisionObject()
        obj.id = object_id
        obj.header.frame_id = self._attach_link
        obj.header.stamp = self._node.get_clock().now().to_msg()
        obj.pose = pose
        obj.operation = CollisionObject.ADD
        return obj

    def _attach(self, obj: CollisionObject, touch_links: list[str]) -> AttachedCollisionObject:
        attached = AttachedCollisionObject()
        attached.link_name = self._attach_link
        attached.object = obj
        attached.touch_links = list(dict.fromkeys(touch_links))
        return attached

    @staticmethod
    def _add_removal(
        scene: PlanningScene, present: _SceneObjects, object_ids: Iterable[str],
    ) -> None:
        """Add the diff entries deleting ``object_ids`` (only those present).

        Detaching an attached object only moves it back into the world, so the
        diff also removes it from the world. MoveIt processes the robot state
        before the world, so both steps fit in one diff.
        """
        detaches = list(scene.robot_state.attached_collision_objects)
        removes = list(scene.world.collision_objects)
        for object_id in object_ids:
            if object_id in present.attached:
                detach = AttachedCollisionObject()
                detach.link_name = present.attached[object_id]
                detach.object.id = object_id
                detach.object.operation = CollisionObject.REMOVE
                detaches.append(detach)
            if object_id in present.ids:
                remove = CollisionObject()
                remove.id = object_id
                remove.operation = CollisionObject.REMOVE
                removes.append(remove)
        scene.robot_state.attached_collision_objects = detaches
        scene.world.collision_objects = removes

    @staticmethod
    def _drop_acm_entries(acm: AllowedCollisionMatrix, names: Iterable[str]) -> bool:
        """Delete entries from the ACM in place; return whether anything changed."""
        names = set(names)
        entry_names = list(acm.entry_names)
        drop = {i for i, name in enumerate(entry_names) if name in names}
        if not drop:
            return False
        keep = [i for i in range(len(entry_names)) if i not in drop]
        rows = [list(entry.enabled) for entry in acm.entry_values]
        acm.entry_names = [entry_names[i] for i in keep]
        acm.entry_values = [
            AllowedCollisionEntry(enabled=[rows[i][j] for j in keep]) for i in keep
        ]
        acm.default_entry_values = [
            value for name, value in zip(acm.default_entry_names, acm.default_entry_values)
            if name not in names
        ]
        acm.default_entry_names = [
            name for name in acm.default_entry_names if name not in names
        ]
        return True

    @staticmethod
    def _set_acm_pair(
        acm: AllowedCollisionMatrix, name_a: str, name_b: str, allowed: bool,
    ) -> None:
        """Set the allowed state of a pair in the ACM, adding entries as needed."""
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

    @staticmethod
    def _load_mesh(path: Path) -> Mesh:
        """Load an STL file (meters, no scaling) into a ``shape_msgs/Mesh``."""
        try:
            loaded = trimesh.load(str(path), force='mesh')
        except Exception as exc:  # noqa: BLE001 - any loader failure is a mesh error
            raise PlanningSceneError(f"failed to load mesh '{path}': {exc}") from exc
        if not isinstance(loaded, trimesh.Trimesh) or len(loaded.faces) == 0:
            raise PlanningSceneError(f"'{path}' does not contain a triangle mesh")
        mesh = Mesh()
        mesh.vertices = [
            Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in loaded.vertices
        ]
        mesh.triangles = [
            MeshTriangle(vertex_indices=[int(f[0]), int(f[1]), int(f[2])]) for f in loaded.faces
        ]
        return mesh
    # endregion helpers
