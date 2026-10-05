# Copyright (c) 2026, Tools Manager Contributors
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
"""MoveIt 2 world manager — manage tool collision objects in the planning scene."""

from __future__ import annotations

import os
import time

import trimesh

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.client import Client as ServiceClient
from rclpy.node import Node

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
from shape_msgs.msg import Mesh, MeshTriangle

from tools_manager.model.tool_info_dto import ToolInfoDto
from tools_manager.model.tools_manager_config import ToolsManagerConfigDTO
from tools_manager.interface.world_manager import WorldManager
from tools_manager.exception.world_manager_exception import (
    ModelAttachError,
    ModelDeleteError,
    ModelSpawnError,
)



class Moveit2WorldManager(WorldManager):
    """Manage tool models as collision objects in the MoveIt 2 planning scene.

    Tools are represented as attached collision objects. A tool is always
    attached to exactly one link (a tool-rack slot link or the tool-mount
    link); moving it between links is a detach-then-attach transfer.
    """

    def __init__(
        self,
        node: Node,
        tools_manager_config: ToolsManagerConfigDTO,
        get_scene_service: str = '/get_planning_scene',
        apply_scene_service: str = '/apply_planning_scene',
        tool_mount_link: str = 'tool_mount_tcp',
        touch_links: list[str] | None = None,
        service_timeout_sec: float = 5.0,
    ) -> None:
        """Create the planning-scene service clients.

        :param node: Node used to create clients and access the logger/clock.
        :param tools_manager_config: Tools manager configuration used to resolve a
            tool's model directory from its metadata.
        :param get_scene_service: ``GetPlanningScene`` service name.
        :param apply_scene_service: ``ApplyPlanningScene`` service name.
        :param tool_mount_link: Link a tool is attached to when mounted.
        :param touch_links: Robot links a mounted tool is permitted to touch
            (in addition to the link it is attached to), used to populate the
            attached-object ``touch_links`` so expected contact is not flagged
            as a collision.
        :param service_timeout_sec: Timeout for service availability and results.
        """
        super().__init__()

        self._node = node
        self._logger = node.get_logger()
        self._tools_manager_config = tools_manager_config
        self._service_timeout_sec = service_timeout_sec
        self._tool_mount_link = tool_mount_link
        self._touch_links = list(touch_links) if touch_links else []
        self._callback_group = MutuallyExclusiveCallbackGroup()

        self._attached_link_lookup = {}

        self._get_client = node.create_client(
            GetPlanningScene,
            get_scene_service,
            callback_group=self._callback_group,
        )
        self._apply_client = node.create_client(
            ApplyPlanningScene,
            apply_scene_service,
            callback_group=self._callback_group,
        )

    def _generate_mesh(self, model_path: str) -> Mesh:
        """Load an STL file into a ``shape_msgs/Mesh``.

        :param model_path: Filesystem path to the STL model.
        :returns: The mesh as a ``shape_msgs/Mesh`` message.
        :raises ValueError: If the file does not contain triangle mesh geometry.
        """
        loaded = trimesh.load(model_path, force='mesh')
        if not isinstance(loaded, trimesh.Trimesh):
            raise ValueError(f'file {model_path} does not contain a triangle mesh')

        mesh = Mesh()
        mesh.vertices = [
            Point(x=float(vertex[0]), y=float(vertex[1]), z=float(vertex[2]))
            for vertex in loaded.vertices
        ]
        mesh.triangles = [
            MeshTriangle(vertex_indices=[int(face[0]), int(face[1]), int(face[2])])
            for face in loaded.faces
        ]
        return mesh

    def _generate_model_path(self, tool: ToolInfoDto) -> str:
        """Generate the filesystem path to a tool's STL model.

        The tool's model directory name is taken from its metadata
        (``ToolMetadataDto.model``) and resolved against the installed
        ``endtools`` package share directory, i.e.
        ``<endtools_share>/model/<model>/meshes/collision.stl``.

        :param tool: Tool information.
        :returns: Filesystem path to the STL model.
        :raises ModelSpawnError: If the tool has no metadata, the ``endtools``
            package cannot be found, or the model file does not exist.
        """
        metadata = self._tools_manager_config.get_tool_metadata(tool.tool_sn)
        if metadata is None:
            raise ModelSpawnError(
                f'No metadata defined for tool {tool.tool_sn}; cannot resolve model path'
            )

        try:
            share_directory = get_package_share_directory('endtools')
        except PackageNotFoundError as error:
            raise ModelSpawnError(
                f"Failed to locate 'endtools' package share directory: {error}"
            ) from error

        model_path = os.path.join(
            share_directory, 'model', metadata.model, 'meshes', 'collision.stl'
        )
        if not os.path.isfile(model_path):
            raise ModelSpawnError(
                f"Collision mesh '{model_path}' not found for tool {tool.tool_sn}"
            )
        return model_path

    def _transfer(self, model_id: str, source_link: str, target_link: str) -> bool:
        """Detach a model from one link and re-attach it to another.

        The object keeps its existing geometry; it is detached back into the
        world and then re-attached to ``target_link`` at zero pose.

        :param model_id: Unique model ID (equal to the tool serial number).
        :param source_link: Link the object is currently attached to.
        :param target_link: Link to attach the object to.
        :returns: ``True`` on success, ``False`` otherwise.
        """
        detach = self._build_attached_object(model_id, source_link, CollisionObject.REMOVE)
        if not self._apply_attached_object(detach, f'detach model {model_id} from {source_link}'):
            return False

        attach = self._build_attached_object(model_id, target_link, CollisionObject.ADD)
        return self._apply_attached_object(attach, f'attach model {model_id} to {target_link}')

    def _build_attached_object(
        self,
        model_id: str,
        link_name: str,
        operation: bytes,
        mesh: Mesh | None = None,
    ) -> AttachedCollisionObject:
        """Build an ``AttachedCollisionObject`` diff message.

        The object sits at the origin of ``link_name`` (zero pose). When ``mesh``
        is ``None`` the geometry is left empty so MoveIt reuses the existing
        object's shapes (used for transfers and removals).

        :param model_id: Unique model ID (equal to the tool serial number).
        :param link_name: Link the object is attached to.
        :param operation: ``CollisionObject`` operation (``ADD`` or ``REMOVE``).
        :param mesh: Mesh geometry to attach, or ``None`` to reuse existing.
        :returns: The populated ``AttachedCollisionObject`` message.
        """
        identity = Pose()
        # NOTE: a small offset is added to the x-coordinate to avoid collision with the link
        identity.position.x = 0.001
        identity.orientation.w = 1.0

        collision_object = CollisionObject()
        collision_object.id = model_id
        collision_object.header.frame_id = link_name
        collision_object.header.stamp = self._node.get_clock().now().to_msg()
        collision_object.operation = operation
        collision_object.pose = identity
        if mesh is not None:
            collision_object.meshes = [mesh]
            collision_object.mesh_poses = [identity]

        attached = AttachedCollisionObject()
        attached.link_name = link_name
        attached.object = collision_object
        if operation == CollisionObject.ADD:
            attached.touch_links = [link_name, *self._touch_links]
        return attached

    def _apply_attached_object(
        self,
        attached: AttachedCollisionObject,
        description: str,
    ) -> bool:
        """Apply an attached-collision-object diff to the planning scene.

        :param attached: The attached-collision-object diff to apply.
        :param description: Human-readable operation name for logging.
        :returns: ``True`` if the scene was applied successfully.
        """
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]
        return self._apply_scene(scene, description)

    def _apply_world_object(self, world_object: CollisionObject, description: str) -> bool:
        """Apply a world collision-object diff to the planning scene.

        :param world_object: The world collision-object diff to apply.
        :param description: Human-readable operation name for logging.
        :returns: ``True`` if the scene was applied successfully.
        """
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [world_object]
        return self._apply_scene(scene, description)

    def _get_allowed_collision_matrix(self) -> AllowedCollisionMatrix | None:
        """Fetch the current allowed-collision matrix from the planning scene.

        :returns: The allowed-collision matrix, or ``None`` on failure.
        """
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        response = self._call(self._get_client, request, 'get allowed collision matrix')
        if response is None:
            self._logger.error('Failed to get allowed collision matrix')
            return None
        return response.scene.allowed_collision_matrix

    @staticmethod
    def _set_acm_pair(
        acm: AllowedCollisionMatrix,
        name_a: str,
        name_b: str,
        allowed: bool,
    ) -> None:
        """Set the allowed state between two entities in the ACM in place.

        Entities not yet present are added, expanding the square matrix with
        ``False`` (collision-checked) defaults for all other pairs.

        :param acm: The allowed-collision matrix to modify in place.
        :param name_a: First entity name (link name or collision-object id).
        :param name_b: Second entity name (link name or collision-object id).
        :param allowed: ``True`` to allow collisions between the pair.
        """
        names: list[str] = list(acm.entry_names)
        rows: list[list[bool]] = [list(entry.enabled) for entry in acm.entry_values]

        for name in (name_a, name_b):
            if name not in names:
                names.append(name)
                for row in rows:
                    row.append(False)
                rows.append([False] * len(names))

        i = names.index(name_a)
        j = names.index(name_b)
        rows[i][j] = allowed
        rows[j][i] = allowed

        acm.entry_names = names
        acm.entry_values = [
            AllowedCollisionEntry(enabled=row) for row in rows
        ]

    def _apply_scene(self, scene: PlanningScene, description: str) -> bool:
        """Send a planning-scene diff through the ``ApplyPlanningScene`` service.

        :param scene: The planning-scene diff to apply.
        :param description: Human-readable operation name for logging.
        :returns: ``True`` if the service reported success.
        """
        request = ApplyPlanningScene.Request()
        request.scene = scene
        response = self._call(self._apply_client, request, description)
        if response is None or not response.success:
            self._logger.error(f'Planning scene apply failed: {description}')
            return False
        return True

    def _call(self, client: ServiceClient, request, description: str):
        """Call a service and wait for the result within the configured timeout.

        Uses ``call_async`` and polls the future so this can run on an executor
        thread without nested spinning (the response is serviced by another
        thread of the node's ``MultiThreadedExecutor``).

        :param client: Service client to call.
        :param request: Request message.
        :param description: Human-readable operation name for logging.
        :returns: The service response, or ``None`` on unavailability/timeout.
        """
        if not client.wait_for_service(timeout_sec=self._service_timeout_sec):
            self._logger.error(f'Service not available for: {description}')
            return None

        future = client.call_async(request)
        deadline = time.monotonic() + self._service_timeout_sec
        while not future.done():
            if time.monotonic() > deadline:
                self._logger.error(f'Service call timed out for: {description}')
                return None
            time.sleep(0.01)

        return future.result()

    def spawn_model(self, tool: ToolInfoDto, link_name: str):
        try:
            mesh = self._generate_mesh(self._generate_model_path(tool))
            attached = self._build_attached_object(
                tool.tool_sn,
                link_name,
                CollisionObject.ADD,
                mesh=mesh,
            )
            if not self._apply_attached_object(attached, f'spawn model {tool.tool_sn}'):
                raise ModelSpawnError(f'Failed to spawn model {tool.tool_sn} in MoveIt2 planning scene')

            self._attached_link_lookup[tool.tool_sn] = link_name
        except Exception as exc:
            self._logger.error(f'Failed to spawn model {tool.tool_sn}: {exc}')
            raise ModelSpawnError(f'Failed to spawn model {tool.tool_sn} in MoveIt2 planning scene: {exc}') from exc

    def delete_model(self, model_id: str):
        try:
            detach = self._build_attached_object(model_id, '', CollisionObject.REMOVE)
            if not self._apply_attached_object(detach, f'detach model {model_id}'):
                raise ModelDeleteError(f'Failed to delete model {model_id} in MoveIt2 planning scene')

            world_object = CollisionObject()
            world_object.id = model_id
            world_object.operation = CollisionObject.REMOVE
            if not self._apply_world_object(world_object, f'remove model {model_id}'):
                raise ModelDeleteError(f'Failed to delete model {model_id} in MoveIt2 planning scene')

            self._attached_link_lookup.pop(model_id, None)
        except Exception as exc:
            self._logger.error(f'Failed to delete model {model_id}: {exc}')
            raise ModelDeleteError(f'Failed to delete model {model_id} in MoveIt2 planning scene: {exc}') from exc

    def attach_to_link(self, model_id: str, link_name: str):
        try:
            source_link = self._attached_link_lookup.get(model_id, None)
            if source_link is None:
                self._logger.error(f'Model {model_id} is not currently attached to any link')
                raise ModelAttachError(f'Model {model_id} is not currently attached to any link')
            if source_link == link_name:
                return
            if not self._transfer(model_id, source_link, link_name):
                raise ModelAttachError(f'Failed to attach model {model_id} to link {link_name}')

            self._attached_link_lookup[model_id] = link_name
        except Exception as exc:
            self._logger.error(f'Failed to attach model {model_id} to tool-mount: {exc}')
            raise ModelAttachError(f'Failed to attach model {model_id} to tool-mount: {exc}') from exc

    def allow_collisions(self, model_id: str, allowed: bool, tool_mount_link: str, slot_link: str) -> bool:
        """Allow or disallow collisions among the tool, tool-mount, and rack slot.

        Permits the transient interpenetration that occurs while the tool-mount
        slides into (or out of) a tool sitting on the rack. The three entities —
        the tool collision object, the tool-mount link, and the tool's rack slot
        link — are pairwise allowed (or disallowed) in the allowed-collision
        matrix. Call with ``allowed=True`` before the maneuver and ``False``
        afterwards to restore normal collision checking.

        :param model_id: Unique model ID (equal to the tool serial number).
        :param allowed: ``True`` to allow collisions, ``False`` to re-enable checking.
        :returns: ``True`` on success, ``False`` otherwise.
        """
        try:
            acm = self._get_allowed_collision_matrix()
            if acm is None:
                return False

            entities = [model_id, tool_mount_link, slot_link]
            for i in range(len(entities)):
                for j in range(i + 1, len(entities)):
                    self._set_acm_pair(acm, entities[i], entities[j], allowed)

            scene = PlanningScene()
            scene.is_diff = True
            scene.allowed_collision_matrix = acm
            state = 'allow' if allowed else 'disallow'
            return self._apply_scene(scene, f'{state} collisions for model {model_id}')
        except Exception as exc:
            self._logger.error(f'Failed to update collisions for model {model_id}: {exc}')
            return False
