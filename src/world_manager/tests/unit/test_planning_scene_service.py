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
# 3. Neither the name of the copyright holder nor its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""Unit tests for MoveIt's planning-scene diff construction."""

from copy import deepcopy
from pathlib import Path

from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from pytest import raises
from geometry_msgs.msg import Pose
from shape_msgs.msg import Mesh, SolidPrimitive
import trimesh

from world_manager.exceptions import MeshLoadError, ObjectNotFoundError, PlanningSceneError
from world_manager.models import (
    AllowedCollisionPairDTO,
    PoseDTO,
    SceneObjectDTO,
    SolidPrimitiveDTO,
    WorldChangesDTO,
)
from world_manager.services.planning_scene_service import PlanningSceneService
from world_manager.services.mesh_loader import load_mesh


class FakePlanningSceneGateway:
    """Records queries and applied diffs, optionally maintaining a simple scene snapshot."""

    def __init__(self) -> None:
        self.scene = PlanningScene()
        self.components: list[int] = []
        self.diffs: list[PlanningScene] = []
        self.fail_apply = False

    def get_scene(self, components: int) -> PlanningScene:
        self.components.append(components)
        return deepcopy(self.scene)

    def apply_scene(self, scene: PlanningScene) -> None:
        if self.fail_apply:
            raise PlanningSceneError('apply failed')
        self.diffs.append(deepcopy(scene))


def _world_object(object_id: str, mesh: bool = False) -> CollisionObject:
    obj = CollisionObject()
    obj.id = object_id
    obj.header.frame_id = 'world'
    obj.operation = CollisionObject.ADD
    if mesh:
        obj.meshes.append(Mesh())
        pose = Pose()
        pose.orientation.w = 1.0
        obj.mesh_poses.append(pose)
    else:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [1.0, 2.0, 3.0]
        obj.primitives.append(primitive)
        pose = Pose()
        pose.orientation.w = 1.0
        obj.primitive_poses.append(pose)
    return obj


def _attached_object(object_id: str, link_name: str, mesh: bool = False) -> AttachedCollisionObject:
    attached = AttachedCollisionObject()
    attached.link_name = link_name
    attached.object = _world_object(object_id, mesh)
    attached.object.header.frame_id = link_name
    return attached


def _primitive_object(
    object_id: str = 'box',
    attach_link: str = '',
    touch_links: tuple[str, ...] = (),
) -> SceneObjectDTO:
    return SceneObjectDTO(
        object_id=object_id,
        primitives=(SolidPrimitiveDTO(type=SolidPrimitive.BOX, dimensions=(1.0, 2.0, 3.0)),),
        primitive_poses=(PoseDTO(z=0.5),),
        pose=PoseDTO(frame_id=attach_link or 'world', x=1.0, y=2.0, z=3.0),
        attach_link=attach_link,
        touch_links=touch_links,
    )


def _operation(diff: PlanningScene, object_id: str, link_name: str | None = None) -> bytes | None:
    if link_name is None:
        for obj in diff.world.collision_objects:
            if obj.id == object_id:
                return obj.operation
        return None
    for attached in diff.robot_state.attached_collision_objects:
        if attached.object.id == object_id and attached.link_name == link_name:
            return attached.object.operation
    return None


def test_apply_changes_uses_one_query_and_one_diff() -> None:
    """Additions, removals and ACM changes use one scene read and one scene apply."""
    gateway = FakePlanningSceneGateway()
    gateway.scene.world.collision_objects.append(_world_object('gone'))
    service = PlanningSceneService(gateway)

    service.apply_world_changes(WorldChangesDTO(
        add_or_replace=(_primitive_object('new'),),
        remove_ids=('gone', 'absent'),
        acm_changes=(AllowedCollisionPairDTO(name_a='new', name_b='finger', allowed=True),),
    ))

    assert gateway.components == [
        PlanningSceneComponents.WORLD_OBJECT_NAMES
        | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    ]
    assert len(gateway.diffs) == 1
    diff = gateway.diffs[0]
    assert _operation(diff, 'gone') == CollisionObject.REMOVE
    assert _operation(diff, 'new') == CollisionObject.ADD
    assert diff.allowed_collision_matrix.entry_names == ['new', 'finger']
    assert diff.allowed_collision_matrix.entry_values[0].enabled == [False, True]


def test_replacing_attached_object_removes_attached_and_world_copies() -> None:
    """Replacement removes the old attachment and world id before adding the new object."""
    gateway = FakePlanningSceneGateway()
    gateway.scene.robot_state.attached_collision_objects.append(_attached_object('tool', 'mount'))
    service = PlanningSceneService(gateway)

    service.apply_world_changes(WorldChangesDTO(add_or_replace=(_primitive_object('tool'),)))

    diff = gateway.diffs[0]
    assert _operation(diff, 'tool', 'mount') == CollisionObject.REMOVE
    assert _operation(diff, 'tool') == CollisionObject.REMOVE
    assert sum(obj.id == 'tool' and obj.operation == CollisionObject.ADD
               for obj in diff.world.collision_objects) == 1


def test_replacing_world_object_removes_it_before_add() -> None:
    """A world-object replacement sends REMOVE and ADD for the same id in one diff."""
    gateway = FakePlanningSceneGateway()
    gateway.scene.world.collision_objects.append(_world_object('box'))
    service = PlanningSceneService(gateway)

    service.apply_world_changes(WorldChangesDTO(add_or_replace=(_primitive_object('box'),)))

    operations = [obj.operation for obj in gateway.diffs[0].world.collision_objects if obj.id == 'box']
    assert operations == [CollisionObject.REMOVE, CollisionObject.ADD]


def test_added_primitive_object_has_pose_and_geometry() -> None:
    """Primitive dimensions, primitive pose and object frame are copied into the diff."""
    gateway = FakePlanningSceneGateway()
    PlanningSceneService(gateway).apply_world_changes(
        WorldChangesDTO(add_or_replace=(_primitive_object(),)))

    obj = gateway.diffs[0].world.collision_objects[0]
    assert obj.header.frame_id == 'world'
    assert obj.pose.position.x == 1.0
    assert tuple(obj.primitives[0].dimensions) == (1.0, 2.0, 3.0)
    assert obj.primitive_poses[0].position.z == 0.5


def test_added_object_attaches_and_deduplicates_touch_links() -> None:
    """Attached objects include the attach link and unique touch links."""
    gateway = FakePlanningSceneGateway()
    attached_dto = _primitive_object('tool', 'mount', ('mount', 'finger', 'finger'))
    PlanningSceneService(gateway).apply_world_changes(
        WorldChangesDTO(add_or_replace=(attached_dto,)))

    diff = gateway.diffs[0]
    assert len(diff.world.collision_objects) == 0
    attached = diff.robot_state.attached_collision_objects[0]
    assert (attached.link_name, attached.object.header.frame_id) == ('mount', 'mount')
    assert attached.touch_links == ['mount', 'finger']


def test_added_object_rejects_pose_in_unmatched_attachment_frame() -> None:
    """Attached geometry must be expressed in its link frame."""
    gateway = FakePlanningSceneGateway()
    obj = _primitive_object('tool', 'mount').model_copy(
        update={'pose': PoseDTO(frame_id='world')})
    with raises(PlanningSceneError, match='must match attach_link'):
        PlanningSceneService(gateway).apply_world_changes(
            WorldChangesDTO(add_or_replace=(obj,)))
    assert gateway.components == []


def test_replaced_id_acm_entries_are_dropped_before_requested_updates() -> None:
    """Replacement drops old ACM defaults and matrix rows, then applies its requested pair."""
    gateway = FakePlanningSceneGateway()
    gateway.scene.world.collision_objects.append(_world_object('tool'))
    acm = gateway.scene.allowed_collision_matrix
    acm.entry_names = ['tool', 'finger']
    acm.entry_values = [
        AllowedCollisionEntry(enabled=[True, True]),
        AllowedCollisionEntry(enabled=[True, False]),
    ]
    acm.default_entry_names = ['tool', 'other']
    acm.default_entry_values = [True, False]

    PlanningSceneService(gateway).apply_world_changes(WorldChangesDTO(
        add_or_replace=(_primitive_object('tool'),),
        acm_changes=(AllowedCollisionPairDTO(name_a='tool', name_b='finger', allowed=False),),
    ))

    result = gateway.diffs[0].allowed_collision_matrix
    assert result.entry_names == ['finger', 'tool']
    assert result.entry_values[0].enabled == [False, False]
    assert result.entry_values[1].enabled == [False, False]
    assert result.default_entry_names == ['other']
    assert result.default_entry_values == [False]


def test_remove_drops_acm_entries_and_absent_ids_are_ignored() -> None:
    """Removing a present object drops its ACM row while absent ids generate no removals."""
    gateway = FakePlanningSceneGateway()
    gateway.scene.world.collision_objects.append(_world_object('tool'))
    gateway.scene.allowed_collision_matrix.entry_names = ['tool', 'finger']
    gateway.scene.allowed_collision_matrix.entry_values = [
        AllowedCollisionEntry(enabled=[True, True]),
        AllowedCollisionEntry(enabled=[True, False]),
    ]
    PlanningSceneService(gateway).apply_world_changes(
        WorldChangesDTO(remove_ids=('tool', 'absent')))

    diff = gateway.diffs[0]
    assert _operation(diff, 'tool') == CollisionObject.REMOVE
    assert _operation(diff, 'absent') is None
    assert diff.allowed_collision_matrix.entry_names == ['finger']


def test_new_acm_names_are_added_and_pair_is_symmetric() -> None:
    """Names absent from the ACM are added with a symmetric pair value."""
    gateway = FakePlanningSceneGateway()
    PlanningSceneService(gateway).apply_world_changes(WorldChangesDTO(
        acm_changes=(AllowedCollisionPairDTO(name_a='new_a', name_b='new_b', allowed=True),)))

    acm = gateway.diffs[0].allowed_collision_matrix
    assert acm.entry_names == ['new_a', 'new_b']
    assert [entry.enabled for entry in acm.entry_values] == [[False, True], [True, False]]


def test_object_cannot_be_both_added_and_removed() -> None:
    """A contradictory add/remove request is rejected before scene access."""
    gateway = FakePlanningSceneGateway()
    with raises(ValueError, match='both added and removed'):
        WorldChangesDTO(add_or_replace=(_primitive_object('x'),), remove_ids=('x',))
    assert gateway.components == []


def test_mesh_load_failure_leaves_scene_untouched() -> None:
    """Meshes are loaded before the scene is queried or changed."""
    gateway = FakePlanningSceneGateway()

    def fail_loader(path: str, file_type: str):
        raise MeshLoadError('invalid mesh')

    obj = SceneObjectDTO(object_id='mesh', mesh_path='/tmp/not-a-mesh.stl')
    with raises(MeshLoadError, match='invalid mesh'):
        PlanningSceneService(gateway, mesh_loader=fail_loader).apply_world_changes(
            WorldChangesDTO(add_or_replace=(obj,)))
    assert gateway.components == []
    assert gateway.diffs == []


def test_mesh_loader_receives_path_and_file_type_before_query() -> None:
    """An extensionless cached mesh path passes its declared file type to the loader."""
    gateway = FakePlanningSceneGateway()
    seen = []

    def loader(path: str, file_type: str):
        seen.append((path, file_type))
        return _world_object('fake').meshes

    obj = SceneObjectDTO(object_id='mesh', mesh_path='/cache/model', mesh_file_type='stl')
    PlanningSceneService(gateway, mesh_loader=loader).apply_world_changes(
        WorldChangesDTO(add_or_replace=(obj,)))
    assert seen == [('/cache/model', 'stl')]
    assert len(gateway.diffs[0].world.collision_objects[0].meshes) == 1


def test_attach_existing_world_geometry_to_link() -> None:
    """Attaching a world object resends its geometry and uses the destination pose/frame."""
    gateway = FakePlanningSceneGateway()
    original = _world_object('box')
    original.pose.position.x = 3.0
    gateway.scene.world.collision_objects.append(original)
    service = PlanningSceneService(gateway)

    service.attach_object('box', 'tool0', PoseDTO(frame_id='tool0', x=0.2), ('finger',))

    diff = gateway.diffs[0]
    assert _operation(diff, 'box') == CollisionObject.REMOVE
    attached = diff.robot_state.attached_collision_objects[0]
    assert attached.object.operation == CollisionObject.ADD
    assert attached.object.header.frame_id == 'tool0'
    assert attached.object.pose.position.x == 0.2
    assert len(attached.object.primitives) == 1
    assert attached.touch_links == ['tool0', 'finger']


def test_attach_existing_object_to_world() -> None:
    """Detaching an attached object sends REMOVE for the old link and an ADD with geometry to world."""
    gateway = FakePlanningSceneGateway()
    attached = _attached_object('tool', 'old_link', mesh=True)
    gateway.scene.robot_state.attached_collision_objects.append(attached)
    service = PlanningSceneService(gateway)

    service.attach_object('tool', '', PoseDTO(frame_id='world', z=0.5), ())

    diff = gateway.diffs[0]
    assert _operation(diff, 'tool', 'old_link') == CollisionObject.REMOVE
    world_obj = next(obj for obj in diff.world.collision_objects if obj.id == 'tool'
                     and obj.operation == CollisionObject.ADD)
    assert world_obj.header.frame_id == 'world'
    assert world_obj.pose.position.z == 0.5
    assert len(world_obj.meshes) == 1


def test_attach_existing_object_transfer_sends_geometry() -> None:
    """Moving an attached object between links sends geometry with the new attachment."""
    gateway = FakePlanningSceneGateway()
    gateway.scene.robot_state.attached_collision_objects.append(_attached_object('tool', 'rack', mesh=True))
    PlanningSceneService(gateway).attach_object(
        'tool', 'mount', PoseDTO(frame_id='mount', qz=0.5, qw=0.5), ('finger',))

    diff = gateway.diffs[0]
    assert _operation(diff, 'tool', 'rack') == CollisionObject.REMOVE
    target = next(item for item in diff.robot_state.attached_collision_objects
                  if item.link_name == 'mount' and item.object.operation == CollisionObject.ADD)
    assert len(target.object.meshes) == 1
    assert target.object.pose.orientation.z == 0.5


def test_attach_unknown_id_fails() -> None:
    """An object absent from world and attached lists is rejected."""
    gateway = FakePlanningSceneGateway()
    with raises(ObjectNotFoundError, match='missing'):
        PlanningSceneService(gateway).attach_object('missing', 'tool0', PoseDTO(), ())
    assert gateway.diffs == []


def test_attach_rejects_pose_frame_mismatch() -> None:
    """The pose frame must be empty or match the destination frame."""
    gateway = FakePlanningSceneGateway()
    service = PlanningSceneService(gateway)
    with raises(PlanningSceneError, match='must be empty'):
        service.attach_object('box', 'tool0', PoseDTO(frame_id='world'), ())
    assert gateway.components == []


def test_list_objects_reports_world_and_attached_ids() -> None:
    """ListObjects reports attachment links for attached objects and empty links for world objects."""
    gateway = FakePlanningSceneGateway()
    gateway.scene.world.collision_objects.extend([_world_object('a'), _world_object('b')])
    gateway.scene.robot_state.attached_collision_objects.append(_attached_object('c', 'tool0'))
    result = PlanningSceneService(gateway).list_objects()
    assert [(item.object_id, item.attach_link) for item in result] == [
        ('a', ''), ('b', ''), ('c', 'tool0')]
    assert gateway.components == [
        PlanningSceneComponents.WORLD_OBJECT_NAMES | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS]


def test_gateway_apply_errors_propagate() -> None:
    """A rejected or unavailable scene apply is not reported as success."""
    gateway = FakePlanningSceneGateway()
    gateway.fail_apply = True
    with raises(PlanningSceneError, match='apply failed'):
        PlanningSceneService(gateway).apply_world_changes(
            WorldChangesDTO(add_or_replace=(_primitive_object(),)))


def test_world_changes_reject_duplicate_added_ids() -> None:
    """Repeated object ids in add_or_replace are rejected by DTO validation."""
    with raises(ValueError, match='must be unique'):
        WorldChangesDTO(add_or_replace=(_primitive_object('x'), _primitive_object('x')))


def test_primitive_count_must_match_pose_count() -> None:
    """A primitive missing its corresponding pose is rejected before planning-scene access."""
    gateway = FakePlanningSceneGateway()
    with raises(ValueError, match='exactly one'):
        SceneObjectDTO(
            object_id='bad',
            primitives=(SolidPrimitiveDTO(type=SolidPrimitive.BOX, dimensions=(1.0, 1.0, 1.0)),),
            primitive_poses=(),
        )
    assert gateway.components == []


def test_mesh_loader_converts_stl_to_ros_mesh(tmp_path: Path) -> None:
    """A valid STL becomes vertices and triangle indices."""
    path = tmp_path / 'box.stl'
    trimesh.creation.box().export(path)
    mesh = load_mesh(str(path), 'stl')
    assert len(mesh.vertices) == 8
    assert len(mesh.triangles) == 12


def test_mesh_loader_rejects_nonexistent_file(tmp_path: Path) -> None:
    """A missing file reports an explicit mesh loading error."""
    with raises(MeshLoadError, match='failed to load'):
        load_mesh(str(tmp_path / 'missing.stl'), 'stl')
