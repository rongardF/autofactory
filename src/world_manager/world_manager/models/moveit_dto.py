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
"""Pydantic DTOs for planning-scene objects and changes."""

from pydantic import BaseModel, ConfigDict, Field, model_validator

from shape_msgs.msg import SolidPrimitive

from world_manager.models.pose_dto import PoseDTO
from world_manager.msg import AllowedCollisionPair, ObjectInfo, SceneObject
from world_manager.srv import ApplyWorldChanges


class SolidPrimitiveDTO(BaseModel):
    """A ROS SolidPrimitive type and its dimensions."""

    model_config = ConfigDict(frozen=True)

    type: int = Field(description='shape_msgs/SolidPrimitive type constant')
    dimensions: tuple[float, ...] = Field(description='Primitive dimensions in meters and radians')

    @staticmethod
    def from_msg(message: SolidPrimitive) -> 'SolidPrimitiveDTO':
        """Convert a ROS primitive."""
        return SolidPrimitiveDTO(type=message.type, dimensions=tuple(message.dimensions))

    @staticmethod
    def to_msg(dto: 'SolidPrimitiveDTO') -> SolidPrimitive:
        """Convert a primitive to ROS."""
        return SolidPrimitive(type=dto.type, dimensions=list(dto.dimensions))


class SceneObjectDTO(BaseModel):
    """Collision object to add or replace in the planning scene."""

    model_config = ConfigDict(frozen=True)

    object_id: str = Field(min_length=1, description='Unique collision object id')
    mesh_path: str = Field(default='', description='Absolute STL path in meters; empty when using primitives')
    mesh_file_type: str = Field(default='stl', description='Mesh file type (station cache files may have no extension)')
    primitives: tuple[SolidPrimitiveDTO, ...] = Field(default=(), description='Primitive geometry')
    primitive_poses: tuple[PoseDTO, ...] = Field(default=(), description='Pose for each primitive in the object frame')
    pose: PoseDTO = Field(default_factory=PoseDTO, description='Object pose and its frame')
    attach_link: str = Field(default='', description='Robot link to attach to; empty places the object in the world')
    touch_links: tuple[str, ...] = Field(default=(), description='Additional allowed-contact links for attached objects')

    @staticmethod
    def from_msg(message: SceneObject) -> 'SceneObjectDTO':
        """Convert a scene object message."""
        return SceneObjectDTO(
            object_id=message.object_id, mesh_path=message.mesh_path,
            mesh_file_type=message.mesh_file_type or 'stl',
            primitives=tuple(SolidPrimitiveDTO.from_msg(item) for item in message.primitives),
            primitive_poses=tuple(PoseDTO.from_msg(item) for item in message.primitive_poses),
            pose=PoseDTO.from_stamped_msg(message.pose),
            attach_link=message.attach_link, touch_links=tuple(message.touch_links),
        )

    @staticmethod
    def to_msg(dto: 'SceneObjectDTO') -> SceneObject:
        """Convert a scene object DTO to ROS."""
        message = SceneObject()
        message.object_id, message.mesh_path = dto.object_id, dto.mesh_path
        message.mesh_file_type = dto.mesh_file_type
        message.primitives = [SolidPrimitiveDTO.to_msg(item) for item in dto.primitives]
        message.primitive_poses = [PoseDTO.to_msg(item) for item in dto.primitive_poses]
        message.pose = PoseDTO.to_stamped_msg(dto.pose)
        message.attach_link, message.touch_links = dto.attach_link, list(dto.touch_links)
        return message

    @model_validator(mode='after')
    def validate_geometry(self) -> 'SceneObjectDTO':
        """Require one valid geometry representation and matching primitive poses."""
        if self.mesh_path and (self.primitives or self.primitive_poses):
            raise ValueError('mesh_path cannot be combined with primitive geometry')
        if self.primitives and len(self.primitives) != len(self.primitive_poses):
            raise ValueError('each primitive must have exactly one primitive pose')
        if not self.mesh_path and not self.primitives:
            raise ValueError('a collision object needs mesh_path or primitive geometry')
        return self


class AllowedCollisionPairDTO(BaseModel):
    """Allowed-collision matrix update for one pair of names."""

    model_config = ConfigDict(frozen=True)

    name_a: str = Field(min_length=1, description='First robot link or collision object id')
    name_b: str = Field(min_length=1, description='Second robot link or collision object id')
    allowed: bool = Field(description='Whether collisions between the names are allowed')

    @staticmethod
    def from_msg(message: AllowedCollisionPair) -> 'AllowedCollisionPairDTO':
        """Convert an ACM pair from ROS."""
        return AllowedCollisionPairDTO(name_a=message.name_a, name_b=message.name_b, allowed=message.allowed)

    @staticmethod
    def to_msg(dto: 'AllowedCollisionPairDTO') -> AllowedCollisionPair:
        """Convert an ACM pair to ROS."""
        return AllowedCollisionPair(name_a=dto.name_a, name_b=dto.name_b, allowed=dto.allowed)


class WorldChangesDTO(BaseModel):
    """One atomic planning-scene diff request."""

    model_config = ConfigDict(frozen=True)

    add_or_replace: tuple[SceneObjectDTO, ...] = Field(default=(), description='Objects to add or replace')
    remove_ids: tuple[str, ...] = Field(default=(), description='Object ids to remove; absent ids are ignored')
    acm_changes: tuple[AllowedCollisionPairDTO, ...] = Field(default=(), description='Allowed-collision pair updates')

    @staticmethod
    def from_msg(message: ApplyWorldChanges.Request) -> 'WorldChangesDTO':
        """Convert a world changes service request."""
        return WorldChangesDTO(
            add_or_replace=tuple(SceneObjectDTO.from_msg(item) for item in message.add_or_replace),
            remove_ids=tuple(message.remove_ids),
            acm_changes=tuple(AllowedCollisionPairDTO.from_msg(item) for item in message.acm_changes),
        )

    @staticmethod
    def to_msg(dto: 'WorldChangesDTO') -> ApplyWorldChanges.Request:
        """Convert a world changes DTO into a service request."""
        request = ApplyWorldChanges.Request()
        request.add_or_replace = [SceneObjectDTO.to_msg(item) for item in dto.add_or_replace]
        request.remove_ids = list(dto.remove_ids)
        request.acm_changes = [AllowedCollisionPairDTO.to_msg(item) for item in dto.acm_changes]
        return request

    @model_validator(mode='after')
    def validate_ids(self) -> 'WorldChangesDTO':
        """Reject ambiguous remove-and-add requests and duplicate add ids."""
        add_ids = [obj.object_id for obj in self.add_or_replace]
        if len(set(add_ids)) != len(add_ids):
            raise ValueError('add_or_replace object ids must be unique')
        overlap = set(add_ids) & set(self.remove_ids)
        if overlap:
            raise ValueError(f'object id {sorted(overlap)[0]} cannot be both added and removed')
        return self


class ObjectInfoDTO(BaseModel):
    """Object id and its current attachment link, if any."""

    model_config = ConfigDict(frozen=True)

    object_id: str = Field(description='Collision object id in the planning scene')
    attach_link: str = Field(default='', description='Attached robot link; empty when the object is in the world')

    @staticmethod
    def from_msg(message: ObjectInfo) -> 'ObjectInfoDTO':
        """Convert planning-scene object information from ROS."""
        return ObjectInfoDTO(object_id=message.object_id, attach_link=message.attach_link)

    @staticmethod
    def to_msg(dto: 'ObjectInfoDTO') -> ObjectInfo:
        """Convert planning-scene object information to ROS."""
        return ObjectInfo(object_id=dto.object_id, attach_link=dto.attach_link)
