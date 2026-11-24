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
"""Data models describing models in the Gazebo world."""

from pydantic import BaseModel, ConfigDict, Field

from world_manager.enums import JointStateEnum
from world_manager.models.pose_dto import PoseDTO
from world_manager.msg import JointInfo, ModelInfo, ModelSpec
from world_manager.srv import DeleteModels


class ModelSpecDTO(BaseModel):
    """Request to spawn one model."""

    model_config = ConfigDict(frozen=True)

    model_id: str = Field(min_length=1, description='Unique world model name; must equal the <model name> in the SDF')
    sdf: str = Field(min_length=1, description='Expanded SDF document containing exactly one model')
    pose: PoseDTO = Field(default_factory=PoseDTO, description='Spawn pose; frame_id is resolved through TF')
    attach_link: str = Field(
        default='',
        description='Empty keeps the joints as authored; otherwise only the joint holding this child link stays attached')
    delete_with_child: bool = Field(
        default=False,
        description='Delete this model (detached first) whenever a model that one of its joints holds is deleted, '
                    'whatever the owner of either model')

    @staticmethod
    def from_msg(message: ModelSpec) -> 'ModelSpecDTO':
        """Convert a spawn specification message."""
        return ModelSpecDTO(
            model_id=message.model_id, sdf=message.sdf,
            pose=PoseDTO.from_stamped_msg(message.pose),
            attach_link=message.attach_link,
            delete_with_child=message.delete_with_child,
        )

    @staticmethod
    def to_msg(dto: 'ModelSpecDTO') -> ModelSpec:
        """Convert a spawn specification to a message."""
        message = ModelSpec()
        message.model_id, message.sdf = dto.model_id, dto.sdf
        message.attach_link = dto.attach_link
        message.delete_with_child = dto.delete_with_child
        message.pose = PoseDTO.to_stamped_msg(dto.pose)
        return message


class DetachableJointDTO(BaseModel):
    """DetachableJoint plugin configuration found in an SDF, with defaults resolved."""

    model_config = ConfigDict(frozen=True)

    link_name: str = Field(description='child_link of the joint')
    child_model: str = Field(description='child_model of the joint')
    attach_topic: str = Field(description='Resolved gz topic that attaches the joint')
    detach_topic: str = Field(description='Resolved gz topic that detaches the joint')
    state_topic: str = Field(description='Resolved gz topic that reports the joint state')


class SdfModelDTO(BaseModel):
    """Result of inspecting an SDF document."""

    model_config = ConfigDict(frozen=True)

    model_name: str = Field(description='Name of the single model in the SDF')
    joints: tuple[DetachableJointDTO, ...] = Field(default=(), description='DetachableJoint plugins of the model')


class JointInfoDTO(BaseModel):
    """Observable state of one joint of a registered model."""

    model_config = ConfigDict(frozen=True)

    link_name: str = Field(description='child_link of the joint')
    child_model: str = Field(description='child_model of the joint')
    state: JointStateEnum = Field(default=JointStateEnum.UNKNOWN, description='Attachment state of the joint')
    state_confirmed: bool = Field(default=False, description='False when the state is an assumption, not an observed event')
    controllable: bool = Field(default=True, description='False when the joint topics are not unique among all joints')

    @staticmethod
    def from_msg(message: JointInfo) -> 'JointInfoDTO':
        """Convert joint information from ROS."""
        return JointInfoDTO(
            link_name=message.link_name, child_model=message.child_model,
            state=JointStateEnum(message.state), state_confirmed=message.state_confirmed,
            controllable=message.controllable,
        )

    @staticmethod
    def to_msg(dto: 'JointInfoDTO') -> JointInfo:
        """Convert joint information to ROS."""
        return JointInfo(
            link_name=dto.link_name, child_model=dto.child_model,
            state=dto.state.value, state_confirmed=dto.state_confirmed,
            controllable=dto.controllable,
        )


class ModelInfoDTO(BaseModel):
    """A registered model with its joints."""

    model_config = ConfigDict(frozen=True)

    model_id: str = Field(description='World model name')
    owner: str = Field(default='', description='Name of the client that spawned the model')
    joints: tuple[JointInfoDTO, ...] = Field(default=(), description='Joints of the model')

    @staticmethod
    def from_msg(message: ModelInfo) -> 'ModelInfoDTO':
        """Convert registered model information from ROS."""
        return ModelInfoDTO(
            model_id=message.model_id, owner=message.owner,
            joints=tuple(JointInfoDTO.from_msg(joint) for joint in message.joints),
        )

    @staticmethod
    def to_msg(dto: 'ModelInfoDTO') -> ModelInfo:
        """Convert registered model information to ROS."""
        return ModelInfo(
            model_id=dto.model_id, owner=dto.owner,
            joints=[JointInfoDTO.to_msg(joint) for joint in dto.joints],
        )


class DeleteResultDTO(BaseModel):
    """Outcome of a delete call."""

    model_config = ConfigDict(frozen=True)

    deleted_ids: tuple[str, ...] = Field(default=(), description='Ids that were removed from the world')
    unknown_ids: tuple[str, ...] = Field(default=(), description='Requested ids that are not registered')

    @staticmethod
    def from_msg(message: DeleteModels.Response) -> 'DeleteResultDTO':
        """Convert a successful delete response."""
        return DeleteResultDTO(deleted_ids=tuple(message.deleted_ids), unknown_ids=tuple(message.unknown_ids))
