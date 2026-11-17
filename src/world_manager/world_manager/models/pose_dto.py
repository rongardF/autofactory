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
"""Pose data model shared by the client library and the manager."""

from pydantic import BaseModel, ConfigDict, Field

from geometry_msgs.msg import Pose, PoseStamped, Transform

from gz.msgs10.pose_pb2 import Pose as GazeboPose


class PoseDTO(BaseModel):
    """Pose of a model, expressed in the frame named by ``frame_id``."""

    model_config = ConfigDict(frozen=True)

    frame_id: str = Field(default='', description='Frame the pose is expressed in; empty means the world frame')
    x: float = Field(default=0.0, description='Position along X in meters')
    y: float = Field(default=0.0, description='Position along Y in meters')
    z: float = Field(default=0.0, description='Position along Z in meters')
    qx: float = Field(default=0.0, description='Orientation quaternion X component')
    qy: float = Field(default=0.0, description='Orientation quaternion Y component')
    qz: float = Field(default=0.0, description='Orientation quaternion Z component')
    qw: float = Field(default=1.0, description='Orientation quaternion W component')

    @staticmethod
    def from_msg(message: Pose, frame_id: str = '') -> 'PoseDTO':
        """Convert a ROS pose into a DTO in the given frame."""
        p, q = message.position, message.orientation
        return PoseDTO(
            frame_id=frame_id, x=p.x, y=p.y, z=p.z,
            qx=q.x, qy=q.y, qz=q.z, qw=q.w,
        )

    @staticmethod
    def from_stamped_msg(message: PoseStamped) -> 'PoseDTO':
        """Convert a stamped ROS pose into a DTO."""
        return PoseDTO.from_msg(message.pose, message.header.frame_id)

    @staticmethod
    def from_transform_msg(message: Transform, frame_id: str = '') -> 'PoseDTO':
        """Convert a ROS transform to a pose DTO."""
        p, q = message.translation, message.rotation
        return PoseDTO(
            frame_id=frame_id, x=p.x, y=p.y, z=p.z,
            qx=q.x, qy=q.y, qz=q.z, qw=q.w,
        )

    @staticmethod
    def to_msg(dto: 'PoseDTO') -> Pose:
        """Convert a DTO to a ROS pose."""
        result = Pose()
        result.position.x, result.position.y, result.position.z = dto.x, dto.y, dto.z
        result.orientation.x, result.orientation.y = dto.qx, dto.qy
        result.orientation.z, result.orientation.w = dto.qz, dto.qw
        return result

    @staticmethod
    def to_stamped_msg(dto: 'PoseDTO') -> PoseStamped:
        """Convert a DTO to a stamped ROS pose."""
        result = PoseStamped()
        result.header.frame_id = dto.frame_id
        result.pose = PoseDTO.to_msg(dto)
        return result

    @staticmethod
    def to_gazebo_msg(dto: 'PoseDTO') -> 'GazeboPose':
        """Convert to a Gazebo pose without requiring Gazebo for ROS clients."""
        result = GazeboPose()
        result.position.x, result.position.y, result.position.z = dto.x, dto.y, dto.z
        result.orientation.x, result.orientation.y = dto.qx, dto.qy
        result.orientation.z, result.orientation.w = dto.qz, dto.qw
        return result
