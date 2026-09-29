# Copyright (c) 2026, Movement Controller Contributors
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
"""TrajectoryPathDTO — Pydantic DTO mirroring TrajectoryPath.msg."""

from __future__ import annotations

import re

from geometry_msgs.msg import Point, PoseStamped
from pydantic import BaseModel, ConfigDict, Field, field_validator

from movement_controller.enums.circ_type_enum import CircTypeEnum
from movement_controller.enums.motion_type_enum import MotionTypeEnum

from movement_controller.msg import TrajectoryPath

_UUID4_RE = re.compile(
    r'^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$',
    re.IGNORECASE,
)


class TrajectoryPathDTO(BaseModel):
    """Validated, immutable representation of a single trajectory path."""

    model_config = ConfigDict(arbitrary_types_allowed=True, frozen=True)

    path_id: str = Field(description='UUID4 path identifier')
    motion_type: MotionTypeEnum = Field(description='Motion type: LIN, PTP, or CIRC')
    target_pose: PoseStamped = Field(description='Target end-effector pose with frame_id')
    blend_radius: float = Field(
        default=0.0,
        description='Blend radius in metres; negative values normalised to 0.0',
    )
    cartesian_speed: float = Field(
        default=0.0,
        description='End-effector cartesian speed in m/s',
    )
    cartesian_acceleration: float = Field(
        default=0.0,
        description='End-effector cartesian acceleration in m/s²',
    )
    joint_speed: float = Field(
        default=0.0,
        description='End-effector joint speed in rad/s',
    )
    joint_acceleration: float = Field(
        default=0.0,
        description='End-effector joint acceleration in rad/s²',
    )
    tool_frame: str = Field(
        default='',
        description='Tool frame override; empty string means use tool0',
    )
    circ_type: CircTypeEnum = Field(
        default=CircTypeEnum.INTERIM,
        description='CIRC arc reference type: interim (waypoint on arc) or center (arc center point)',
    )
    circ_point: Point = Field(
        default_factory=Point,
        description='CIRC arc reference point; ignored for LIN/PTP',
    )

    @field_validator('path_id', mode='before')
    @classmethod
    def validate_path_id(cls, v: str) -> str:
        """Validate that ``path_id`` is a non-empty UUID4 string.

        :param v: Raw value for ``path_id`` before Pydantic coercion.
        :type v: str
        :returns: The validated ``path_id`` string, unchanged.
        :rtype: str
        :raises ValueError: If ``path_id`` is empty or does not match the UUID4
            pattern ``^[0-9a-f]{8}-...-4...-[89ab]...$``.
        """
        if not v:
            raise ValueError('path_id must be non-empty')
        if not _UUID4_RE.match(v):
            raise ValueError(f'path_id must be a valid UUID4, got {v!r}')
        return v

    @field_validator('blend_radius', mode='before')
    @classmethod
    def normalise_blend_radius(cls, v: object) -> float:
        """Coerce negative ``blend_radius`` values to ``0.0``.

        :param v: Raw value for ``blend_radius`` before Pydantic coercion.
        :returns: ``0.0`` if the value is negative, otherwise the float value.
        :rtype: float
        :raises ValueError: If the value cannot be converted to a :class:`float`.
        """
        try:
            f = float(v)  # type: ignore[arg-type]
        except (TypeError, ValueError):
            raise ValueError(f'blend_radius must be a number, got {v!r}')
        return 0.0 if f < 0 else f

    @classmethod
    def from_ros_msg(cls, ros_msg: TrajectoryPath) -> TrajectoryPathDTO:
        """Construct a :class:`TrajectoryPathDTO` from a :class:`~movement_controller.msg.TrajectoryPath` ROS 2 message.

        Resolves ``motion_type`` and ``circ_type`` string values to their
        respective :class:`~movement_controller.enums.MotionTypeEnum` and
        :class:`~movement_controller.enums.CircTypeEnum` members.

        :param ros_msg: ROS 2 message with all trajectory path fields populated.
        :type ros_msg: TrajectoryPath
        :returns: Validated, immutable :class:`TrajectoryPathDTO`.
        :rtype: TrajectoryPathDTO
        :raises ValueError: If ``motion_type`` is not a recognised
            :class:`~movement_controller.enums.MotionTypeEnum` value, or if a
            CIRC path has an absent or unrecognised ``circ_type``.
        """

        try:
            motion_type = MotionTypeEnum(ros_msg.motion_type)
        except ValueError:
            raise ValueError(f'Invalid motion_type: {ros_msg.motion_type!r}')

        if motion_type == MotionTypeEnum.CIRC:
            if not ros_msg.circ_type:
                raise ValueError(
                    f'path {ros_msg.path_id!r} has motion_type CIRC but circ_type is empty; '
                    f'must be "interim" or "center"'
                )
            try:
                circ_type = CircTypeEnum(ros_msg.circ_type)
            except ValueError:
                raise ValueError(
                    f'path {ros_msg.path_id!r} has motion_type CIRC but invalid circ_type '
                    f'{ros_msg.circ_type!r}; must be "interim" or "center"'
                )
        else:
            if ros_msg.circ_type:
                try:
                    circ_type = CircTypeEnum(ros_msg.circ_type)
                except ValueError:
                    circ_type = CircTypeEnum.INTERIM  # circ_type is irrelevant for non-CIRC paths
            else:
                circ_type = CircTypeEnum.INTERIM

        return cls(
            path_id=ros_msg.path_id,
            motion_type=motion_type,
            target_pose=ros_msg.target_pose,
            blend_radius=ros_msg.blend_radius,
            cartesian_speed=ros_msg.cartesian_speed,
            cartesian_acceleration=ros_msg.cartesian_acceleration,
            joint_speed=ros_msg.joint_speed,
            joint_acceleration=ros_msg.joint_acceleration,
            tool_frame=ros_msg.tool_frame,
            circ_type=circ_type,
            circ_point=ros_msg.circ_point,
        )
