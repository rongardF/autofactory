# Copyright (c) 2026, Autofactory
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS WARRANTIES ARE DISCLAIMED.
"""Pydantic models for dispensing pattern payloads and ROS conversion."""

from typing import Any
from uuid import UUID

from pydantic import (
    BaseModel,
    Field,
    field_validator,
    model_validator,
    StrictFloat,
)

from repositories.enums.operation_repository import (
    ArgumentTypeEnum,
    CircTypeEnum,
    MotionTypeEnum,
    PatternActionTimingEnum,
    PatternActionTypeEnum,
)
from repositories.models.operation_repository.argument_dto import ArgumentDTO
from repositories.msg import (
    OperationRepositoryDispensingMovement,
    OperationRepositoryDispensingPattern,
    OperationRepositoryPatternAction,
)
from repositories.utils.pose_converter import point3_to_point, pose6_to_pose_stamped

Pose6 = tuple[StrictFloat, StrictFloat, StrictFloat, StrictFloat, StrictFloat, StrictFloat]
Point3 = tuple[StrictFloat, StrictFloat, StrictFloat]


class PatternActionDTO(BaseModel, frozen=True):
    """Validated dispensing action."""

    type: PatternActionTypeEnum = Field(description='Action kind.')  # noqa: A003
    timing: PatternActionTimingEnum = Field(description='Movement boundary for the action.')
    action_args: list[ArgumentDTO] | None = Field(
        default=None,
        description='Named typed arguments supplied for this action, or null when empty.',
    )

    @model_validator(mode='before')
    @classmethod
    def _parse_action_args(cls, data: Any) -> Any:
        if not isinstance(data, dict):
            return data
        arguments = data.get('action_args')
        copied = dict(data)
        if arguments is None or arguments == {} or arguments == []:
            copied['action_args'] = None
        elif isinstance(arguments, dict):
            copied['action_args'] = [
                ArgumentDTO.from_named_value(name, value)
                for name, value in arguments.items()
            ]
        else:
            raise ValueError('action_args must be an object or null')
        return copied

    @model_validator(mode='after')
    def _validate_wait_action(self) -> 'PatternActionDTO':
        if self.type == PatternActionTypeEnum.WAIT:
            wait_argument = next(
                (
                    arg
                    for arg in self.action_args or []
                    if arg.argument_name == 'wait_time_s'
                ),
                None,
            )
            if (
                wait_argument is None
                or wait_argument.argument_type != ArgumentTypeEnum.FLOAT
                or wait_argument.argument_value_float is None
                or wait_argument.argument_value_float < 0
            ):
                raise ValueError('wait action requires action_args.wait_time_s >= 0')
        return self

    def to_msg(self) -> OperationRepositoryPatternAction:
        """Convert action and its arguments to the ROS message."""
        message = OperationRepositoryPatternAction()
        message.type = self.type.value
        message.timing = self.timing.value
        message.action_args = [
            argument.to_msg() for argument in self.action_args or []
        ]
        return message


class DispensingMovementDTO(BaseModel, frozen=True):
    """Validated dispensing movement."""

    id: UUID = Field(description='UUID4 identifying this movement.')  # noqa: A003
    motion_type: MotionTypeEnum = Field(description='Movement planning mode.')
    target_pose_m: Pose6 = Field(description='XYZ position and fixed-axis RPY target.')
    blend_radius_m: StrictFloat | None = Field(
        description='Blend radius, or null for default.'
    )
    cartesian_speed_m_s: StrictFloat | None = Field(
        description='Cartesian speed, or null for default.'
    )
    joint_speed_rad_s: StrictFloat | None = Field(
        description='Joint speed, or null for default.'
    )
    circ_type: CircTypeEnum | None = Field(description='Circular point interpretation, or null.')
    circ_point_m: Point3 | None = Field(description='Circular point in meters, or null.')
    actions: list[PatternActionDTO] = Field(description='Actions associated with this movement.')

    @field_validator('id')
    @classmethod
    def _validate_uuid4(cls, value: UUID) -> UUID:
        if value.version != 4:
            raise ValueError(f'{value} must be a UUID4')
        return value

    def to_msg(self, frame_id: str) -> OperationRepositoryDispensingMovement:
        """Convert movement fields into the flat ROS message."""
        message = OperationRepositoryDispensingMovement()
        message.id = str(self.id)
        message.motion_type = self.motion_type.value
        message.target_pose = pose6_to_pose_stamped(self.target_pose_m, frame_id)
        message.blend_radius_m = -1.0 if self.blend_radius_m is None else self.blend_radius_m
        message.cartesian_speed_m_s = (
            -1.0 if self.cartesian_speed_m_s is None else self.cartesian_speed_m_s
        )
        message.joint_speed_rad_s = (
            -1.0 if self.joint_speed_rad_s is None else self.joint_speed_rad_s
        )
        message.circ_type = '' if self.circ_type is None else self.circ_type.value
        message.circ_point = (
            [] if self.circ_point_m is None else [point3_to_point(self.circ_point_m)]
        )
        message.actions = [action.to_msg() for action in self.actions]
        return message


class DispensingPatternDTO(BaseModel, frozen=True):
    """Validated dispensing pattern and its associated movements."""

    id: UUID = Field(description='UUID4 identifying this pattern.')  # noqa: A003
    start_pose_m: Pose6 = Field(description='XYZ position and fixed-axis RPY start pose.')
    pattern_args: list[ArgumentDTO] | None = Field(
        default=None,
        description='Named typed pattern arguments, or null when empty.',
    )
    movements: list[DispensingMovementDTO] = Field(
        description='Ordered dispensing movements.'
    )

    @field_validator('id')
    @classmethod
    def _validate_uuid4(cls, value: UUID) -> UUID:
        if value.version != 4:
            raise ValueError(f'{value} must be a UUID4')
        return value

    @model_validator(mode='before')
    @classmethod
    def _parse_pattern_args(cls, data: Any) -> Any:
        if not isinstance(data, dict):
            return data
        pattern_args = data.get('pattern_args')
        copied = dict(data)
        if pattern_args is None or pattern_args == {} or pattern_args == []:
            copied['pattern_args'] = None
        elif isinstance(pattern_args, dict):
            copied['pattern_args'] = [
                ArgumentDTO.from_named_value(name, value)
                for name, value in pattern_args.items()
            ]
        else:
            raise ValueError('pattern_args must be an object or null')
        return copied

    @model_validator(mode='after')
    def _validate_flowrate_argument(self) -> 'DispensingPatternDTO':
        flowrate = next(
            (
                argument
                for argument in self.pattern_args or []
                if argument.argument_name == 'flowrate_m3_s'
            ),
            None,
        )
        if (
            flowrate is None
            or flowrate.argument_value_float is None
            or flowrate.argument_type != ArgumentTypeEnum.FLOAT
        ):
            raise ValueError('pattern_args.flowrate_m3_s must be a float')
        return self

    def to_msg(self, frame_id: str) -> OperationRepositoryDispensingPattern:
        """Convert the pattern and all movements to the ROS message."""
        message = OperationRepositoryDispensingPattern()
        message.id = str(self.id)
        message.start_pose = pose6_to_pose_stamped(self.start_pose_m, frame_id)
        message.pattern_args = [
            argument.to_msg() for argument in self.pattern_args or []
        ]
        message.movements = [movement.to_msg(frame_id) for movement in self.movements]
        return message
