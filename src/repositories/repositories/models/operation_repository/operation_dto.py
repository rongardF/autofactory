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
"""Operation data models and ROS message conversion."""

from typing import Any
from uuid import UUID

from pydantic import BaseModel, Field, field_validator, model_validator, StrictStr

from repositories.enums.operation_repository import (
    OperationTypeEnum,
    SCHEMA_VERSION,
)
from repositories.models.operation_repository.argument_dto import ArgumentDTO
from repositories.models.operation_repository.dispensing_pattern_dto import (
    DispensingPatternDTO,
)
from repositories.msg import (
    OperationRepositoryEndTool,
    OperationRepositoryMaterial,
    OperationRepositoryOperation,
)


class MaterialDTO(BaseModel, frozen=True):
    """Product material identity."""

    product_number: StrictStr = Field(description='Material product number.')
    product_revision: StrictStr = Field(description='Material product revision.')

    def to_msg(self) -> OperationRepositoryMaterial:
        """Convert material identity to its ROS message."""
        message = OperationRepositoryMaterial()
        message.product_number = self.product_number
        message.product_revision = self.product_revision
        return message


class EndToolDTO(BaseModel, frozen=True):
    """End-tool product identity."""

    product_number: StrictStr = Field(description='End-tool product number.')
    product_revision: StrictStr = Field(description='End-tool product revision.')

    def to_msg(self) -> OperationRepositoryEndTool:
        """Convert end-tool identity to its ROS message."""
        message = OperationRepositoryEndTool()
        message.product_number = self.product_number
        message.product_revision = self.product_revision
        return message


class OperationDTO(BaseModel, frozen=True):
    """Validated operation payload."""

    id: UUID = Field(description='UUID4 identifying this operation.')  # noqa: A003
    operation_type: OperationTypeEnum = Field(description='Operation category.')
    operation_number: StrictStr = Field(description='Human-readable operation number.')
    description: StrictStr = Field(description='Operation description.')
    start_assembly_model_id: UUID = Field(description='UUID4 model at operation start.')
    end_assembly_model_id: UUID = Field(description='UUID4 model at operation end.')
    material: MaterialDTO = Field(description='Material used by the operation.')
    end_tool: EndToolDTO = Field(description='End tool used by the operation.')
    operation_args: list[ArgumentDTO] | None = Field(
        default=None,
        description='Named typed operation arguments, or null when empty.',
    )
    dispensing_patterns: list[DispensingPatternDTO] = Field(
        default_factory=list,
        description='Dispensing patterns for a dispensing operation.',
    )

    @field_validator('id', 'start_assembly_model_id', 'end_assembly_model_id')
    @classmethod
    def _validate_uuid4(cls, value: UUID) -> UUID:
        if value.version != 4:
            raise ValueError(f'{value} must be a UUID4')
        return value

    @model_validator(mode='before')
    @classmethod
    def _parse_operation_args(cls, data: Any) -> Any:
        if not isinstance(data, dict):
            return data
        arguments = data.get('operation_args')
        copied = dict(data)
        if arguments is None or arguments == {} or arguments == []:
            copied['operation_args'] = None
        elif isinstance(arguments, dict):
            copied['operation_args'] = [
                ArgumentDTO.from_named_value(name, value)
                for name, value in arguments.items()
            ]
        else:
            raise ValueError('operation_args must be an object or null')
        return copied

    @classmethod
    def from_operation_data(cls, data: dict[str, Any]) -> 'OperationDTO':
        """Validate schema version, dispatch patterns and parse one operation."""
        if not isinstance(data, dict):
            raise ValueError('operation data must be a JSON object')
        schema_version = data.get('schema_version', SCHEMA_VERSION)
        if schema_version != SCHEMA_VERSION:
            raise ValueError(
                f'unsupported schema_version {schema_version!r}; '
                f'expected {SCHEMA_VERSION!r}'
            )
        patterns = data.get('patterns')
        if not isinstance(patterns, list):
            raise ValueError('operation key "patterns" must be a list')
        payload = {
            key: value
            for key, value in data.items()
            if key not in {'schema_version', 'patterns'}
        }
        try:
            operation_type = OperationTypeEnum(payload.get('operation_type'))
        except (TypeError, ValueError) as error:
            raise ValueError(
                f'unknown operation_type {payload.get("operation_type")!r}'
            ) from error
        match operation_type:
            case OperationTypeEnum.DISPENSING:
                payload['dispensing_patterns'] = patterns
        return cls.model_validate(payload)

    def to_msg(self, frame_id: str) -> OperationRepositoryOperation:
        """Convert the operation DTO to its ROS message."""
        message = OperationRepositoryOperation()
        message.id = str(self.id)
        message.operation_type = self.operation_type.value
        message.operation_number = self.operation_number
        message.description = self.description
        message.start_assembly_model_id = str(self.start_assembly_model_id)
        message.end_assembly_model_id = str(self.end_assembly_model_id)
        message.material = self.material.to_msg()
        message.end_tool = self.end_tool.to_msg()
        message.operation_args = [
            argument.to_msg() for argument in self.operation_args or []
        ]
        message.dispensing_patterns = [
            pattern.to_msg(frame_id) for pattern in self.dispensing_patterns
        ]
        return message
