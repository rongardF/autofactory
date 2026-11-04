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
"""Immutable, validated configuration for the operation repository node."""

import os

from pydantic import (
    BaseModel,
    Field,
    field_validator,
    model_validator,
    ValidationInfo,
)

from repositories.utils.uuid_validator import parse_uuid4


class OperationRepositoryConfigDTO(BaseModel, frozen=True):
    """Configuration snapshot captured on lifecycle configure."""

    local_repository: bool = Field(description='Use the local assembly file when true.')
    assembly_file_path: str = Field(description='Absolute path to the local assembly JSON file.')
    station_id: str = Field(default='', description='Cloud station UUID4; required in cloud mode.')
    operation_repository_url: str = Field(
        default='',
        description='Cloud operation repository REST base URL; required in cloud mode.',
    )
    product_origin_frame_id: str = Field(description='Frame id assigned to converted poses.')

    @field_validator('station_id')
    @classmethod
    def _validate_station_id(cls, value: str, info: ValidationInfo) -> str:
        if info.data.get('local_repository', True) or not value:
            return value
        return parse_uuid4(value, 'station_id')

    @field_validator('assembly_file_path')
    @classmethod
    def _validate_assembly_file_path(cls, value: str) -> str:
        if not value or not os.path.isabs(value):
            raise ValueError(
                f"assembly_file_path must be an absolute path (got '{value}')"
            )
        return os.path.normpath(value)

    @field_validator('product_origin_frame_id')
    @classmethod
    def _validate_frame_id(cls, value: str) -> str:
        if not value.strip():
            raise ValueError('product_origin_frame_id must not be empty')
        return value

    @model_validator(mode='after')
    def _validate_cloud_config(self) -> 'OperationRepositoryConfigDTO':
        if not self.local_repository:
            parse_uuid4(self.station_id, 'station_id')
            if not self.operation_repository_url:
                raise ValueError(
                    'operation_repository_url is required when local_repository is false'
                )
        return self
