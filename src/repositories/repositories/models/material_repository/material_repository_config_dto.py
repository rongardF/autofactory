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
"""Frozen configuration model for the ``material_repository`` node.

``MaterialRepositoryConfigDTO`` captures the validated configuration read from
ROS parameters on ``configure`` and owns every validation rule. A
:class:`pydantic.ValidationError` signals an invalid/missing required value and
drives the configure transition to FAILURE.
"""

import os

from pydantic import BaseModel, Field, field_validator, model_validator

from repositories.utils.uuid_validator import parse_uuid4


class MaterialRepositoryConfigDTO(BaseModel, frozen=True):
    """Validated, immutable configuration for the ``material_repository`` node."""

    station_id: str = Field(
        description='Station scope; must be a UUID4 string.',
    )
    local_storage: bool = Field(
        description='True serves materials from the local materials file; False '
        'selects cloud (REST) mode.',
    )
    material_service_url: str = Field(
        default='',
        description='Base URL of the REST material service. Required only when '
        'local_storage is False; ignored in local mode.',
    )
    request_timeout_sec: float = Field(
        default=5.0,
        gt=0.0,
        description='HTTP timeout in seconds for cloud calls (provisional; '
        'unused until cloud mode is implemented). Must be > 0.',
    )
    materials_file_path: str = Field(
        default='/tmp/station/materials.json',
        description='Absolute path of the JSON file listing the materials; used '
        'only when local_storage is True.',
    )

    @field_validator('station_id')
    @classmethod
    def _validate_station_id(cls, value: str) -> str:
        """Ensure ``station_id`` is a canonical UUID4 string.

        Args:
            value: The raw station identifier.

        Returns:
            str: The lowercase canonical UUID4 string.

        Raises:
            ValueError: If the value is not a valid UUID4.
        """
        return parse_uuid4(value, 'station_id')

    @model_validator(mode='after')
    def _validate_mode_settings(self) -> 'MaterialRepositoryConfigDTO':
        """Require the settings of the selected mode.

        Returns:
            MaterialRepositoryConfigDTO: The validated model.

        Raises:
            ValueError: If local mode has no absolute ``materials_file_path`` or
                cloud mode has no ``material_service_url``.
        """
        if self.local_storage:
            if not self.materials_file_path or not os.path.isabs(
                self.materials_file_path
            ):
                raise ValueError(
                    'materials_file_path must be an absolute path '
                    f"(got '{self.materials_file_path}')"
                )
        elif not self.material_service_url:
            raise ValueError(
                'material_service_url is required when local_storage is false'
            )
        return self
