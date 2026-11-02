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
"""Frozen configuration model for the ``object_model_storage`` node.

``ObjectModelStorageConfigDTO`` captures the validated configuration read from
ROS parameters on ``configure`` and owns every validation rule. A
:class:`pydantic.ValidationError` signals an invalid/missing required value and
drives the configure transition to FAILURE.
"""

import os

from pydantic import BaseModel, Field, field_validator, model_validator

from repositories.utils.uuid_validator import parse_uuid4


class ObjectModelStorageConfigDTO(BaseModel, frozen=True):
    """Validated, immutable configuration for the ``object_model_storage`` node."""

    local_storage: bool = Field(
        description='True selects local-disk mode (metadata from '
        'local_storage.json, no network); False selects cloud (REST) mode.',
    )
    download_path: str = Field(
        description='Absolute path of the directory that holds cached model '
        'files/folders (and local_storage.json in local mode).',
    )
    station_id: str = Field(
        description='Station scope used in cloud REST paths; must be a UUID4 '
        'string.',
    )
    storage_service_url: str = Field(
        default='',
        description='Base URL of the REST model storage service. Required only '
        'when local_storage is False; ignored in local mode.',
    )
    request_timeout_sec: float = Field(
        default=5.0,
        gt=0.0,
        description='HTTP timeout in seconds for cloud calls (provisional; '
        'unused until cloud mode is implemented). Must be > 0.',
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

    @field_validator('download_path')
    @classmethod
    def _validate_download_path(cls, value: str) -> str:
        """Ensure ``download_path`` is a non-empty absolute path.

        Args:
            value: The raw download directory path.

        Returns:
            str: The normalized absolute path.

        Raises:
            ValueError: If the path is empty or relative.
        """
        if not value or not os.path.isabs(value):
            raise ValueError(
                f"download_path must be an absolute path (got '{value}')"
            )
        return os.path.normpath(value)

    @model_validator(mode='after')
    def _validate_cloud_url(self) -> 'ObjectModelStorageConfigDTO':
        """Require ``storage_service_url`` whenever cloud mode is selected.

        Returns:
            ObjectModelStorageConfigDTO: The validated model.

        Raises:
            ValueError: If ``local_storage`` is False but no URL is configured.
        """
        if not self.local_storage and not self.storage_service_url:
            raise ValueError(
                'storage_service_url is required when local_storage is false'
            )
        return self
