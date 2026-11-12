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
"""Frozen configuration model for the ``image_repository`` node.

``ImageRepositoryConfigDTO`` captures the validated configuration read from ROS
parameters on ``configure``. A :class:`pydantic.ValidationError` signals an
invalid value and drives the configure transition to FAILURE.
"""

from pathlib import PurePath

from pydantic import BaseModel, Field, field_validator, model_validator

from repositories.utils.uuid_validator import parse_uuid4

# Fixed headroom between the eviction threshold and the cache size limit.
CACHE_HYSTERESIS_BYTES = 10 * 1024 * 1024


class ImageRepositoryConfigDTO(BaseModel, frozen=True):
    """Validated, immutable configuration for the ``image_repository`` node."""

    local_storage: bool = Field(
        default=True,
        description='True keeps images only on the local filesystem; False uses '
        'the remote image service with the local directory as a cache.',
    )
    local_storage_path: str = Field(
        default='/tmp/station/images',
        min_length=1,
        description='Absolute path of the directory dedicated to image and '
        'metadata files.',
    )
    station_id: str = Field(
        description='Station identifier used to namespace remote calls; must be '
        'a canonical UUID4 string.',
    )
    image_service_url: str = Field(
        default='',
        description='Base URL of the remote image service. Required only when '
        'local_storage is False.',
    )
    request_timeout_sec: float = Field(
        default=5.0,
        gt=0.0,
        description='HTTP timeout in seconds for remote calls.',
    )
    cache_size_mb: int = Field(
        default=50,
        gt=10,
        description='Upper bound in MB for the total size of the local '
        'directory; eviction starts 10 MB below it.',
    )
    cache_check_interval_sec: float = Field(
        default=60.0,
        gt=0.0,
        description='Period in seconds of the background cache size check.',
    )

    @field_validator('local_storage_path')
    @classmethod
    def _validate_path(cls, value: str) -> str:
        """Require an absolute path.

        Args:
            value: The raw path.

        Returns:
            str: The validated path, unchanged.

        Raises:
            ValueError: If the path is not absolute.
        """
        if not PurePath(value).is_absolute():
            raise ValueError(f"local_storage_path must be absolute (got '{value}')")
        return value

    @field_validator('station_id')
    @classmethod
    def _validate_station_id(cls, value: str) -> str:
        """Require a canonical UUID4 station identifier.

        Args:
            value: The raw station identifier.

        Returns:
            str: The validated identifier, unchanged.

        Raises:
            ValueError: If the value is not a canonical UUID4.
        """
        parse_uuid4(value, 'station_id')
        return value

    @model_validator(mode='after')
    def _validate_remote_url(self) -> 'ImageRepositoryConfigDTO':
        """Require ``image_service_url`` whenever remote mode is selected.

        Returns:
            ImageRepositoryConfigDTO: The validated model.

        Raises:
            ValueError: If ``local_storage`` is False but no URL is configured.
        """
        if not self.local_storage and not self.image_service_url:
            raise ValueError('image_service_url is required when local_storage is false')
        return self

    @property
    def limit_bytes(self) -> int:
        """Upper bound for the local directory size in bytes."""
        return self.cache_size_mb * 1024 * 1024

    @property
    def threshold_bytes(self) -> int:
        """Size in bytes above which eviction starts."""
        return self.limit_bytes - CACHE_HYSTERESIS_BYTES
