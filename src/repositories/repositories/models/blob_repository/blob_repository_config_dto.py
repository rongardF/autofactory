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
"""Frozen configuration model for the ``blob_repository`` node.

``BlobRepositoryConfigDTO`` captures the validated configuration read from ROS
parameters on ``configure``. A :class:`pydantic.ValidationError` signals an
invalid value and drives the configure transition to FAILURE.
"""

from pathlib import PurePath
from urllib.parse import urlsplit

from pydantic import BaseModel, Field, field_validator, model_validator

from repositories.utils.uuid_validator import parse_uuid4

_MIB = 1024 * 1024


class BlobRepositoryConfigDTO(BaseModel, frozen=True):
    """Validated, immutable configuration for the ``blob_repository`` node."""

    local_storage: bool = Field(
        default=True,
        description='True keeps blobs only on the local filesystem; False uses the '
        'cloud blob service with the local directory as a cache.',
    )
    local_storage_path: str = Field(
        default='/tmp/station/blobs',
        min_length=1,
        description='Absolute path of the directory dedicated to blob and '
        'metadata files.',
    )
    station_id: str = Field(
        description='Station identifier used to namespace cloud calls; must be '
        'a canonical UUID4 string.',
    )
    blob_service_url: str = Field(
        default='',
        description='HTTPS base URL of the cloud blob service. Required only when '
        'local_storage is False.',
    )
    request_timeout_sec: float = Field(
        default=5.0,
        gt=0.0,
        description='Timeout in seconds for connecting and for metadata requests.',
    )
    transfer_timeout_sec: float = Field(
        default=600.0,
        gt=0.0,
        description='Total timeout in seconds for streaming one file.',
    )
    cache_size_mb: int = Field(
        default=3072,
        gt=0,
        description='Upper bound in MiB for the total size of the local directory.',
    )
    max_blob_size_mb: int = Field(
        default=1024,
        gt=0,
        description='Upper bound in MiB for the size of one blob.',
    )
    cache_check_interval_sec: float = Field(
        default=60.0,
        gt=0.0,
        description='Period in seconds of the background cache size check.',
    )
    allowed_path_roots: tuple[str, ...] = Field(
        min_length=1,
        description='Absolute directories that upload sources and explicit fetch '
        'destinations must resolve within.',
    )

    @field_validator('local_storage_path')
    @classmethod
    def _validate_storage_path(cls, value: str) -> str:
        """Require an absolute storage path.

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

        Raises:
            ValueError: If the value is not a canonical UUID4.
        """
        parse_uuid4(value, 'station_id')
        return value

    @field_validator('allowed_path_roots')
    @classmethod
    def _validate_roots(cls, value: tuple[str, ...]) -> tuple[str, ...]:
        """Require every allowed root to be a non-empty absolute path.

        Raises:
            ValueError: If a root is empty or relative.
        """
        for root in value:
            if not root or not PurePath(root).is_absolute():
                raise ValueError(f"allowed_path_roots entries must be absolute (got '{root}')")
        return value

    @model_validator(mode='after')
    def _validate_relations(self) -> 'BlobRepositoryConfigDTO':
        """Check the size relationship and the cloud URL.

        Raises:
            ValueError: If ``max_blob_size_mb`` exceeds ``cache_size_mb`` or the
                cloud mode URL is missing or not an HTTPS URL.
        """
        if self.max_blob_size_mb > self.cache_size_mb:
            raise ValueError(
                f'max_blob_size_mb ({self.max_blob_size_mb}) must not exceed '
                f'cache_size_mb ({self.cache_size_mb})'
            )
        if not self.local_storage:
            if not self.blob_service_url:
                raise ValueError('blob_service_url is required when local_storage is false')
            try:
                parts = urlsplit(self.blob_service_url)
                hostname = parts.hostname
            except ValueError as exc:
                raise ValueError('blob_service_url is not a valid URL') from exc
            # The URL is never echoed: it could embed credentials.
            if parts.query or parts.fragment or parts.username or parts.password:
                raise ValueError(
                    'blob_service_url must not contain credentials, a query or a fragment'
                )
            if parts.scheme != 'https' or not hostname:
                raise ValueError(
                    'blob_service_url must be an https:// URL with a host '
                    f"(got scheme '{parts.scheme}')"
                )
        return self

    @property
    def cache_size_bytes(self) -> int:
        """Upper bound for the local directory size in bytes."""
        return self.cache_size_mb * _MIB

    @property
    def max_blob_size_bytes(self) -> int:
        """Upper bound for the size of one blob in bytes."""
        return self.max_blob_size_mb * _MIB
