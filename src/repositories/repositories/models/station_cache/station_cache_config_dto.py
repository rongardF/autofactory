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
"""Frozen configuration model for the ``station_cache`` node.

``StationCacheConfigDTO`` captures the full, validated configuration read from
ROS parameters on ``configure`` and owns every validation rule. The node builds
one instance from its declared parameters; a :class:`pydantic.ValidationError`
signals an invalid/missing required value and drives the configure transition to
FAILURE.
"""

import uuid

from pydantic import BaseModel, Field, field_validator, model_validator


class StationCacheConfigDTO(BaseModel, frozen=True):
    """Validated, immutable configuration for the ``station_cache`` node."""

    local_cache: bool = Field(
        description='True selects the in-memory LocalCacheBackend; False selects '
        'the CloudCacheBackend (REST, currently stubbed).',
    )
    station_id: str = Field(
        description='Required scope for all keys; must be a UUID4 string.',
    )
    cache_service_url: str = Field(
        default='',
        description='Base URL of the REST cache service. Required only when '
        'local_cache is False; ignored in local mode.',
    )
    expiry_sweep_interval_sec: float = Field(
        gt=0.0,
        description='Period in seconds of the background expiry sweep timer '
        '(local mode). Must be > 0.',
    )
    request_timeout_sec: float = Field(
        default=5.0,
        description='HTTP timeout in seconds for cloud backend calls '
        '(provisional; unused until cloud mode is implemented).',
    )

    @field_validator('station_id')
    @classmethod
    def _validate_station_id(cls, value: str) -> str:
        """Ensure ``station_id`` is a non-empty, valid UUID4 string.

        Args:
            value: The raw station identifier.

        Returns:
            str: The validated identifier, unchanged.

        Raises:
            ValueError: If the value is empty or not a valid UUID4.
        """
        try:
            parsed = uuid.UUID(value)
        except ValueError as exc:
            raise ValueError(
                f"station_id must be a valid UUID4 string (got '{value}')"
            ) from exc
        if parsed.version != 4:
            raise ValueError(
                f"station_id must be a UUID4 string (got UUID version "
                f'{parsed.version})'
            )
        return value

    @model_validator(mode='after')
    def _validate_cloud_url(self) -> 'StationCacheConfigDTO':
        """Require ``cache_service_url`` whenever cloud mode is selected.

        Returns:
            StationCacheConfigDTO: The validated model.

        Raises:
            ValueError: If ``local_cache`` is False but no URL is configured.
        """
        if not self.local_cache and not self.cache_service_url:
            raise ValueError(
                'cache_service_url is required when local_cache is false'
            )
        return self
