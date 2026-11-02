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
"""Object model metadata models for the ``object_model_storage`` node.

These DTOs mirror the ``ObjectModelStorageMappingValue`` and
``ObjectModelStorageModelInfo`` ROS messages and are the shape parsed from
``local_storage.json`` (local mode) and the storage service (cloud mode).
"""

import json
from collections.abc import Iterable
from typing import Any

from pydantic import BaseModel, Field, field_validator

from repositories.enums.object_model_storage.model_type_enum import ModelTypeEnum
from repositories.msg import ObjectModelStorageMappingValue, ObjectModelStorageModelInfo
from repositories.utils.uuid_validator import parse_uuid4


class MappingValueDTO(BaseModel, frozen=True):
    """A single key/value tag attached to an object model."""

    key: str = Field(description='Tag key; matched exactly (case-sensitive).')
    value: str = Field(description='Tag value; matched exactly (case-sensitive).')


class ObjectModelInfoDTO(BaseModel, frozen=True):
    """Metadata describing one (physical) object model."""

    model_id: str = Field(
        description='UUID4 identifying the model; also the name of its cache '
        'entry in download_path.',
    )
    model_type: ModelTypeEnum = Field(
        description="File format of the model ('sdf', 'stl', 'dae' or 'obj').",
    )
    filter_keys: tuple[MappingValueDTO, ...] = Field(
        default=(),
        description='Tags used by get_models_info filtering.',
    )
    metadata_json: str = Field(
        default='',
        description='Arbitrary model metadata as a raw JSON string; empty when '
        'absent.',
    )

    @field_validator('model_id')
    @classmethod
    def _validate_model_id(cls, value: str) -> str:
        """Ensure ``model_id`` is a canonical UUID4 string.

        Args:
            value: The raw model identifier.

        Returns:
            str: The lowercase canonical UUID4 string.

        Raises:
            ValueError: If the value is not a valid UUID4.
        """
        return parse_uuid4(value, 'model_id')

    @field_validator('metadata_json', mode='before')
    @classmethod
    def _normalize_metadata_json(cls, value: Any) -> str:
        """Accept a JSON string, a JSON-compatible object, or null.

        Sources may embed metadata either as a pre-serialized JSON string or as
        a nested JSON object; both are normalized to a JSON string.

        Args:
            value: The raw metadata value.

        Returns:
            str: A JSON string, or ``''`` when no metadata is present.

        Raises:
            ValueError: If a non-empty string is not valid JSON.
        """
        if value is None:
            return ''
        if isinstance(value, str):
            if value.strip():
                try:
                    json.loads(value)
                except json.JSONDecodeError as exc:
                    raise ValueError(f'metadata_json is not valid JSON: {exc}') from exc
            return value
        return json.dumps(value)

    def matches_all(self, filters: Iterable[MappingValueDTO]) -> bool:
        """Return whether this model carries every given key/value pair.

        Args:
            filters: The required tags; an empty iterable matches every model.

        Returns:
            bool: True if each pair in ``filters`` is present in ``filter_keys``.
        """
        tags = set(self.filter_keys)
        return all(item in tags for item in filters)

    def to_msg(self) -> ObjectModelStorageModelInfo:
        """Convert this model info into its ROS message.

        Returns:
            ObjectModelStorageModelInfo: The populated message.
        """
        msg = ObjectModelStorageModelInfo()
        msg.model_id = self.model_id
        msg.model_type = self.model_type.value
        msg.filter_keys = [
            ObjectModelStorageMappingValue(key=item.key, value=item.value)
            for item in self.filter_keys
        ]
        msg.metadata_json = self.metadata_json
        return msg
