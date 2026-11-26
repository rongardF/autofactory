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
"""Blob descriptor and request item models used by ``blob_repository``."""

from pathlib import Path
import re

from pydantic import AwareDatetime, BaseModel, Field, field_validator, JsonValue

from repositories.utils.json_metadata import parse_metadata
from repositories.utils.uuid_validator import parse_uuid4

DEFAULT_MEDIA_TYPE = 'application/octet-stream'

_TOKEN = r'[A-Za-z0-9!#$&^_.+-]+'
_MEDIA_TYPE_RE = re.compile(
    rf'^{_TOKEN}/{_TOKEN}(\s*;\s*{_TOKEN}=("[^"\x00-\x1f\x7f]*"|{_TOKEN}))*$'
)


def validate_media_type(value: str) -> str:
    """Require a ``type/subtype`` MIME type with optional parameters.

    Raises:
        ValueError: If ``value`` is not a well-formed media type.
    """
    if len(value) > 255 or not _MEDIA_TYPE_RE.match(value):
        raise ValueError('media_type must be a MIME type such as "application/json"')
    return value


class BlobUploadDescriptorDTO(BaseModel, frozen=True):
    """Descriptor sent with uploaded content (the ``descriptor`` multipart part)."""

    blob_name: str = Field(default='', description='Informational display name.')
    media_type: str = Field(default=DEFAULT_MEDIA_TYPE, description='MIME type.')
    size_bytes: int = Field(ge=0, description='Content length in bytes.')
    sha256: str = Field(
        pattern=r'^[0-9a-f]{64}$', description='Lowercase hex SHA-256 of the content.'
    )
    metadata: JsonValue = Field(
        default=None, description='Optional user JSON metadata; None means none.'
    )


class BlobDescriptorDTO(BlobUploadDescriptorDTO, frozen=True):
    """Stored descriptor of a blob (sidecar file and cloud ``info`` response)."""

    blob_id: str = Field(description='Canonical lowercase UUID4 of the blob.')
    created_at: AwareDatetime = Field(description='Creation time (timezone-aware).')
    updated_at: AwareDatetime = Field(description='Last replacement time (timezone-aware).')

    @field_validator('blob_id')
    @classmethod
    def _validate_blob_id(cls, value: str) -> str:
        return parse_uuid4(value, 'blob_id')

    def upload_descriptor(self) -> BlobUploadDescriptorDTO:
        """Return the upload descriptor that recreates this blob."""
        return BlobUploadDescriptorDTO(
            blob_name=self.blob_name,
            media_type=self.media_type,
            size_bytes=self.size_bytes,
            sha256=self.sha256,
            metadata=self.metadata,
        )

    def sidecar_bytes(self) -> bytes:
        """Serialize the ``<blob_id>_metadata.json`` sidecar content."""
        return self.model_dump_json().encode('utf-8')


class BlobUploadItemDTO(BaseModel, frozen=True):
    """One validated upload request item."""

    source_path: str = Field(description='Raw absolute source path from the request.')
    blob_id: str | None = Field(
        default=None, description='Canonical UUID4 to upsert, or None for a new blob.'
    )
    blob_name: str = Field(default='', description='Display name; empty uses the basename.')
    media_type: str = Field(default='', description='MIME type; empty means infer.')
    metadata: JsonValue = Field(default=None, description='Parsed user JSON metadata.')

    @classmethod
    def from_request(
        cls,
        source_path: str,
        blob_id: str = '',
        blob_name: str = '',
        media_type: str = '',
        metadata: str = '',
    ) -> 'BlobUploadItemDTO':
        """Validate raw ROS request fields.

        Raises:
            ValueError: If the id, media type or metadata is malformed.
        """
        if not source_path:
            raise ValueError('source_path is required')
        if media_type:
            validate_media_type(media_type)
        return cls(
            source_path=source_path,
            blob_id=parse_uuid4(blob_id, 'blob_id') if blob_id else None,
            blob_name=blob_name,
            media_type=media_type,
            metadata=parse_metadata(metadata),
        )


class BlobFetchResultDTO(BaseModel, frozen=True):
    """Outcome of fetching one blob."""

    local_path: Path = Field(description='Path where the blob content is available.')
    descriptor: BlobDescriptorDTO = Field(description='Descriptor of the fetched blob.')
