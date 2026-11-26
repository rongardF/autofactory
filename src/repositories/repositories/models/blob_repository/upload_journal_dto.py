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
"""Rollback journal models for batch uploads in ``blob_repository``."""

from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field

from repositories.models.blob_repository.blob_descriptor_dto import BlobDescriptorDTO
from repositories.models.common.stored_file_entry_dto import DetachedEntryDTO


class PreviousBlobDTO(BaseModel, frozen=True):
    """Content and descriptor replaced by a batch, kept until commit or rollback."""

    descriptor: BlobDescriptorDTO = Field(description='Descriptor of the replaced blob.')
    content_path: Path = Field(description='Hidden backup file holding the replaced content.')
    detached: DetachedEntryDTO | None = Field(
        default=None,
        description='Set when the backup is the detached local copy; None when the '
        'content was downloaded into a standalone backup file.',
    )


class UploadRecordDTO(BaseModel):
    """Mutable rollback journal entry for one item of an upload batch."""

    model_config = ConfigDict(validate_assignment=True)

    blob_id: str | None = Field(
        default=None, description='Blob id once known (supplied, generated or assigned).'
    )
    previous: PreviousBlobDTO | None = Field(
        default=None, description='Backup of the replaced blob, if this item overwrote one.'
    )
    remote_attempted: bool = Field(
        default=False, description='Whether the cloud write was started.'
    )
    local_written: bool = Field(
        default=False, description='Whether the local store may hold the new content.'
    )
