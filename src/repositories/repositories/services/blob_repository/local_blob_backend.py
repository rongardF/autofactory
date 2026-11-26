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
"""Local-mode ``BlobBackend``: the local store is the source of truth."""

from datetime import datetime, timezone
from pathlib import Path
from typing import BinaryIO
import uuid

from repositories.exceptions.blob_repository import BlobNotFoundError, BlobRepositoryError
from repositories.interfaces.blob_repository.blob_backend import BlobBackend
from repositories.models.blob_repository.blob_descriptor_dto import (
    BlobDescriptorDTO,
    BlobUploadDescriptorDTO,
)
from repositories.services.blob_repository.local_blob_store import LocalBlobStore
from repositories.services.blob_repository.streaming import copy_stream


class LocalBlobBackend(BlobBackend):
    """Stores blobs directly in the node's :class:`LocalBlobStore`."""

    def __init__(self, store: LocalBlobStore) -> None:
        """Wrap the local store."""
        self._store = store

    @property
    def is_local(self) -> bool:
        """Content lives in the local store itself."""
        return True

    def initialize(self) -> None:
        """Nothing to acquire."""

    def shutdown(self) -> None:
        """Nothing to release."""

    def upload(
        self,
        content_path: Path,
        descriptor: BlobUploadDescriptorDTO,
        blob_id: str | None = None,
        replaced: BlobDescriptorDTO | None = None,
    ) -> BlobDescriptorDTO:
        """Move the staged content into the store under ``blob_id`` (or a new id)."""
        now = datetime.now(timezone.utc)
        stored = BlobDescriptorDTO(
            **descriptor.model_dump(),
            blob_id=blob_id or str(uuid.uuid4()),
            created_at=replaced.created_at if replaced is not None else now,
            updated_at=now,
        )
        self._store.commit(content_path, stored)
        return stored

    def download(self, blob_id: str, sink: BinaryIO, max_bytes: int) -> None:
        """Copy stored content into ``sink``."""
        if not self._store.contains(blob_id):
            raise BlobNotFoundError(f'blob not found: {blob_id}')
        with open(self._store.content_path(blob_id), 'rb') as source:
            copy_stream(source, sink)

    def info(self, blob_id: str) -> BlobDescriptorDTO:
        """Return the stored descriptor."""
        descriptor = self._store.descriptor(blob_id)
        if descriptor is None:
            raise BlobNotFoundError(f'blob not found: {blob_id}')
        return descriptor

    def delete(self, blob_id: str) -> None:
        """Delete the blob; an absent blob counts as deleted."""
        if not self._store.remove(blob_id):
            raise BlobRepositoryError(f'could not delete local blob {blob_id}')
