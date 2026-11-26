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
"""Local blob store: content/descriptor pairs on top of :class:`ManagedFileStore`.

Layout under the root directory: ``<blob_id>`` (raw content) and
``<blob_id>_metadata.json`` (a :class:`BlobDescriptorDTO`). The limit is strict:
an operation that cannot make room fails instead of exceeding it.
"""

from collections.abc import Iterable, Iterator
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path

from pydantic import JsonValue, ValidationError
from rclpy.impl.rcutils_logger import RcutilsLogger

from repositories.exceptions.blob_repository import BlobRepositoryError
from repositories.models.blob_repository.blob_descriptor_dto import BlobDescriptorDTO
from repositories.models.common.stored_file_entry_dto import (
    DetachedEntryDTO,
    StoredFileEntryDTO,
)
from repositories.services.common.managed_file_store import ManagedFileStore

_SIDECAR_MARGIN_BYTES = 256


class LocalBlobStore:
    """Stores blob content with JSON descriptors under a strict size limit."""

    def __init__(self, root: Path, capacity_bytes: int, logger: RcutilsLogger) -> None:
        """Create the store; no filesystem access happens until :meth:`prepare`."""
        self._capacity_bytes = capacity_bytes
        self._core = ManagedFileStore(
            root, capacity_bytes, logger, item_label='blob', validate_entry=self._is_valid
        )

    @property
    def capacity_bytes(self) -> int:
        """Upper bound for the directory size in bytes."""
        return self._capacity_bytes

    @property
    def root(self) -> Path:
        """Directory dedicated to this store."""
        return self._core.root

    def prepare(self) -> None:
        """Create and clean the directory, index valid blobs and enforce the limit.

        Raises:
            OSError: If the directory cannot be created or is not writable.
        """
        self._core.prepare()

    def contains(self, blob_id: str) -> bool:
        """Return whether ``blob_id`` is stored."""
        return self._core.contains(blob_id)

    def ids(self) -> list[str]:
        """Return the ids of all stored blobs."""
        return [entry.entry_id for entry in self._core.entries()]

    def content_path(self, blob_id: str) -> Path:
        """Return the content file path of ``blob_id``."""
        return self._core.content_path(blob_id)

    def descriptor(self, blob_id: str) -> BlobDescriptorDTO | None:
        """Return the stored descriptor of ``blob_id`` or None if absent.

        Raises:
            BlobRepositoryError: If the sidecar is unreadable or corrupt.
        """
        with self._core.lock:
            entry = self._core.entry(blob_id)
            if entry is None:
                return None
            try:
                return BlobDescriptorDTO.model_validate_json(entry.metadata_path.read_bytes())
            except (OSError, ValidationError) as exc:
                raise BlobRepositoryError(
                    f'stored descriptor of {blob_id} is unreadable'
                ) from exc

    def new_staging_path(self) -> Path:
        """Return a fresh hidden path in the store for staging content."""
        return self._core.new_staging_path()

    def reserve(self, incoming_bytes: int, protected_ids: Iterable[str] = ()) -> None:
        """Evict the oldest unprotected blobs until ``incoming_bytes`` fit.

        Raises:
            BlobRepositoryError: If the space cannot be made available.
        """
        if incoming_bytes > self._capacity_bytes:
            raise BlobRepositoryError(
                f'blob does not fit in the local cache ({incoming_bytes} > '
                f'{self._capacity_bytes} bytes)'
            )
        if not self._core.make_room(incoming_bytes, frozenset(protected_ids)):
            raise BlobRepositoryError(
                f'insufficient local cache space for {incoming_bytes} bytes'
            )

    def commit(self, staged_content: Path, descriptor: BlobDescriptorDTO) -> None:
        """Move staged content into place with its descriptor (replacing any blob).

        Raises:
            OSError: If the pair cannot be put in place.
        """
        self._core.commit_staged(
            descriptor.blob_id, staged_content, descriptor.sidecar_bytes()
        )

    def detach(self, blob_id: str) -> DetachedEntryDTO | None:
        """Move a stored blob to hidden backup files (see :meth:`ManagedFileStore.detach`)."""
        return self._core.detach(blob_id)

    def restore(self, detached: DetachedEntryDTO) -> None:
        """Put a detached blob back in place."""
        self._core.restore(detached)

    def discard(self, detached: DetachedEntryDTO) -> bool:
        """Delete the backup files of a detached blob."""
        return self._core.discard(detached)

    def remove(self, blob_id: str) -> bool:
        """Delete a blob; returns False if a file could not be deleted."""
        return self._core.remove(blob_id)

    @contextmanager
    def pinned(self, blob_ids: Iterable[str]) -> Iterator[None]:
        """Protect ``blob_ids`` from eviction while the context is open."""
        with self._core.pinned(blob_ids):
            yield

    def enforce_limit(self) -> bool:
        """Evict the oldest unpinned blobs while the directory is above the limit."""
        return self._core.enforce_limit()

    def directory_size_bytes(self) -> int:
        """Return the total size of the regular files directly under the root."""
        return self._core.directory_size_bytes()

    @staticmethod
    def sidecar_estimate(blob_name: str, media_type: str, metadata: JsonValue) -> int:
        """Return an upper estimate of the sidecar size for space reservation."""
        now = datetime.now(timezone.utc)
        probe = BlobDescriptorDTO(
            blob_id='00000000-0000-4000-8000-000000000000',
            blob_name=blob_name,
            media_type=media_type,
            size_bytes=2**63 - 1,
            sha256='0' * 64,
            created_at=now,
            updated_at=now,
            metadata=metadata,
        )
        return len(probe.sidecar_bytes()) + _SIDECAR_MARGIN_BYTES

    @staticmethod
    def _is_valid(entry: StoredFileEntryDTO) -> bool:
        """Keep only pairs whose descriptor parses, matches the id and the size."""
        try:
            descriptor = BlobDescriptorDTO.model_validate_json(entry.metadata_path.read_bytes())
            return (
                descriptor.blob_id == entry.entry_id
                and descriptor.size_bytes == entry.content_path.stat().st_size
            )
        except (OSError, ValidationError):
            return False
