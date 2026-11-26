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
"""Upload/fetch/info/delete orchestration for ``blob_repository`` (no ROS dependencies).

Coordinates the source-of-truth backend, the local store (store or cache),
path policy, size limits, checksums, per-id locking and batch rollback:

* Upload stages the source file into the store directory while hashing it,
  then writes the backend (cloud first) and finally the local cache.
* Overwrites keep the previous content and descriptor in a hidden backup until
  the batch commits, so a failed or canceled batch can be rolled back.
* Fetch serves the local copy, downloading into the cache on a miss (or always
  with ``download_fresh`` in cloud mode), verifies size and SHA-256 and copies
  to an explicit destination through an atomically renamed temporary file.
"""

from collections.abc import Callable, Iterable, Iterator
from contextlib import contextmanager, ExitStack
import mimetypes
import os
from pathlib import Path
import threading
import uuid

from pydantic import JsonValue
from rclpy.impl.rcutils_logger import RcutilsLogger

from repositories.exceptions.blob_repository import (
    BlobNotFoundError,
    BlobOperationCanceledError,
    BlobRepositoryError,
)
from repositories.interfaces.blob_repository.blob_backend import BlobBackend
from repositories.models.blob_repository.blob_descriptor_dto import (
    BlobDescriptorDTO,
    BlobFetchResultDTO,
    BlobUploadDescriptorDTO,
    BlobUploadItemDTO,
    DEFAULT_MEDIA_TYPE,
)
from repositories.models.blob_repository.upload_journal_dto import (
    PreviousBlobDTO,
    UploadRecordDTO,
)
from repositories.services.blob_repository.local_blob_store import LocalBlobStore
from repositories.services.blob_repository.path_policy import (
    check_destination_entry,
    PathPolicy,
)
from repositories.services.blob_repository.streaming import (
    copy_stream,
    verify,
    write_new_file,
)
from repositories.utils.uuid_validator import parse_uuid4

UploadProgress = Callable[[int, BlobUploadItemDTO, BlobDescriptorDTO], None]
FetchProgress = Callable[[int, str, BlobFetchResultDTO], None]
CancelCheck = Callable[[], bool]


class BlobManager:
    """Coordinates the backend, the local store and the path policy."""

    def __init__(
        self,
        store: LocalBlobStore,
        backend: BlobBackend,
        path_policy: PathPolicy,
        max_blob_size_bytes: int,
        logger: RcutilsLogger,
    ) -> None:
        """Create the manager.

        Args:
            store: Local store (local mode) or cache (cloud mode).
            backend: Source-of-truth backend.
            path_policy: Allowlist for sources and destinations.
            max_blob_size_bytes: Upper bound for one blob.
            logger: Logger for rollback and eviction diagnostics.
        """
        self._store = store
        self._backend = backend
        self._paths = path_policy
        self._max_blob_size_bytes = max_blob_size_bytes
        self._logger = logger
        self._locks_guard = threading.Lock()
        self._id_locks: dict[str, tuple[threading.RLock, int]] = {}

    # region: upload

    def upload(
        self,
        items: list[BlobUploadItemDTO],
        on_progress: UploadProgress | None = None,
        should_cancel: CancelCheck | None = None,
    ) -> list[BlobDescriptorDTO]:
        """Store all items or none, in request order.

        Args:
            items: Validated upload items.
            on_progress: Called after each successfully stored item.
            should_cancel: Polled between items; True cancels and rolls back.

        Returns:
            list[BlobDescriptorDTO]: Stored descriptors, in request order.

        Raises:
            BlobOperationCanceledError: If canceled; everything was rolled back.
            BlobRepositoryError: If any item fails; everything was rolled back.
                Remote rollback failures are listed in the message.
        """
        if not items:
            raise BlobRepositoryError('no files to upload')
        sources = []
        for index, item in enumerate(items):
            try:
                source = self._paths.resolve_source(item.source_path)
                self._check_size(source.stat().st_size, item.blob_name, item.metadata)
            except BlobRepositoryError as exc:
                raise BlobRepositoryError(self._item_prefix(index, len(items)) + str(exc)) from exc
            sources.append(source)

        journal: list[UploadRecordDTO] = []
        results: list[BlobDescriptorDTO] = []
        supplied = [item.blob_id for item in items if item.blob_id is not None]
        with self._locked(supplied):
            try:
                for index, (item, source) in enumerate(zip(items, sources)):
                    if should_cancel is not None and should_cancel():
                        raise BlobOperationCanceledError(
                            f'canceled after {index} of {len(items)} files'
                        )
                    protected = {d.blob_id for d in results} | set(supplied)
                    try:
                        descriptor = self._upload_one(item, source, journal, protected)
                    except Exception as exc:
                        raise self._item_error(exc, index, len(items)) from exc
                    results.append(descriptor)
                    if on_progress is not None:
                        on_progress(index, item, descriptor)
            except Exception as exc:
                affected = self._rollback(journal)
                message = str(exc) or type(exc).__name__
                if affected:
                    message += '; rollback failed for blob ids: ' + ', '.join(affected)
                error_type = (
                    BlobOperationCanceledError
                    if isinstance(exc, BlobOperationCanceledError) else BlobRepositoryError
                )
                raise error_type(message) from exc
            self._commit(journal)
        return results

    def _upload_one(
        self,
        item: BlobUploadItemDTO,
        source: Path,
        journal: list[UploadRecordDTO],
        protected: set[str],
    ) -> BlobDescriptorDTO:
        requested_name = os.path.basename(item.source_path)
        blob_name = item.blob_name or requested_name
        media_type = item.media_type or self._guess_media_type(requested_name)
        size = source.stat().st_size
        self._check_size(size, blob_name, item.metadata)
        self._store.reserve(
            size + self._store.sidecar_estimate(blob_name, media_type, item.metadata),
            protected,
        )

        staged = self._store.new_staging_path()
        try:
            with open(source, 'rb') as handle:
                writer = write_new_file(staged, size, lambda sink: copy_stream(handle, sink))
            if writer.size_bytes != size:
                raise BlobRepositoryError('source file changed size during upload')
            descriptor = BlobUploadDescriptorDTO(
                blob_name=blob_name,
                media_type=media_type,
                size_bytes=writer.size_bytes,
                sha256=writer.sha256,
                metadata=item.metadata,
            )
            record = UploadRecordDTO(blob_id=item.blob_id)
            journal.append(record)
            if self._backend.is_local:
                return self._upload_local(staged, descriptor, record)
            return self._upload_remote(staged, descriptor, record, protected)
        finally:
            staged.unlink(missing_ok=True)

    def _upload_local(
        self, staged: Path, descriptor: BlobUploadDescriptorDTO, record: UploadRecordDTO
    ) -> BlobDescriptorDTO:
        blob_id = record.blob_id or str(uuid.uuid4())
        record.blob_id = blob_id
        previous = self._store.descriptor(blob_id)
        if previous is not None:
            detached = self._store.detach(blob_id)
            if detached is not None:
                record.previous = PreviousBlobDTO(
                    descriptor=previous, content_path=detached.content_path, detached=detached
                )
        record.local_written = True
        return self._backend.upload(staged, descriptor, blob_id, replaced=previous)

    def _upload_remote(
        self,
        staged: Path,
        descriptor: BlobUploadDescriptorDTO,
        record: UploadRecordDTO,
        protected: set[str],
    ) -> BlobDescriptorDTO:
        if record.blob_id is not None:
            record.previous = self._backup_remote(record.blob_id, protected)
        record.remote_attempted = True
        stored = self._backend.upload(
            staged,
            descriptor,
            record.blob_id,
            replaced=record.previous.descriptor if record.previous else None,
        )
        record.blob_id = stored.blob_id
        record.local_written = True
        self._store.commit(staged, stored)
        return stored

    def _backup_remote(self, blob_id: str, protected: set[str]) -> PreviousBlobDTO | None:
        """Keep the current cloud content of ``blob_id`` restorable until commit."""
        try:
            current = self._backend.info(blob_id)
        except BlobNotFoundError:
            return None
        cached = self._store.descriptor(blob_id)
        if (
            cached is not None
            and cached.sha256 == current.sha256
            and cached.size_bytes == current.size_bytes
        ):
            detached = self._store.detach(blob_id)
            if detached is not None:
                return PreviousBlobDTO(
                    descriptor=current, content_path=detached.content_path, detached=detached
                )
        self._check_size(current.size_bytes, current.blob_name, current.metadata)
        self._store.reserve(current.size_bytes, protected | {blob_id})
        backup = self._store.new_staging_path()
        self._download_verified(blob_id, backup, current)
        return PreviousBlobDTO(descriptor=current, content_path=backup)

    def _rollback(self, journal: list[UploadRecordDTO]) -> list[str]:
        """Undo a failed batch in reverse order; returns ids whose rollback failed."""
        affected: list[str] = []
        for record in reversed(journal):
            if not self._rollback_one(record) and record.blob_id is not None:
                affected.append(record.blob_id)
        return list(dict.fromkeys(affected))

    def _rollback_one(self, record: UploadRecordDTO) -> bool:
        blob_id = record.blob_id
        previous = record.previous
        remote_ok = True
        if blob_id is not None and not self._backend.is_local and record.remote_attempted:
            try:
                if previous is None:
                    self._backend.delete(blob_id)
                else:
                    self._backend.upload(
                        previous.content_path,
                        previous.descriptor.upload_descriptor(),
                        blob_id,
                    )
            except Exception as exc:  # noqa: BLE001 - keep rolling back
                self._logger.error(f'rollback: cloud restore of blob {blob_id} failed: {exc}')
                remote_ok = False
        local_ok = True
        try:
            if blob_id is not None and (record.local_written or not remote_ok):
                if not self._store.remove(blob_id):
                    self._logger.error(f'rollback: could not remove local blob {blob_id}')
                    local_ok = False
            if previous is not None:
                if not remote_ok:
                    # The cloud keeps the new content; a restored cache would be stale.
                    self._drop_backup(previous)
                elif previous.detached is not None:
                    self._store.restore(previous.detached)
                else:
                    self._store.commit(previous.content_path, previous.descriptor)
        except Exception as exc:  # noqa: BLE001 - keep rolling back
            self._logger.error(f'rollback: local restore of blob {blob_id} failed: {exc}')
            local_ok = False
        return remote_ok and local_ok

    def _commit(self, journal: list[UploadRecordDTO]) -> None:
        """Drop the backups of a successful batch."""
        for record in journal:
            if record.previous is not None:
                self._drop_backup(record.previous)

    def _drop_backup(self, previous: PreviousBlobDTO) -> None:
        if previous.detached is not None:
            self._store.discard(previous.detached)
        else:
            previous.content_path.unlink(missing_ok=True)

    # endregion: upload

    # region: fetch / info / delete

    def fetch(
        self,
        blob_ids: list[str],
        destinations: list[str],
        download_fresh: bool = False,
        on_progress: FetchProgress | None = None,
        should_cancel: CancelCheck | None = None,
    ) -> list[BlobFetchResultDTO]:
        """Make blobs available locally, fail-fast, in request order.

        Args:
            blob_ids: UUID4 ids to fetch.
            destinations: Empty, or one path per id; empty paths use the cache.
            download_fresh: Cloud mode only: always re-download from the cloud.
            on_progress: Called after each successfully fetched item.
            should_cancel: Polled between items; True stops the batch.

        Returns:
            list[BlobFetchResultDTO]: One result per id, in request order.

        Raises:
            BlobOperationCanceledError: If canceled between items.
            BlobRepositoryError: On the first failing item (earlier items stay
                available) or if the request is malformed.
        """
        if not blob_ids:
            raise BlobRepositoryError('no blob ids requested')
        if destinations and len(destinations) != len(blob_ids):
            raise BlobRepositoryError(
                'destination_paths must be empty or have the same length as blob_ids'
            )
        try:
            ids = [parse_uuid4(blob_id, 'blob_id') for blob_id in blob_ids]
        except ValueError as exc:
            raise BlobRepositoryError(str(exc)) from exc
        targets: list[Path | None] = []
        for raw in destinations or [''] * len(ids):
            targets.append(self._paths.resolve_destination(raw) if raw else None)

        results: list[BlobFetchResultDTO] = []
        for index, (blob_id, target) in enumerate(zip(ids, targets)):
            if should_cancel is not None and should_cancel():
                raise BlobOperationCanceledError(f'canceled after {index} of {len(ids)} blobs')
            protected = {r.descriptor.blob_id for r in results}
            try:
                result = self._fetch_one(blob_id, target, download_fresh, protected)
            except Exception as exc:
                raise self._item_error(exc, index, len(ids)) from exc
            results.append(result)
            if on_progress is not None:
                on_progress(index, blob_id, result)
        return results

    def info(self, blob_id: str) -> BlobDescriptorDTO:
        """Return the descriptor of a blob without fetching content.

        Raises:
            BlobRepositoryError: If the id is invalid or the lookup fails.
            BlobNotFoundError: If the blob does not exist.
        """
        return self._backend.info(self._parse_id(blob_id))

    def delete(self, blob_id: str) -> None:
        """Delete a blob and its local copy; an absent blob counts as deleted.

        Raises:
            BlobRepositoryError: If the id is invalid or the deletion fails.
        """
        blob_id = self._parse_id(blob_id)
        with self._locked([blob_id]):
            self._backend.delete(blob_id)
            if not self._backend.is_local and not self._store.remove(blob_id):
                raise BlobRepositoryError(f'could not delete the local copy of {blob_id}')

    def enforce_cache_limit(self) -> None:
        """Evict the oldest unpinned blobs while the store is above its limit."""
        self._store.enforce_limit()

    def _fetch_one(
        self, blob_id: str, target: Path | None, download_fresh: bool, protected: set[str]
    ) -> BlobFetchResultDTO:
        with self._locked([blob_id]):
            descriptor = self._ensure_local(blob_id, download_fresh, protected)
            with self._store.pinned([blob_id]):
                if target is None:
                    return BlobFetchResultDTO(
                        local_path=self._store.content_path(blob_id), descriptor=descriptor
                    )
                self._copy_to_destination(blob_id, descriptor, target)
                return BlobFetchResultDTO(local_path=target, descriptor=descriptor)

    def _ensure_local(
        self, blob_id: str, download_fresh: bool, protected: set[str]
    ) -> BlobDescriptorDTO:
        cached = self._store.descriptor(blob_id)
        if self._backend.is_local:
            if cached is None:
                raise BlobNotFoundError(f'blob not found: {blob_id}')
            return cached
        if cached is not None and not download_fresh:
            return cached

        remote = self._backend.info(blob_id)
        self._check_size(remote.size_bytes, remote.blob_name, remote.metadata)
        self._store.reserve(
            remote.size_bytes + len(remote.sidecar_bytes()), protected | {blob_id}
        )
        staged = self._store.new_staging_path()
        try:
            self._download_verified(blob_id, staged, remote)
            self._store.commit(staged, remote)
        finally:
            staged.unlink(missing_ok=True)
        return remote

    def _download_verified(
        self, blob_id: str, path: Path, expected: BlobDescriptorDTO
    ) -> None:
        """Download into a new file at ``path`` and verify size and SHA-256."""
        writer = write_new_file(
            path,
            expected.size_bytes,
            lambda sink: self._backend.download(blob_id, sink, expected.size_bytes),
        )
        try:
            verify(writer, expected.size_bytes, expected.sha256, f'download of {blob_id}')
        except BlobRepositoryError:
            path.unlink(missing_ok=True)
            raise

    def _copy_to_destination(
        self, blob_id: str, descriptor: BlobDescriptorDTO, target: Path
    ) -> None:
        """Copy and verify into a temp file next to ``target``, then rename over it."""
        tmp = target.parent / f'.blob-{uuid.uuid4().hex}.tmp'
        with open(self._store.content_path(blob_id), 'rb') as source:
            writer = write_new_file(
                tmp, descriptor.size_bytes, lambda sink: copy_stream(source, sink)
            )
        try:
            verify(writer, descriptor.size_bytes, descriptor.sha256, f'local copy of {blob_id}')
            check_destination_entry(target)
            os.replace(tmp, target)
        finally:
            tmp.unlink(missing_ok=True)

    # endregion: fetch / info / delete

    # region: helpers

    def _check_size(self, size: int, blob_name: str, metadata: JsonValue) -> None:
        if size > self._max_blob_size_bytes:
            raise BlobRepositoryError(
                f'file is too large ({size} bytes > max_blob_size_mb limit of '
                f'{self._max_blob_size_bytes} bytes)'
            )
        needed = size + self._store.sidecar_estimate(
            blob_name, DEFAULT_MEDIA_TYPE, metadata
        )
        if needed > self._store.capacity_bytes:
            raise BlobRepositoryError(
                f'file does not fit in the local cache ({needed} > '
                f'{self._store.capacity_bytes} bytes)'
            )

    @staticmethod
    def _guess_media_type(file_name: str) -> str:
        media_type, _ = mimetypes.guess_type(file_name, strict=False)
        return media_type or DEFAULT_MEDIA_TYPE

    @staticmethod
    def _parse_id(blob_id: str) -> str:
        try:
            return parse_uuid4(blob_id, 'blob_id')
        except ValueError as exc:
            raise BlobRepositoryError(str(exc)) from exc

    @staticmethod
    def _item_prefix(index: int, total: int) -> str:
        return '' if total == 1 else f'item {index}: '

    @classmethod
    def _item_error(cls, exc: Exception, index: int, total: int) -> BlobRepositoryError:
        """Wrap a per-item failure, keeping the specific error type if it has one."""
        error_type = type(exc) if isinstance(exc, BlobRepositoryError) else BlobRepositoryError
        return error_type(cls._item_prefix(index, total) + (str(exc) or type(exc).__name__))

    @contextmanager
    def _locked(self, blob_ids: Iterable[str]) -> Iterator[None]:
        """Hold the per-id locks of ``blob_ids`` (acquired in sorted order)."""
        ids = sorted(set(blob_ids))
        with ExitStack() as stack:
            for blob_id in ids:
                lock = self._acquire_lock_ref(blob_id)
                stack.callback(self._release_lock_ref, blob_id)
                lock.acquire()
                stack.callback(lock.release)
            yield

    def _acquire_lock_ref(self, blob_id: str) -> threading.RLock:
        with self._locks_guard:
            lock, refs = self._id_locks.get(blob_id, (threading.RLock(), 0))
            self._id_locks[blob_id] = (lock, refs + 1)
            return lock

    def _release_lock_ref(self, blob_id: str) -> None:
        with self._locks_guard:
            lock, refs = self._id_locks[blob_id]
            if refs <= 1:
                del self._id_locks[blob_id]
            else:
                self._id_locks[blob_id] = (lock, refs - 1)

    # endregion: helpers
