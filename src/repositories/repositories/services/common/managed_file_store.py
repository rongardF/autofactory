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
"""Shared filesystem store core: atomic file pairs, index, locking and eviction.

Layout under the root directory: ``<entry_id>`` (content, no extension) and
``<entry_id>_metadata.json`` per entry, where ``entry_id`` is a canonical
lowercase UUID4. Every other name is foreign to the store; temporary, staging
and backup files are hidden (``.``-prefixed) so :meth:`ManagedFileStore.prepare`
removes leftovers of interrupted operations.

All index and file operations take one reentrant lock, so they never interleave
with eviction. Eviction removes the oldest entries first (content modification
time, then id) and skips protected, pinned and (optionally) the most recently
stored entries.
"""

from collections import Counter
from collections.abc import Callable, Iterable, Iterator
from contextlib import contextmanager
import os
from pathlib import Path
from threading import RLock
import uuid

from rclpy.impl.rcutils_logger import RcutilsLogger

from repositories.models.common.stored_file_entry_dto import (
    DetachedEntryDTO,
    StoredFileEntryDTO,
)
from repositories.utils.uuid_validator import is_uuid4

METADATA_SUFFIX = '_metadata.json'


class ManagedFileStore:
    """Stores content/metadata file pairs and evicts the oldest ones above a size."""

    def __init__(
        self,
        root: Path,
        threshold_bytes: int,
        logger: RcutilsLogger,
        item_label: str = 'item',
        protect_last_stored: bool = False,
        validate_entry: Callable[[StoredFileEntryDTO], bool] | None = None,
    ) -> None:
        """Create the store; no filesystem access happens until :meth:`prepare`.

        Args:
            root: Directory dedicated to this store.
            threshold_bytes: Directory size above which eviction starts.
            logger: Logger used for cleanup and eviction messages.
            item_label: Noun used in log messages (e.g. ``'image'``).
            protect_last_stored: Never evict the most recently stored entry.
            validate_entry: Optional check run by :meth:`prepare` on every
                complete pair; pairs for which it returns False are deleted.
        """
        self._root = root
        self._threshold_bytes = threshold_bytes
        self._logger = logger
        self._label = item_label
        self._protect_last_stored = protect_last_stored
        self._validate_entry = validate_entry
        self._lock = RLock()
        self._index: dict[str, StoredFileEntryDTO] = {}
        self._last_stored_id: str | None = None
        self._pins: Counter[str] = Counter()

    @property
    def root(self) -> Path:
        """Directory dedicated to this store."""
        return self._root

    @property
    def threshold_bytes(self) -> int:
        """Directory size above which eviction starts."""
        return self._threshold_bytes

    @property
    def lock(self) -> RLock:
        """Return the lock serializing all index and file operations."""
        return self._lock

    # region: lifecycle

    def prepare(self) -> None:
        """Create the directory, clean it, build the index and enforce the limit.

        Files that are not a complete content/metadata pair (including hidden
        temporary, staging and backup files) are deleted; subdirectories and
        symbolic links are only reported.

        Raises:
            OSError: If the directory cannot be created or is not writable.
        """
        with self._lock:
            self._root.mkdir(parents=True, exist_ok=True)
            if not self._root.is_dir() or not os.access(self._root, os.W_OK | os.X_OK):
                raise OSError(f'{self._root} is not a writable directory')

            self._index = {}
            self._last_stored_id = None
            contents: set[str] = set()
            metadata: set[str] = set()
            unrelated: list[Path] = []
            with os.scandir(self._root) as entries:
                for entry in entries:
                    path = Path(entry.path)
                    if entry.is_symlink() or not entry.is_file(follow_symlinks=False):
                        self._logger.warning(f'ignoring non-file entry {path}')
                        continue
                    parsed = self._parse_name(entry.name)
                    if parsed is None:
                        unrelated.append(path)
                    elif parsed[1]:
                        metadata.add(parsed[0])
                    else:
                        contents.add(parsed[0])

            orphans = [self.content_path(i) for i in contents - metadata]
            orphans += [self.metadata_path(i) for i in metadata - contents]
            for path in unrelated + orphans:
                self._delete_file(path, 'removing stray file')

            for entry_id in contents & metadata:
                try:
                    entry = self._build_entry(entry_id)
                except OSError as exc:
                    self._logger.warning(f'skipping unreadable {self._label} {entry_id}: {exc}')
                    continue
                if self._validate_entry is not None and not self._validate_entry(entry):
                    self._logger.warning(f'removing incomplete {self._label} {entry_id}')
                    self._delete_file(entry.content_path, 'removing incomplete entry', True)
                    self._delete_file(entry.metadata_path, 'removing incomplete entry', True)
                    continue
                self._index[entry_id] = entry

            self.enforce_limit()

    # endregion: lifecycle

    # region: queries

    def contains(self, entry_id: str) -> bool:
        """Return whether ``entry_id`` is indexed."""
        with self._lock:
            return entry_id in self._index

    def entry(self, entry_id: str) -> StoredFileEntryDTO | None:
        """Return the index entry of ``entry_id`` or None."""
        with self._lock:
            return self._index.get(entry_id)

    def entries(self) -> list[StoredFileEntryDTO]:
        """Return a snapshot of all index entries."""
        with self._lock:
            return list(self._index.values())

    def content_path(self, entry_id: str) -> Path:
        """Return the content file path of ``entry_id`` (whether or not it exists)."""
        return self._root / entry_id

    def metadata_path(self, entry_id: str) -> Path:
        """Return the metadata sidecar path of ``entry_id`` (whether or not it exists)."""
        return self._root / f'{entry_id}{METADATA_SUFFIX}'

    def new_staging_path(self) -> Path:
        """Return a fresh hidden path in the root for staging incoming content."""
        return self._root / f'.staging-{uuid.uuid4().hex}.tmp'

    # endregion: queries

    # region: writes

    def write_bytes(self, entry_id: str, content: bytes, metadata: bytes) -> None:
        """Atomically write a content/metadata pair from memory, then index it.

        The caller is responsible for making room first (see :meth:`make_room`).

        Raises:
            OSError: If a file cannot be written; the entry is then removed.
        """
        with self._lock:
            try:
                self.atomic_write_bytes(self.content_path(entry_id), content)
                self.atomic_write_bytes(self.metadata_path(entry_id), metadata)
                entry = self._build_entry(entry_id)
            except Exception:
                self.remove(entry_id)
                raise
            self._index[entry_id] = entry
            self._last_stored_id = entry_id

    def commit_staged(self, entry_id: str, staged_content: Path, metadata: bytes) -> None:
        """Move a staged content file into place with its metadata, then index it.

        Any entry already stored under ``entry_id`` is replaced. A failure
        before the content is moved leaves the previous entry untouched; the
        staged file then stays where it is and belongs to the caller.

        Args:
            entry_id: Canonical UUID4 to store the content under.
            staged_content: Content file on the same filesystem (usually from
                :meth:`new_staging_path`), fully written and flushed.
            metadata: Metadata sidecar bytes.

        Raises:
            OSError: If the pair cannot be put in place.
        """
        with self._lock:
            metadata_tmp = self.new_staging_path()
            try:
                self._write_synced(metadata_tmp, metadata)
                os.replace(staged_content, self.content_path(entry_id))
            except BaseException:
                metadata_tmp.unlink(missing_ok=True)
                raise
            try:
                os.replace(metadata_tmp, self.metadata_path(entry_id))
                entry = self._build_entry(entry_id)
            except BaseException:
                metadata_tmp.unlink(missing_ok=True)
                self.remove(entry_id)
                raise
            self._index[entry_id] = entry
            self._last_stored_id = entry_id

    def detach(self, entry_id: str) -> DetachedEntryDTO | None:
        """Move an entry to hidden backup files and drop it from the index.

        The backup keeps counting towards the directory size until it is
        restored or discarded, and is deleted by the next :meth:`prepare`.

        Returns:
            DetachedEntryDTO | None: The backup, or None if the id is not indexed.

        Raises:
            OSError: If the files cannot be moved; the entry is then unchanged.
        """
        with self._lock:
            entry = self._index.get(entry_id)
            if entry is None:
                return None
            token = uuid.uuid4().hex
            content_backup = self._root / f'.{entry_id}.{token}.bak'
            metadata_backup = self._root / f'.{entry_id}{METADATA_SUFFIX}.{token}.bak'
            os.replace(entry.content_path, content_backup)
            try:
                os.replace(entry.metadata_path, metadata_backup)
            except BaseException:
                os.replace(content_backup, entry.content_path)
                raise
            self._index.pop(entry_id, None)
            if self._last_stored_id == entry_id:
                self._last_stored_id = None
            return DetachedEntryDTO(
                entry_id=entry_id,
                content_path=content_backup,
                metadata_path=metadata_backup,
                size_bytes=entry.size_bytes,
            )

    def restore(self, detached: DetachedEntryDTO) -> None:
        """Put a detached entry back in place, replacing whatever is stored now.

        Raises:
            OSError: If the current entry cannot be removed or the files moved.
        """
        entry_id = detached.entry_id
        with self._lock:
            if not self.remove(entry_id):
                raise OSError(f'cannot remove current {self._label} {entry_id}')
            os.replace(detached.content_path, self.content_path(entry_id))
            os.replace(detached.metadata_path, self.metadata_path(entry_id))
            self._index[entry_id] = self._build_entry(entry_id)

    def discard(self, detached: DetachedEntryDTO) -> bool:
        """Delete the backup files of a detached entry.

        Returns:
            bool: False if a file could not be deleted (it is logged).
        """
        with self._lock:
            content_ok = self._delete_file(detached.content_path, 'discarding backup', True)
            metadata_ok = self._delete_file(detached.metadata_path, 'discarding backup', True)
            return content_ok and metadata_ok

    def remove(self, entry_id: str) -> bool:
        """Delete both files of an entry and drop it from the index.

        Returns:
            bool: False if a file could not be deleted; the index entry is then kept.
        """
        with self._lock:
            content_removed = self._delete_file(
                self.content_path(entry_id), f'removing {self._label}', missing_ok=True
            )
            metadata_removed = self._delete_file(
                self.metadata_path(entry_id), f'removing {self._label}', missing_ok=True
            )
            if not (content_removed and metadata_removed):
                return False
            self._index.pop(entry_id, None)
            if self._last_stored_id == entry_id:
                self._last_stored_id = None
            return True

    # endregion: writes

    # region: size limit

    @contextmanager
    def pinned(self, entry_ids: Iterable[str]) -> Iterator[None]:
        """Protect ``entry_ids`` from eviction while the context is open."""
        ids = list(entry_ids)
        with self._lock:
            self._pins.update(ids)
        try:
            yield
        finally:
            with self._lock:
                self._pins.subtract(ids)
                self._pins += Counter()

    def directory_size_bytes(self) -> int:
        """Return the total size of the regular files directly under the root."""
        total = 0
        with self._lock, os.scandir(self._root) as entries:
            for entry in entries:
                try:
                    if entry.is_file(follow_symlinks=False):
                        total += entry.stat(follow_symlinks=False).st_size
                except OSError:
                    continue
        return total

    def make_room(
        self, incoming_bytes: int, protected_ids: frozenset[str] = frozenset()
    ) -> bool:
        """Evict the oldest pairs until ``incoming_bytes`` fit below the threshold.

        Never raises for lack of space: if every evictable pair is gone and the
        directory is still too large, a warning is logged and False returned.

        Args:
            incoming_bytes: Size of the files about to be written.
            protected_ids: Entry ids that must not be evicted.

        Returns:
            bool: Whether ``incoming_bytes`` now fit below the threshold.
        """
        with self._lock:
            target = self._threshold_bytes - incoming_bytes
            size = self.directory_size_bytes()
            if size <= target:
                return True
            candidates = sorted(
                (e for e in self._index.values() if self._is_evictable(e.entry_id, protected_ids)),
                key=lambda e: (e.modified_ns, e.entry_id),
            )
            for entry in candidates:
                if size <= target:
                    return True
                if not self.remove(entry.entry_id):
                    continue
                size -= entry.size_bytes
                self._logger.info(
                    f'evicted {self._label} {entry.entry_id} ({entry.size_bytes} bytes)'
                )
            if size > target:
                self._logger.warning(
                    f'{self._label} directory {self._root} is above the cache threshold '
                    f'({size} > {target} bytes) and nothing more can be evicted'
                )
                return False
            return True

    def enforce_limit(self, protected_ids: frozenset[str] = frozenset()) -> bool:
        """Evict the oldest pairs while the directory is above the threshold."""
        return self.make_room(0, protected_ids)

    # endregion: size limit

    # region: helpers

    @staticmethod
    def atomic_write_bytes(path: Path, data: bytes) -> None:
        """Write ``data`` to a hidden sibling temp file, fsync it and rename it over ``path``."""
        tmp_path = path.with_name(f'.{path.name}.tmp')
        try:
            ManagedFileStore._write_synced(tmp_path, data)
            os.replace(tmp_path, path)
        except BaseException:
            tmp_path.unlink(missing_ok=True)
            raise

    @staticmethod
    def _write_synced(path: Path, data: bytes) -> None:
        with open(path, 'wb') as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())

    def _is_evictable(self, entry_id: str, protected_ids: frozenset[str]) -> bool:
        if entry_id in protected_ids or self._pins[entry_id] > 0:
            return False
        return not (self._protect_last_stored and entry_id == self._last_stored_id)

    @staticmethod
    def _parse_name(name: str) -> tuple[str, bool] | None:
        """Classify a file name as ``(entry_id, is_metadata)`` or None if unrelated."""
        is_metadata = name.endswith(METADATA_SUFFIX)
        stem = name[: -len(METADATA_SUFFIX)] if is_metadata else name
        if stem == stem.lower() and is_uuid4(stem):
            return stem, is_metadata
        return None

    def _build_entry(self, entry_id: str) -> StoredFileEntryDTO:
        content_path = self.content_path(entry_id)
        metadata_path = self.metadata_path(entry_id)
        content_stat = content_path.stat()
        return StoredFileEntryDTO(
            entry_id=entry_id,
            content_path=content_path,
            metadata_path=metadata_path,
            size_bytes=content_stat.st_size + metadata_path.stat().st_size,
            modified_ns=content_stat.st_mtime_ns,
        )

    def _delete_file(self, path: Path, action: str, missing_ok: bool = False) -> bool:
        try:
            path.unlink(missing_ok=missing_ok)
        except OSError as exc:
            self._logger.warning(f'{action}: could not delete {path}: {exc}')
            return False
        if not missing_ok:
            self._logger.warning(f'{action}: deleted {path}')
        return True

    # endregion: helpers
