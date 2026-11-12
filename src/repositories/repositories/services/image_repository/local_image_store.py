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
"""Filesystem image store with an in-memory index and size-limited eviction.

Layout under the root directory: ``<image_id>`` (PNG bytes, no extension) and
``<image_id>_metadata.json`` per image. Every public method takes one lock, so
index and file access never interleave with eviction.
"""

import os
from threading import RLock
from pathlib import Path

from rclpy.impl.rcutils_logger import RcutilsLogger

from repositories.exceptions.image_repository import ImageNotFoundError
from repositories.models.image_repository.image_index_entry_dto import ImageIndexEntryDTO
from repositories.models.image_repository.image_payload_dto import ImagePayloadDTO
from repositories.utils.uuid_validator import is_uuid4, parse_uuid4


class LocalImageStore:
    """Stores image/metadata file pairs and evicts the oldest ones above a size."""

    _METADATA_SUFFIX = '_metadata.json'

    def __init__(self, root: Path, threshold_bytes: int, logger: RcutilsLogger) -> None:
        """Create the store; no filesystem access happens until :meth:`prepare`.

        Args:
            root: Directory dedicated to this store.
            threshold_bytes: Directory size above which eviction starts.
            logger: Logger used for cleanup and eviction messages.
        """
        self._root = root
        self._threshold_bytes = threshold_bytes
        self._logger = logger
        self._lock = RLock()
        self._index: dict[str, ImageIndexEntryDTO] = {}
        self._last_stored_id: str | None = None

    # region: lifecycle

    def prepare(self) -> None:
        """Create the directory, clean it, build the index and enforce the limit.

        Files that are not a complete image/metadata pair are deleted;
        subdirectories and symbolic links are only reported.

        Raises:
            OSError: If the directory cannot be created or is not writable.
        """
        with self._lock:
            self._root.mkdir(parents=True, exist_ok=True)
            if not self._root.is_dir() or not os.access(self._root, os.W_OK | os.X_OK):
                raise OSError(f'{self._root} is not a writable directory')

            self._index = {}
            self._last_stored_id = None
            images: set[str] = set()
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
                        images.add(parsed[0])

            orphans = [self._root / i for i in images - metadata]
            orphans += [self._root / f'{i}{self._METADATA_SUFFIX}' for i in metadata - images]
            for path in unrelated + orphans:
                self._delete_file(path, 'removing stray file')

            for image_id in images & metadata:
                try:
                    self._index[image_id] = self._build_entry(image_id)
                except OSError as exc:
                    self._logger.warning(f'skipping unreadable image {image_id}: {exc}')

            self.enforce_limit()

    # endregion: lifecycle

    # region: queries and I/O

    def contains(self, image_id: str) -> bool:
        """Return whether ``image_id`` is indexed."""
        with self._lock:
            return image_id in self._index

    def read(self, image_id: str) -> ImagePayloadDTO:
        """Load a stored image.

        Args:
            image_id: UUID4 identifier of the image.

        Returns:
            ImagePayloadDTO: The stored payload.

        Raises:
            ImageNotFoundError: If the id is not indexed.
            ValueError: If the id is not a UUID4 or the stored files are corrupt.
        """
        image_id = parse_uuid4(image_id, 'image_id')
        with self._lock:
            entry = self._index.get(image_id)
            if entry is None:
                raise ImageNotFoundError(f'image not found: {image_id}')
            try:
                png_bytes = entry.image_path.read_bytes()
                metadata_bytes = entry.metadata_path.read_bytes()
            except OSError as exc:
                raise ValueError(f'stored image {image_id} is unreadable: {exc}') from exc
        try:
            return ImagePayloadDTO.from_files(png_bytes, metadata_bytes)
        except ValueError as exc:
            raise ValueError(f'stored metadata of {image_id} is corrupt: {exc}') from exc

    def write(self, image_id: str, payload: ImagePayloadDTO) -> None:
        """Atomically write an image and its metadata, then index the pair.

        The caller is responsible for making room first (see :meth:`make_room`).

        Args:
            image_id: UUID4 identifier to store the image under.
            payload: The image to store.

        Raises:
            ValueError: If ``image_id`` is not a UUID4.
            OSError: If a file cannot be written; partial files are removed.
        """
        image_id = parse_uuid4(image_id, 'image_id')
        image_path = self._root / image_id
        metadata_path = self._root / f'{image_id}{self._METADATA_SUFFIX}'
        with self._lock:
            try:
                self._atomic_write(image_path, payload.png_bytes)
                self._atomic_write(metadata_path, payload.metadata_file_bytes())
                entry = self._build_entry(image_id)
            except Exception:
                self.remove(image_id)
                raise
            self._index[image_id] = entry
            self._last_stored_id = image_id

    def remove(self, image_id: str) -> bool:
        """Delete both files of an image and drop it from the index.

        Returns:
            bool: False if a file could not be deleted; the index entry is then kept.

        Raises:
            ValueError: If ``image_id`` is not a UUID4.
        """
        image_id = parse_uuid4(image_id, 'image_id')
        with self._lock:
            image_removed = self._delete_file(
                self._root / image_id, 'removing image', missing_ok=True
            )
            metadata_removed = self._delete_file(
                self._root / f'{image_id}{self._METADATA_SUFFIX}', 'removing image', missing_ok=True
            )
            if not (image_removed and metadata_removed):
                return False
            self._index.pop(image_id, None)
            if self._last_stored_id == image_id:
                self._last_stored_id = None
            return True

    # endregion: queries and I/O

    # region: size limit

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
    ) -> None:
        """Evict the oldest pairs until ``incoming_bytes`` fit below the threshold.

        Never raises for lack of space: if every evictable pair is gone and the
        directory is still too large, a warning is logged.

        Args:
            incoming_bytes: Size of the files about to be written.
            protected_ids: Image ids that must not be evicted.
        """
        with self._lock:
            target = self._threshold_bytes - incoming_bytes
            size = self.directory_size_bytes()
            if size <= target:
                return
            candidates = sorted(
                (e for e in self._index.values()
                 if e.image_id not in protected_ids and e.image_id != self._last_stored_id),
                key=lambda e: (e.modified_ns, e.image_id),
            )
            for entry in candidates:
                if size <= target:
                    return
                if not self.remove(entry.image_id):
                    continue
                size -= entry.size_bytes
                self._logger.info(
                    f'evicted image {entry.image_id} ({entry.size_bytes} bytes)'
                )
            if size > target:
                self._logger.warning(
                    f'image directory {self._root} is above the cache threshold '
                    f'({size} > {target} bytes) and nothing more can be evicted'
                )

    def enforce_limit(self, protected_ids: frozenset[str] = frozenset()) -> None:
        """Evict the oldest pairs while the directory is above the threshold."""
        self.make_room(0, protected_ids)

    # endregion: size limit

    # region: helpers
    def _parse_name(self, name: str) -> tuple[str, bool] | None:
        """Classify a file name as ``(image_id, is_metadata)`` or None if unrelated."""
        is_metadata = name.endswith(self._METADATA_SUFFIX)
        stem = name[: -len(self._METADATA_SUFFIX)] if is_metadata else name
        if stem == stem.lower() and is_uuid4(stem):
            return stem, is_metadata
        return None

    def _build_entry(self, image_id: str) -> ImageIndexEntryDTO:
        image_path = self._root / image_id
        metadata_path = self._root / f'{image_id}{self._METADATA_SUFFIX}'
        image_stat = image_path.stat()
        return ImageIndexEntryDTO(
            image_id=image_id,
            image_path=image_path,
            metadata_path=metadata_path,
            size_bytes=image_stat.st_size + metadata_path.stat().st_size,
            modified_ns=image_stat.st_mtime_ns,
        )

    @staticmethod
    def _atomic_write(path: Path, data: bytes) -> None:
        tmp_path = path.with_name(f'.{path.name}.tmp')
        try:
            with open(tmp_path, 'wb') as handle:
                handle.write(data)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(tmp_path, path)
        except BaseException:
            tmp_path.unlink(missing_ok=True)
            raise

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
