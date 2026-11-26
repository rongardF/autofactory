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
``<image_id>_metadata.json`` per image. Indexing, locking, atomic writes,
directory cleanup and oldest-first eviction come from the shared
:class:`ManagedFileStore`; every public method runs under its lock, so index and
file access never interleave with eviction.
"""

from pathlib import Path

from rclpy.impl.rcutils_logger import RcutilsLogger

from repositories.exceptions.image_repository import ImageNotFoundError
from repositories.models.image_repository.image_payload_dto import ImagePayloadDTO
from repositories.services.common.managed_file_store import ManagedFileStore
from repositories.utils.uuid_validator import parse_uuid4


class LocalImageStore:
    """Stores image/metadata file pairs and evicts the oldest ones above a size."""

    def __init__(self, root: Path, threshold_bytes: int, logger: RcutilsLogger) -> None:
        """Create the store; no filesystem access happens until :meth:`prepare`.

        Args:
            root: Directory dedicated to this store.
            threshold_bytes: Directory size above which eviction starts.
            logger: Logger used for cleanup and eviction messages.
        """
        self._core = ManagedFileStore(
            root, threshold_bytes, logger, item_label='image', protect_last_stored=True
        )

    # region: lifecycle

    def prepare(self) -> None:
        """Create the directory, clean it, build the index and enforce the limit.

        Files that are not a complete image/metadata pair are deleted;
        subdirectories and symbolic links are only reported.

        Raises:
            OSError: If the directory cannot be created or is not writable.
        """
        self._core.prepare()

    # endregion: lifecycle

    # region: queries and I/O

    def contains(self, image_id: str) -> bool:
        """Return whether ``image_id`` is indexed."""
        return self._core.contains(image_id)

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
        with self._core.lock:
            entry = self._core.entry(image_id)
            if entry is None:
                raise ImageNotFoundError(f'image not found: {image_id}')
            try:
                png_bytes = entry.content_path.read_bytes()
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
        self._core.write_bytes(image_id, payload.png_bytes, payload.metadata_file_bytes())

    def remove(self, image_id: str) -> bool:
        """Delete both files of an image and drop it from the index.

        Returns:
            bool: False if a file could not be deleted; the index entry is then kept.

        Raises:
            ValueError: If ``image_id`` is not a UUID4.
        """
        return self._core.remove(parse_uuid4(image_id, 'image_id'))

    # endregion: queries and I/O

    # region: size limit

    def directory_size_bytes(self) -> int:
        """Return the total size of the regular files directly under the root."""
        return self._core.directory_size_bytes()

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
        self._core.make_room(incoming_bytes, protected_ids)

    def enforce_limit(self, protected_ids: frozenset[str] = frozenset()) -> None:
        """Evict the oldest pairs while the directory is above the threshold."""
        self._core.make_room(0, protected_ids)

    # endregion: size limit
