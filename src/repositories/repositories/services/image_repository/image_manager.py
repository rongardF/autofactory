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
"""Upload/fetch orchestration for ``image_repository`` (no ROS dependencies).

Combines the local store with an optional remote backend: remote first on
upload, make room before writing, all-or-nothing batches with rollback.
"""

import uuid

from rclpy.impl.rcutils_logger import RcutilsLogger

from repositories.exceptions.image_repository import (
    ImageNotFoundError,
    ImageRepositoryError,
)
from repositories.interfaces.image_repository.image_backend import ImageBackend
from repositories.models.image_repository.image_payload_dto import ImagePayloadDTO
from repositories.services.image_repository.local_image_store import LocalImageStore
from repositories.utils.uuid_validator import is_uuid4, parse_uuid4


class ImageManager:
    """Coordinates the local store and the optional remote backend."""

    def __init__(
        self,
        store: LocalImageStore,
        backend: ImageBackend | None,
        logger: RcutilsLogger,
    ) -> None:
        """Create the manager.

        Args:
            store: The local image store.
            backend: Remote backend, or None for local mode.
            logger: Logger used for rollback diagnostics.
        """
        self._store = store
        self._backend = backend
        self._logger = logger

    def upload(self, payloads: list[ImagePayloadDTO]) -> list[str]:
        """Store all payloads or none.

        Args:
            payloads: Already validated and encoded images.

        Returns:
            list[str]: The image ids, in request order.

        Raises:
            ImageRepositoryError: If any item fails; everything stored by this
                call has been rolled back.
            NotImplementedError: If the remote backend is a stub and nothing
                could be stored.
        """
        if not payloads:
            raise ImageRepositoryError('no images to upload')

        stored: list[str] = []
        pending_remote_id: str | None = None
        try:
            for payload in payloads:
                if self._backend is None:
                    image_id = str(uuid.uuid4())
                else:
                    image_id = parse_uuid4(self._backend.upload(payload), 'remote image_id')
                    pending_remote_id = image_id
                self._store.make_room(payload.size_bytes, frozenset(stored))
                self._store.write(image_id, payload)
                stored.append(image_id)
                pending_remote_id = None
        except Exception as exc:
            remote_failures = self._rollback(stored, pending_remote_id)
            if isinstance(exc, NotImplementedError) and not remote_failures:
                raise
            message = str(exc) or type(exc).__name__
            if remote_failures:
                message += (
                    '; could not be removed from the remote service: '
                    + ', '.join(remote_failures)
                )
            raise ImageRepositoryError(message) from exc

        self._store.enforce_limit(frozenset(stored))
        return stored

    def fetch(self, image_ids: list[str]) -> list[ImagePayloadDTO]:
        """Load all images or fail.

        Args:
            image_ids: UUID4 identifiers to load.

        Returns:
            list[ImagePayloadDTO]: The payloads, in request order.

        Raises:
            ImageRepositoryError: If any id is invalid, unknown or unreadable.
            NotImplementedError: If an image must come from a stub backend.
        """
        if not image_ids:
            raise ImageRepositoryError('no image ids requested')
        invalid = [i for i in image_ids if not is_uuid4(i)]
        if invalid:
            raise ImageRepositoryError(f'invalid image_id (must be UUID4): {", ".join(invalid)}')
        ids = [parse_uuid4(i) for i in image_ids]

        if self._backend is None:
            missing = [i for i in image_ids if not self._store.contains(i.lower())]
            if missing:
                raise ImageRepositoryError(
                    'image not found' if len(image_ids) == 1
                    else f'image not found: {", ".join(missing)}'
                )

        payloads: list[ImagePayloadDTO] = []
        for index, image_id in enumerate(ids):
            payloads.append(self._fetch_one(image_id, frozenset(ids[:index])))
        self._store.enforce_limit(frozenset(ids))
        return payloads

    def enforce_cache_limit(self) -> None:
        """Evict the oldest images while the local directory is above its threshold."""
        self._store.enforce_limit()

    def _fetch_one(self, image_id: str, protected: frozenset[str]) -> ImagePayloadDTO:
        try:
            return self._store.read(image_id)
        except ImageNotFoundError:
            pass
        except ValueError as exc:
            raise ImageRepositoryError(str(exc)) from exc

        backend = self._backend
        if backend is None:
            raise ImageRepositoryError('image not found')
        try:
            payload = backend.fetch(image_id)
        except ImageNotFoundError:
            raise ImageRepositoryError(f'image not found: {image_id}') from None
        self._store.make_room(payload.size_bytes, protected)
        self._store.write(image_id, payload)
        return payload

    def _rollback(self, stored: list[str], pending_remote_id: str | None) -> list[str]:
        """Undo a failed upload; returns the ids that remain on the remote service."""
        remote_failures: list[str] = []
        if pending_remote_id is not None and not self._delete_remote(pending_remote_id):
            remote_failures.append(pending_remote_id)
        for image_id in reversed(stored):
            if self._backend is not None and not self._delete_remote(image_id):
                remote_failures.append(image_id)
            try:
                if not self._store.remove(image_id):
                    self._logger.error(f'rollback: could not remove local image {image_id}')
            except Exception as exc:
                self._logger.error(f'rollback: local removal of {image_id} failed: {exc}')
        return remote_failures

    def _delete_remote(self, image_id: str) -> bool:
        if self._backend is None:
            return True
        try:
            self._backend.delete(image_id)
        except ImageNotFoundError:
            return True
        except Exception as exc:
            self._logger.error(f'rollback: remote delete of {image_id} failed: {exc}')
            return False
        return True
