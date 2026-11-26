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
"""Behavior tests for ``LocalImageStore`` and ``ImageManager``.

These pin the image repository behavior that must survive the shared store
core refactor: layout, cleanup, indexing, eviction order and protection,
non-failing eviction, write rollback and upload rollback.
"""

import os
from pathlib import Path
from unittest.mock import MagicMock
import uuid

import pytest

from repositories.exceptions.image_repository import (
    ImageNotFoundError,
    ImageRepositoryError,
)
from repositories.interfaces.image_repository.image_backend import ImageBackend
from repositories.models.image_repository.image_payload_dto import (
    ImageInfoDTO,
    ImagePayloadDTO,
)
from repositories.services.image_repository.image_manager import ImageManager
from repositories.services.image_repository.local_image_store import LocalImageStore


def _payload(size: int = 100, name: str = 'img') -> ImagePayloadDTO:
    return ImagePayloadDTO(
        png_bytes=b'x' * size,
        image_name=name,
        metadata={'k': 1},
        info=ImageInfoDTO(encoding='mono8', width=1, height=1),
    )


def _new_id() -> str:
    return str(uuid.uuid4())


def _set_mtime(store_root: Path, image_id: str, seconds: int) -> None:
    os.utime(store_root / image_id, ns=(seconds * 10**9, seconds * 10**9))


@pytest.fixture
def logger() -> MagicMock:
    return MagicMock()


@pytest.fixture
def store(tmp_path: Path, logger: MagicMock) -> LocalImageStore:
    image_store = LocalImageStore(tmp_path / 'images', 10_000_000, logger)
    image_store.prepare()
    return image_store


def test_prepare_removes_stray_and_orphan_files_but_keeps_pairs(
    tmp_path: Path, logger: MagicMock
) -> None:
    """Test that prepare deletes unrelated/orphan/temp files and indexes complete pairs."""
    root = tmp_path / 'images'
    root.mkdir()
    paired, orphan_image, orphan_meta = _new_id(), _new_id(), _new_id()
    (root / paired).write_bytes(b'png')
    (root / f'{paired}_metadata.json').write_bytes(_payload().metadata_file_bytes())
    (root / orphan_image).write_bytes(b'png')
    (root / f'{orphan_meta}_metadata.json').write_bytes(b'{}')
    (root / 'unrelated.txt').write_bytes(b'x')
    (root / f'.{paired}.tmp').write_bytes(b'partial')
    (root / 'subdir').mkdir()
    (root / 'link').symlink_to(root / paired)

    image_store = LocalImageStore(root, 10_000_000, logger)
    image_store.prepare()

    remaining = sorted(p.name for p in root.iterdir())
    assert remaining == sorted([paired, f'{paired}_metadata.json', 'subdir', 'link'])
    assert image_store.contains(paired)
    assert not image_store.contains(orphan_image)


def test_write_then_read_round_trips_payload(store: LocalImageStore) -> None:
    """Test that a written image is readable with identical content and metadata."""
    image_id = _new_id()
    payload = _payload(name='front')
    store.write(image_id, payload)

    assert store.read(image_id) == payload


def test_write_replaces_existing_image(store: LocalImageStore) -> None:
    """Test that writing to an existing id replaces its content."""
    image_id = _new_id()
    store.write(image_id, _payload(10, 'old'))
    store.write(image_id, _payload(20, 'new'))

    assert store.read(image_id).image_name == 'new'
    assert len(store.read(image_id).png_bytes) == 20


def test_read_unknown_id_raises_not_found(store: LocalImageStore) -> None:
    """Test that reading an unknown id raises ImageNotFoundError."""
    with pytest.raises(ImageNotFoundError):
        store.read(_new_id())


def test_read_invalid_id_raises_value_error(store: LocalImageStore) -> None:
    """Test that a non-UUID4 id is rejected."""
    with pytest.raises(ValueError):
        store.read('not-a-uuid')


def test_write_failure_removes_partial_files(
    store: LocalImageStore, tmp_path: Path, mocker
) -> None:
    """Test that a failed metadata write leaves neither file nor index entry behind."""
    image_id = _new_id()
    real_replace = os.replace

    def failing_replace(src, dst):
        if str(dst).endswith('_metadata.json'):
            raise OSError('disk full')
        return real_replace(src, dst)

    mocker.patch('os.replace', side_effect=failing_replace)
    with pytest.raises(OSError):
        store.write(image_id, _payload())

    assert not store.contains(image_id)
    assert list((tmp_path / 'images').iterdir()) == []


def test_remove_deletes_both_files(store: LocalImageStore, tmp_path: Path) -> None:
    """Test that remove deletes the image and metadata files and the index entry."""
    image_id = _new_id()
    store.write(image_id, _payload())

    assert store.remove(image_id) is True
    assert not store.contains(image_id)
    assert list((tmp_path / 'images').iterdir()) == []


def _pair_size(size: int = 1000) -> int:
    payload = _payload(size)
    return len(payload.png_bytes) + len(payload.metadata_file_bytes())


def test_make_room_evicts_oldest_first_with_id_tiebreak(
    tmp_path: Path, logger: MagicMock
) -> None:
    """Test eviction order: oldest mtime first, then id for equal mtimes."""
    root = tmp_path / 'images'
    image_store = LocalImageStore(root, 3 * _pair_size(), logger)
    image_store.prepare()
    ids = sorted(_new_id() for _ in range(3))
    for image_id in ids:
        image_store.write(image_id, _payload(1000))
    _set_mtime(root, ids[0], 200)
    _set_mtime(root, ids[1], 100)
    _set_mtime(root, ids[2], 100)
    image_store.prepare()  # re-index with the new mtimes; last stored is reset

    image_store.make_room(_pair_size())

    assert not image_store.contains(ids[1])
    assert image_store.contains(ids[2])
    assert image_store.contains(ids[0])


def test_make_room_skips_protected_and_last_stored(
    tmp_path: Path, logger: MagicMock
) -> None:
    """Test that protected ids and the most recently stored id are never evicted."""
    image_store = LocalImageStore(tmp_path / 'images', 0, logger)
    image_store.prepare()
    protected, last = _new_id(), _new_id()
    image_store.write(protected, _payload(1000))
    image_store.write(last, _payload(1000))

    image_store.make_room(0, frozenset({protected}))

    assert image_store.contains(protected)
    assert image_store.contains(last)
    logger.warning.assert_called()


def test_make_room_never_raises_when_space_is_insufficient(
    tmp_path: Path, logger: MagicMock
) -> None:
    """Test that a lack of evictable space only logs a warning."""
    image_store = LocalImageStore(tmp_path / 'images', 10, logger)
    image_store.prepare()

    image_store.make_room(1_000_000)

    assert 'image directory' in logger.warning.call_args[0][0]
    assert 'above the cache threshold' in logger.warning.call_args[0][0]


def test_enforce_limit_evicts_until_below_threshold(
    tmp_path: Path, logger: MagicMock
) -> None:
    """Test that enforce_limit evicts old pairs once the directory exceeds the threshold."""
    root = tmp_path / 'images'
    image_store = LocalImageStore(root, 2 * _pair_size() - 1, logger)
    image_store.prepare()
    old, new = _new_id(), _new_id()
    image_store.write(old, _payload(1000))
    image_store.write(new, _payload(1000))
    _set_mtime(root, old, 1)

    image_store.enforce_limit()

    assert not image_store.contains(old)
    assert image_store.contains(new)


class _FailingSecondUploadBackend(ImageBackend):
    def __init__(self) -> None:
        self.uploaded: list[str] = []
        self.deleted: list[str] = []

    def initialize(self) -> None:
        return None

    def shutdown(self) -> None:
        return None

    def upload(self, payload: ImagePayloadDTO) -> str:
        if self.uploaded:
            raise RuntimeError('remote down')
        image_id = _new_id()
        self.uploaded.append(image_id)
        return image_id

    def fetch(self, image_id: str) -> ImagePayloadDTO:
        raise ImageNotFoundError(image_id)

    def delete(self, image_id: str) -> None:
        self.deleted.append(image_id)


def test_manager_upload_rolls_back_local_and_remote_on_failure(
    store: LocalImageStore, logger: MagicMock
) -> None:
    """Test that a failed batch upload removes earlier local and remote images."""
    backend = _FailingSecondUploadBackend()
    manager = ImageManager(store, backend, logger)

    with pytest.raises(ImageRepositoryError, match='remote down'):
        manager.upload([_payload(), _payload()])

    assert backend.deleted == backend.uploaded
    assert not store.contains(backend.uploaded[0])


def test_manager_local_fetch_of_unknown_id_fails(
    store: LocalImageStore, logger: MagicMock
) -> None:
    """Test that local mode reports a missing image as an ImageRepositoryError."""
    manager = ImageManager(store, None, logger)

    with pytest.raises(ImageRepositoryError, match='image not found'):
        manager.fetch([_new_id()])


def test_manager_local_upload_then_fetch(store: LocalImageStore, logger: MagicMock) -> None:
    """Test a local-mode upload/fetch round trip through the manager."""
    manager = ImageManager(store, None, logger)
    payload = _payload(name='a')

    image_ids = manager.upload([payload])

    assert manager.fetch(image_ids) == [payload]
