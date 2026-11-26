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
"""Behavior tests for ``BlobManager`` with the local store and a fake cloud backend."""

from datetime import datetime, timezone
import hashlib
from pathlib import Path
from typing import BinaryIO
from unittest.mock import MagicMock
import uuid

import pytest

from repositories.exceptions.blob_repository import (
    BlobNotFoundError,
    BlobOperationCanceledError,
    BlobRepositoryError,
)
from repositories.interfaces.blob_repository.blob_backend import BlobBackend
from repositories.models.blob_repository.blob_descriptor_dto import (
    BlobDescriptorDTO,
    BlobUploadDescriptorDTO,
    BlobUploadItemDTO,
)
from repositories.services.blob_repository import streaming
from repositories.services.blob_repository.blob_manager import BlobManager
from repositories.services.blob_repository.local_blob_backend import LocalBlobBackend
from repositories.services.blob_repository.local_blob_store import LocalBlobStore
from repositories.services.blob_repository.path_policy import PathPolicy

MIB = 1024 * 1024


class FakeCloud(BlobBackend):
    """In-memory cloud with failure injection (content stays small in tests)."""

    def __init__(self) -> None:
        self.blobs: dict[str, tuple[bytes, BlobDescriptorDTO]] = {}
        self.fail_upload_calls: set[int] = set()
        self.fail_rollback_uploads = False
        self.fail_deletes = False
        self.fail_downloads = False
        self.corrupt_downloads = False
        self.upload_calls = 0
        self.download_calls = 0
        self.deleted: list[str] = []

    @property
    def is_local(self) -> bool:
        return False

    def initialize(self) -> None:
        return None

    def shutdown(self) -> None:
        return None

    def upload(
        self,
        content_path: Path,
        descriptor: BlobUploadDescriptorDTO,
        blob_id: str | None = None,
        replaced: BlobDescriptorDTO | None = None,
    ) -> BlobDescriptorDTO:
        self.upload_calls += 1
        if self.upload_calls in self.fail_upload_calls:
            raise BlobRepositoryError('cloud upload failed: service error (HTTP 500)')
        if self.fail_rollback_uploads and replaced is None and blob_id in self.blobs:
            raise BlobRepositoryError('cloud upload failed: service error (HTTP 503)')
        now = datetime.now(timezone.utc)
        stored = BlobDescriptorDTO(
            **descriptor.model_dump(),
            blob_id=blob_id or str(uuid.uuid4()),
            created_at=now,
            updated_at=now,
        )
        self.blobs[stored.blob_id] = (content_path.read_bytes(), stored)
        return stored

    def download(self, blob_id: str, sink: BinaryIO, max_bytes: int) -> None:
        self.download_calls += 1
        if self.fail_downloads:
            raise BlobRepositoryError('cloud download failed: ConnectionError')
        data = self._get(blob_id)[0]
        if self.corrupt_downloads:
            data = bytes([data[0] ^ 0xFF]) + data[1:]
        sink.write(data)

    def info(self, blob_id: str) -> BlobDescriptorDTO:
        return self._get(blob_id)[1]

    def delete(self, blob_id: str) -> None:
        if self.fail_deletes:
            raise BlobRepositoryError('cloud delete failed: service error (HTTP 500)')
        self.deleted.append(blob_id)
        self.blobs.pop(blob_id, None)

    def _get(self, blob_id: str) -> tuple[bytes, BlobDescriptorDTO]:
        if blob_id not in self.blobs:
            raise BlobNotFoundError(f'blob not found: {blob_id}')
        return self.blobs[blob_id]


class Env:
    """A data directory plus a manager in local or cloud mode."""

    def __init__(self, tmp_path: Path, cloud: FakeCloud | None, capacity: int, max_size: int):
        self.data = tmp_path / 'data'
        self.data.mkdir()
        self.store = LocalBlobStore(tmp_path / 'store', capacity, MagicMock())
        self.store.prepare()
        self.cloud = cloud
        backend = cloud if cloud is not None else LocalBlobBackend(self.store)
        self.logger = MagicMock()
        self.manager = BlobManager(
            self.store,
            backend,
            PathPolicy([str(self.data)], self.store.root),
            max_size,
            self.logger,
        )

    def file(self, name: str, content: bytes) -> str:
        path = self.data / name
        path.write_bytes(content)
        return str(path)

    def upload(self, name: str, content: bytes, **fields) -> BlobDescriptorDTO:
        return self.manager.upload([self.item(name, content, **fields)])[0]

    def item(self, name: str, content: bytes, **fields) -> BlobUploadItemDTO:
        return BlobUploadItemDTO.from_request(self.file(name, content), **fields)


@pytest.fixture
def local(tmp_path: Path) -> Env:
    return Env(tmp_path, None, 10 * MIB, 5 * MIB)


@pytest.fixture
def cloud(tmp_path: Path) -> Env:
    return Env(tmp_path, FakeCloud(), 10 * MIB, 5 * MIB)


# region: local mode


def test_local_upload_fetch_info_delete_round_trip(local: Env) -> None:
    """Test the local-mode lifecycle of one blob, byte for byte."""
    content = b'\x00\x01payload' * 100
    stored = local.upload('a.bin', content, metadata='{"k": [1, 2]}')

    fetched = local.manager.fetch([stored.blob_id], [str(local.data / 'out.bin')])[0]

    assert (local.data / 'out.bin').read_bytes() == content
    assert fetched.descriptor.sha256 == hashlib.sha256(content).hexdigest()
    info = local.manager.info(stored.blob_id)
    assert info.blob_name == 'a.bin'
    assert (info.size_bytes, info.metadata) == (len(content), {'k': [1, 2]})
    local.manager.delete(stored.blob_id)
    with pytest.raises(BlobNotFoundError):
        local.manager.info(stored.blob_id)


def test_local_fetch_without_destination_returns_store_path(local: Env) -> None:
    """Test that an empty destination returns the managed local path."""
    stored = local.upload('a.bin', b'abc')

    result = local.manager.fetch([stored.blob_id], [])[0]

    assert result.local_path == local.store.content_path(stored.blob_id)


def test_upload_defaults_name_and_infers_media_type(local: Env) -> None:
    """Test basename and media type defaults, with octet-stream fallback."""
    json_blob = local.upload('doc.json', b'{}')
    raw_blob = local.upload('noext', b'x')

    assert (json_blob.blob_name, json_blob.media_type) == ('doc.json', 'application/json')
    assert raw_blob.media_type == 'application/octet-stream'


def test_supplied_id_overwrites_content_and_keeps_created_at(local: Env) -> None:
    """Test that a supplied UUID4 replaces content and descriptor (upsert)."""
    first = local.upload('a.bin', b'one')
    second = local.upload('b.bin', b'two!', blob_id=first.blob_id)

    assert second.blob_id == first.blob_id
    assert second.created_at == first.created_at
    assert local.store.content_path(first.blob_id).read_bytes() == b'two!'
    assert local.manager.info(first.blob_id).blob_name == 'b.bin'


def test_supplied_id_for_unknown_blob_creates_it(local: Env) -> None:
    """Test that a supplied, unused UUID4 creates a new blob under that id."""
    blob_id = str(uuid.uuid4())

    assert local.upload('a.bin', b'x', blob_id=blob_id).blob_id == blob_id


def test_download_fresh_is_ignored_in_local_mode(local: Env) -> None:
    """Test that download_fresh has no effect in local mode."""
    stored = local.upload('a.bin', b'abc')

    assert local.manager.fetch([stored.blob_id], [], download_fresh=True)[0].descriptor == stored


def test_delete_absent_blob_succeeds_and_invalid_id_fails(local: Env) -> None:
    """Test delete semantics for absent and malformed ids."""
    local.manager.delete(str(uuid.uuid4()))

    with pytest.raises(BlobRepositoryError, match='UUID4'):
        local.manager.delete('nope')


def test_too_large_file_is_rejected_before_transfer(tmp_path: Path) -> None:
    """Test that files above max_blob_size are rejected without staging anything."""
    env = Env(tmp_path, None, 10 * MIB, 1000)

    with pytest.raises(BlobRepositoryError, match='too large'):
        env.upload('big.bin', b'x' * 1001)
    assert list(env.store.root.iterdir()) == []


def test_file_larger_than_cache_capacity_is_rejected(tmp_path: Path) -> None:
    """Test that a file that cannot fit in the cache is rejected up front."""
    env = Env(tmp_path, None, 2000, 2000)

    with pytest.raises(BlobRepositoryError, match='does not fit'):
        env.upload('big.bin', b'x' * 1900)


def test_insufficient_space_fails_when_nothing_is_evictable(tmp_path: Path) -> None:
    """Test a clear failure when protected blobs leave no room in the batch."""
    env = Env(tmp_path, None, 6000, 3000)
    items = [env.item(f'{i}.bin', b'x' * 2500) for i in range(3)]

    with pytest.raises(BlobRepositoryError, match='insufficient local cache space'):
        env.manager.upload(items)
    assert env.store.ids() == []


def test_local_mode_evicts_oldest_blob_permanently(tmp_path: Path) -> None:
    """Test that local-mode eviction deletes the oldest blob to make room."""
    env = Env(tmp_path, None, 6000, 3000)
    first = env.upload('1.bin', b'x' * 2500)
    second = env.upload('2.bin', b'y' * 2500)

    env.upload('3.bin', b'z' * 2500)

    assert not env.store.contains(first.blob_id)
    assert env.store.contains(second.blob_id)
    assert env.store.directory_size_bytes() <= 6000


def test_upload_streams_in_bounded_chunks(local: Env, mocker) -> None:
    """Test that uploads copy through bounded chunks, never the whole file at once."""
    mocker.patch.object(streaming, 'CHUNK_SIZE', 1024)
    write_spy = mocker.spy(streaming.HashingWriter, 'write')

    local.upload('a.bin', b'q' * 10_000)

    sizes = [len(call.args[1]) for call in write_spy.call_args_list]
    assert max(sizes) <= 1024 and sum(sizes) == 10_000


def test_batch_upload_rolls_back_new_and_replaced_blobs(local: Env, mocker) -> None:
    """Test all-or-nothing local batches: new blobs removed, replaced ones restored."""
    existing = local.upload('old.bin', b'original')
    real_commit = local.store.commit
    calls: list[int] = []

    def failing_third_commit(staged: Path, descriptor: BlobDescriptorDTO) -> None:
        calls.append(1)
        if len(calls) == 3:
            raise OSError('disk full')
        real_commit(staged, descriptor)

    mocker.patch.object(local.store, 'commit', side_effect=failing_third_commit)
    items = [
        local.item('new.bin', b'new'),
        local.item('replace.bin', b'replacement', blob_id=existing.blob_id),
        local.item('third.bin', b'third'),
    ]

    with pytest.raises(BlobRepositoryError, match='item 2: disk full'):
        local.manager.upload(items)

    assert local.store.ids() == [existing.blob_id]
    assert local.store.content_path(existing.blob_id).read_bytes() == b'original'
    assert local.manager.info(existing.blob_id) == existing
    assert [p.name for p in local.store.root.iterdir() if p.name.startswith('.')] == []


def test_batch_upload_reports_ordered_progress(local: Env) -> None:
    """Test that progress is reported once per item, in request order."""
    progress: list[tuple[int, str]] = []
    items = [local.item(f'{i}.bin', bytes([i])) for i in range(3)]

    stored = local.manager.upload(items, lambda i, item, d: progress.append((i, d.blob_id)))

    assert progress == [(i, d.blob_id) for i, d in enumerate(stored)]


def test_batch_upload_cancellation_rolls_back(local: Env) -> None:
    """Test that cancellation between items rolls back already stored items."""
    items = [local.item(f'{i}.bin', b'x') for i in range(3)]
    done: list[int] = []

    with pytest.raises(BlobOperationCanceledError, match='canceled after 1 of 3'):
        local.manager.upload(
            items, lambda i, item, d: done.append(i), should_cancel=lambda: len(done) == 1
        )

    assert local.store.ids() == []


def test_duplicate_id_in_canceled_batch_rolls_back_to_original(local: Env) -> None:
    """Test that a batch writing one id twice rolls back to the original content."""
    existing = local.upload('old.bin', b'v0')
    items = [
        local.item('v1.bin', b'v1', blob_id=existing.blob_id),
        local.item('v2.bin', b'v2', blob_id=existing.blob_id),
    ]

    with pytest.raises(BlobOperationCanceledError):
        local.manager.upload(
            items,
            should_cancel=lambda: local.store.content_path(existing.blob_id).read_bytes() == b'v1',
        )

    assert local.store.content_path(existing.blob_id).read_bytes() == b'v0'


def test_batch_fetch_is_fail_fast_and_keeps_earlier_results(local: Env) -> None:
    """Test that a later unknown id fails the batch while earlier copies remain."""
    stored = local.upload('a.bin', b'abc')
    destinations = [str(local.data / 'first.bin'), str(local.data / 'second.bin')]

    with pytest.raises(BlobNotFoundError, match='item 1'):
        local.manager.fetch([stored.blob_id, str(uuid.uuid4())], destinations)

    assert (local.data / 'first.bin').read_bytes() == b'abc'
    assert not (local.data / 'second.bin').exists()


def test_batch_fetch_validates_everything_before_side_effects(local: Env) -> None:
    """Test that invalid ids, bad destinations or mismatched arrays fail before work."""
    stored = local.upload('a.bin', b'abc')
    first = str(local.data / 'first.bin')

    with pytest.raises(BlobRepositoryError, match='same length'):
        local.manager.fetch([stored.blob_id], [first, first])
    with pytest.raises(BlobRepositoryError, match='UUID4'):
        local.manager.fetch([stored.blob_id, 'bad'], [])
    with pytest.raises(BlobRepositoryError, match='outside the allowed'):
        local.manager.fetch([stored.blob_id, stored.blob_id], [first, '/etc/out.bin'])
    assert not Path(first).exists()


def test_batch_fetch_cancellation_between_items(local: Env) -> None:
    """Test that cancellation stops a fetch batch between items."""
    stored = local.upload('a.bin', b'abc')
    progress: list[int] = []

    with pytest.raises(BlobOperationCanceledError):
        local.manager.fetch(
            [stored.blob_id] * 3, [], on_progress=lambda i, b, r: progress.append(i),
            should_cancel=lambda: len(progress) == 2,
        )
    assert progress == [0, 1]


def test_fetch_overwrites_existing_destination(local: Env) -> None:
    """Test that an existing regular destination file is replaced on success."""
    stored = local.upload('a.bin', b'new content')
    target = local.data / 'target.bin'
    target.write_bytes(b'previous')

    local.manager.fetch([stored.blob_id], [str(target)])

    assert target.read_bytes() == b'new content'
    assert [p.name for p in local.data.iterdir() if p.name.startswith('.')] == []


def test_failed_verification_leaves_destination_unchanged(local: Env) -> None:
    """Test that a corrupted source copy does not replace the destination."""
    stored = local.upload('a.bin', b'good content')
    local.store.content_path(stored.blob_id).write_bytes(b'evil content')
    target = local.data / 'target.bin'
    target.write_bytes(b'previous')

    with pytest.raises(BlobRepositoryError, match='checksum mismatch'):
        local.manager.fetch([stored.blob_id], [str(target)])

    assert target.read_bytes() == b'previous'
    assert [p.name for p in local.data.iterdir() if p.name.startswith('.')] == []


# endregion: local mode

# region: cloud mode


def test_cloud_upload_writes_cloud_then_cache(cloud: Env) -> None:
    """Test that cloud mode stores remotely and keeps a cached copy."""
    stored = cloud.upload('a.bin', b'abc')

    assert stored.blob_id in cloud.cloud.blobs
    assert cloud.store.content_path(stored.blob_id).read_bytes() == b'abc'


def test_cloud_cache_hit_does_not_contact_cloud(cloud: Env) -> None:
    """Test that a cached blob is served without a download."""
    stored = cloud.upload('a.bin', b'abc')

    cloud.manager.fetch([stored.blob_id], [])

    assert cloud.cloud.download_calls == 0


def test_cloud_cache_miss_downloads_and_caches(cloud: Env) -> None:
    """Test that a cache miss downloads, verifies and caches the blob."""
    stored = cloud.upload('a.bin', b'abc')
    cloud.store.remove(stored.blob_id)

    result = cloud.manager.fetch([stored.blob_id], [str(cloud.data / 'out.bin')])[0]

    assert cloud.cloud.download_calls == 1
    assert (cloud.data / 'out.bin').read_bytes() == b'abc'
    assert cloud.store.contains(stored.blob_id)
    assert result.descriptor.sha256 == stored.sha256


def test_cloud_download_fresh_redownloads_on_cache_hit(cloud: Env) -> None:
    """Test that download_fresh forces a download and refreshes the cache."""
    stored = cloud.upload('a.bin', b'abc')
    content, descriptor = cloud.cloud.blobs[stored.blob_id]
    newer = descriptor.model_copy(
        update={'size_bytes': 4, 'sha256': hashlib.sha256(b'abcd').hexdigest()}
    )
    cloud.cloud.blobs[stored.blob_id] = (b'abcd', newer)

    result = cloud.manager.fetch([stored.blob_id], [], download_fresh=True)[0]

    assert cloud.cloud.download_calls == 1
    assert result.local_path.read_bytes() == b'abcd'
    assert cloud.store.descriptor(stored.blob_id).size_bytes == 4


def test_cloud_download_fresh_failure_keeps_cached_copy(cloud: Env) -> None:
    """Test that a failed fresh download fails without touching the cached copy."""
    stored = cloud.upload('a.bin', b'abc')
    cloud.cloud.fail_downloads = True

    with pytest.raises(BlobRepositoryError, match='download failed'):
        cloud.manager.fetch([stored.blob_id], [], download_fresh=True)

    assert cloud.store.content_path(stored.blob_id).read_bytes() == b'abc'
    assert cloud.store.descriptor(stored.blob_id) == stored


def test_cloud_checksum_mismatch_discards_download(cloud: Env) -> None:
    """Test that a corrupted download is rejected and not cached."""
    stored = cloud.upload('a.bin', b'abc')
    cloud.store.remove(stored.blob_id)
    cloud.cloud.corrupt_downloads = True

    with pytest.raises(BlobRepositoryError, match='checksum mismatch'):
        cloud.manager.fetch([stored.blob_id], [])

    assert not cloud.store.contains(stored.blob_id)
    assert list(cloud.store.root.iterdir()) == []


def test_cloud_batch_failure_deletes_new_remote_blobs(cloud: Env) -> None:
    """Test that a failed cloud batch deletes remotely created blobs and cache copies."""
    cloud.cloud.fail_upload_calls = {2}

    with pytest.raises(BlobRepositoryError, match='item 1'):
        cloud.manager.upload([cloud.item('a.bin', b'a'), cloud.item('b.bin', b'b')])

    assert cloud.cloud.blobs == {}
    assert cloud.store.ids() == []


def test_cloud_overwrite_rollback_restores_previous_remote_and_cache(cloud: Env) -> None:
    """Test that a rolled back cloud overwrite puts the previous content back."""
    existing = cloud.upload('old.bin', b'original')
    cloud.cloud.fail_upload_calls = {3}
    items = [
        cloud.item('new.bin', b'replacement', blob_id=existing.blob_id),
        cloud.item('other.bin', b'other'),
    ]

    with pytest.raises(BlobRepositoryError):
        cloud.manager.upload(items)

    assert cloud.cloud.blobs[existing.blob_id][0] == b'original'
    assert cloud.store.content_path(existing.blob_id).read_bytes() == b'original'


def test_cloud_overwrite_of_uncached_blob_downloads_backup(cloud: Env) -> None:
    """Test that an uncached overwritten blob is downloaded as a restorable backup."""
    existing = cloud.upload('old.bin', b'original')
    cloud.store.remove(existing.blob_id)
    cloud.cloud.fail_upload_calls = {3}

    with pytest.raises(BlobRepositoryError):
        cloud.manager.upload([
            cloud.item('new.bin', b'replacement', blob_id=existing.blob_id),
            cloud.item('other.bin', b'other'),
        ])

    assert cloud.cloud.download_calls == 1
    assert cloud.cloud.blobs[existing.blob_id][0] == b'original'
    assert [p.name for p in cloud.store.root.iterdir() if p.name.startswith('.')] == []


def test_cloud_rollback_failure_is_reported_with_ids(cloud: Env) -> None:
    """Test that remote rollback failures name the affected ids and are logged."""
    cloud.cloud.fail_upload_calls = {2}
    cloud.cloud.fail_deletes = True

    with pytest.raises(BlobRepositoryError, match='rollback failed for blob ids') as info:
        cloud.manager.upload([cloud.item('a.bin', b'a'), cloud.item('b.bin', b'b')])

    remaining = next(iter(cloud.cloud.blobs))
    assert remaining in str(info.value)
    cloud.logger.error.assert_called()


def test_cloud_batch_commit_discards_backups(cloud: Env) -> None:
    """Test that a committed overwrite leaves no hidden backup files behind."""
    existing = cloud.upload('old.bin', b'original')

    cloud.upload('new.bin', b'replacement', blob_id=existing.blob_id)

    assert cloud.cloud.blobs[existing.blob_id][0] == b'replacement'
    assert [p.name for p in cloud.store.root.iterdir() if p.name.startswith('.')] == []


def test_cloud_delete_removes_remote_and_cache(cloud: Env) -> None:
    """Test that cloud delete removes the remote blob and the cached copy."""
    stored = cloud.upload('a.bin', b'abc')

    cloud.manager.delete(stored.blob_id)

    assert stored.blob_id not in cloud.cloud.blobs
    assert not cloud.store.contains(stored.blob_id)


def test_cloud_eviction_keeps_remote_copy(tmp_path: Path) -> None:
    """Test that cloud-mode eviction only drops the cached copy."""
    env = Env(tmp_path, FakeCloud(), 6000, 3000)
    first = env.upload('1.bin', b'x' * 2500)
    env.upload('2.bin', b'y' * 2500)
    env.upload('3.bin', b'z' * 2500)

    assert not env.store.contains(first.blob_id)
    assert first.blob_id in env.cloud.blobs


# endregion: cloud mode
