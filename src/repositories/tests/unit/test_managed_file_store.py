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
"""Tests for the shared ``ManagedFileStore`` core used by image and blob stores."""

import os
from pathlib import Path
from unittest.mock import MagicMock
import uuid

import pytest

from repositories.services.common.managed_file_store import ManagedFileStore


def _new_id() -> str:
    return str(uuid.uuid4())


def _staged(core: ManagedFileStore, data: bytes) -> Path:
    path = core.new_staging_path()
    path.write_bytes(data)
    return path


@pytest.fixture
def store(tmp_path: Path) -> ManagedFileStore:
    core = ManagedFileStore(tmp_path / 'store', 10_000, MagicMock(), item_label='blob')
    core.prepare()
    return core


def test_commit_staged_replaces_existing_entry(store: ManagedFileStore) -> None:
    """Test that committing staged content over an id replaces content and metadata."""
    entry_id = _new_id()
    store.commit_staged(entry_id, _staged(store, b'old'), b'{"v":1}')
    store.commit_staged(entry_id, _staged(store, b'new!'), b'{"v":2}')

    assert store.content_path(entry_id).read_bytes() == b'new!'
    assert store.metadata_path(entry_id).read_bytes() == b'{"v":2}'
    assert store.entry(entry_id).size_bytes == 4 + 7


def test_commit_failure_before_move_keeps_previous_entry(
    store: ManagedFileStore, mocker
) -> None:
    """Test that a failed commit leaves the previously stored entry untouched."""
    entry_id = _new_id()
    store.commit_staged(entry_id, _staged(store, b'old'), b'{}')
    staged = _staged(store, b'new')
    mocker.patch('os.replace', side_effect=OSError('disk error'))

    with pytest.raises(OSError):
        store.commit_staged(entry_id, staged, b'{}')

    assert store.content_path(entry_id).read_bytes() == b'old'
    assert store.contains(entry_id)


def test_prepare_removes_hidden_temp_staging_and_backup_files(tmp_path: Path) -> None:
    """Test that leftovers of interrupted operations are cleaned on prepare."""
    root = tmp_path / 'store'
    core = ManagedFileStore(root, 10_000, MagicMock())
    core.prepare()
    entry_id = _new_id()
    core.commit_staged(entry_id, _staged(core, b'x'), b'{}')
    _staged(core, b'partial')
    core.detach(entry_id)

    core.prepare()

    assert list(root.iterdir()) == []


def test_prepare_drops_pairs_failing_validation(tmp_path: Path) -> None:
    """Test that the validation hook removes incomplete pairs on prepare."""
    root = tmp_path / 'store'
    good, bad = _new_id(), _new_id()
    root.mkdir()
    for entry_id in (good, bad):
        (root / entry_id).write_bytes(b'x')
        (root / f'{entry_id}_metadata.json').write_bytes(b'{}')
    core = ManagedFileStore(
        root, 10_000, MagicMock(), validate_entry=lambda entry: entry.entry_id == good
    )

    core.prepare()

    assert core.contains(good)
    assert not (root / bad).exists()


def test_detach_restore_round_trip(store: ManagedFileStore) -> None:
    """Test that a detached entry can be restored over newer content."""
    entry_id = _new_id()
    store.commit_staged(entry_id, _staged(store, b'old'), b'{}')
    detached = store.detach(entry_id)
    store.commit_staged(entry_id, _staged(store, b'new'), b'{}')

    store.restore(detached)

    assert store.content_path(entry_id).read_bytes() == b'old'
    assert not detached.content_path.exists()


def test_discard_deletes_backup_files(store: ManagedFileStore) -> None:
    """Test that discarding a detached entry removes its backup files."""
    entry_id = _new_id()
    store.commit_staged(entry_id, _staged(store, b'old'), b'{}')
    detached = store.detach(entry_id)

    assert store.discard(detached) is True
    assert list(store.root.iterdir()) == []


def test_pinned_entries_are_not_evicted(tmp_path: Path) -> None:
    """Test that pinned ids survive eviction and the shortfall is reported."""
    core = ManagedFileStore(tmp_path / 'store', 10, MagicMock())
    core.prepare()
    entry_id = _new_id()
    core.commit_staged(entry_id, _staged(core, b'x' * 20), b'{}')

    with core.pinned([entry_id]):
        assert core.make_room(0) is False
    assert core.contains(entry_id)
    assert core.make_room(0) is True
    assert not core.contains(entry_id)


def test_make_room_reports_capacity_shortfall(store: ManagedFileStore) -> None:
    """Test that make_room returns False when the request cannot fit."""
    assert store.make_room(20_000) is False


def test_eviction_is_oldest_first_then_by_id(tmp_path: Path) -> None:
    """Test deterministic oldest-first eviction with id tie-break."""
    root = tmp_path / 'store'
    entry_size = 100 + 2
    core = ManagedFileStore(root, 3 * entry_size, MagicMock())
    core.prepare()
    ids = sorted(_new_id() for _ in range(3))
    for entry_id in ids:
        core.commit_staged(entry_id, _staged(core, b'x' * 100), b'{}')
    for entry_id, seconds in zip(ids, (300, 100, 100)):
        os.utime(root / entry_id, ns=(seconds * 10**9, seconds * 10**9))
    core.prepare()

    core.make_room(entry_size)

    assert [core.contains(i) for i in ids] == [True, False, True]
