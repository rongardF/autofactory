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
"""Allowlist tests for the ``blob_repository`` ``PathPolicy``."""

import os
from pathlib import Path

import pytest

from repositories.exceptions.blob_repository import BlobRepositoryError
from repositories.services.blob_repository.path_policy import PathPolicy


@pytest.fixture
def roots(tmp_path: Path) -> dict[str, Path]:
    allowed = tmp_path / 'allowed'
    outside = tmp_path / 'outside'
    store = tmp_path / 'allowed' / 'store'
    for directory in (allowed, outside, store):
        directory.mkdir(parents=True)
    (allowed / 'file.bin').write_bytes(b'data')
    (outside / 'secret.bin').write_bytes(b'secret')
    return {'allowed': allowed, 'outside': outside, 'store': store}


@pytest.fixture
def policy(roots: dict[str, Path]) -> PathPolicy:
    return PathPolicy([str(roots['allowed'])], roots['store'])


def test_missing_root_fails_construction(tmp_path: Path) -> None:
    """Test that a non-existent allowed root is a configuration error."""
    with pytest.raises(BlobRepositoryError, match='not an existing directory'):
        PathPolicy([str(tmp_path / 'missing')], tmp_path / 'store')


def test_source_inside_root_is_accepted(policy: PathPolicy, roots: dict[str, Path]) -> None:
    """Test that a regular file under an allowed root resolves."""
    resolved = policy.resolve_source(str(roots['allowed'] / 'file.bin'))

    assert resolved == roots['allowed'] / 'file.bin'


@pytest.mark.parametrize('relative', ['../outside/secret.bin', 'missing.bin'])
def test_source_traversal_or_missing_file_is_rejected(
    policy: PathPolicy, roots: dict[str, Path], relative: str
) -> None:
    """Test that traversal out of the root and missing files are rejected."""
    with pytest.raises(BlobRepositoryError):
        policy.resolve_source(f"{roots['allowed']}/{relative}")


def test_source_relative_path_is_rejected(policy: PathPolicy) -> None:
    """Test that relative source paths are rejected."""
    with pytest.raises(BlobRepositoryError, match='absolute'):
        policy.resolve_source('file.bin')


def test_source_symlink_escape_is_rejected(policy: PathPolicy, roots: dict[str, Path]) -> None:
    """Test that a symlink inside the root pointing outside of it is rejected."""
    link = roots['allowed'] / 'escape.bin'
    link.symlink_to(roots['outside'] / 'secret.bin')

    with pytest.raises(BlobRepositoryError, match='outside the allowed path roots'):
        policy.resolve_source(str(link))


def test_source_directory_is_rejected(policy: PathPolicy, roots: dict[str, Path]) -> None:
    """Test that a directory is not a valid upload source."""
    (roots['allowed'] / 'dir').mkdir()

    with pytest.raises(BlobRepositoryError, match='regular file'):
        policy.resolve_source(str(roots['allowed'] / 'dir'))


def test_paths_inside_storage_directory_are_rejected(
    policy: PathPolicy, roots: dict[str, Path]
) -> None:
    """Test that the node's own storage directory is never a source or destination."""
    (roots['store'] / 'x').write_bytes(b'x')

    with pytest.raises(BlobRepositoryError, match='storage directory'):
        policy.resolve_source(str(roots['store'] / 'x'))
    with pytest.raises(BlobRepositoryError, match='storage directory'):
        policy.resolve_destination(str(roots['store'] / 'y'))


def test_destination_new_and_existing_regular_files_are_accepted(
    policy: PathPolicy, roots: dict[str, Path]
) -> None:
    """Test that new files and existing regular files are valid destinations."""
    for name in ('new.bin', 'file.bin'):
        assert policy.resolve_destination(str(roots['allowed'] / name)) == roots['allowed'] / name


def test_destination_outside_roots_is_rejected(
    policy: PathPolicy, roots: dict[str, Path]
) -> None:
    """Test that destinations outside the allowed roots are rejected."""
    with pytest.raises(BlobRepositoryError, match='outside the allowed path roots'):
        policy.resolve_destination(str(roots['outside'] / 'out.bin'))


def test_destination_symlink_is_rejected(policy: PathPolicy, roots: dict[str, Path]) -> None:
    """Test that an existing symlink destination is rejected, even if it points inside."""
    link = roots['allowed'] / 'link.bin'
    link.symlink_to(roots['allowed'] / 'file.bin')

    with pytest.raises(BlobRepositoryError, match='symbolic link'):
        policy.resolve_destination(str(link))


def test_destination_non_regular_entry_is_rejected(
    policy: PathPolicy, roots: dict[str, Path]
) -> None:
    """Test that an existing directory or FIFO destination is rejected."""
    (roots['allowed'] / 'dir').mkdir()
    os.mkfifo(roots['allowed'] / 'fifo')

    for name in ('dir', 'fifo'):
        with pytest.raises(BlobRepositoryError, match='not a regular file'):
            policy.resolve_destination(str(roots['allowed'] / name))


def test_destination_in_symlinked_directory_escape_is_rejected(
    policy: PathPolicy, roots: dict[str, Path]
) -> None:
    """Test that a parent-directory symlink escaping the root is rejected."""
    (roots['allowed'] / 'linkdir').symlink_to(roots['outside'])

    with pytest.raises(BlobRepositoryError, match='outside the allowed path roots'):
        policy.resolve_destination(str(roots['allowed'] / 'linkdir' / 'out.bin'))


@pytest.mark.parametrize('suffix', ['missing_dir/out.bin', ''])
def test_destination_missing_parent_or_no_name_is_rejected(
    policy: PathPolicy, roots: dict[str, Path], suffix: str
) -> None:
    """Test that a missing parent directory or a path without a file name fails."""
    with pytest.raises(BlobRepositoryError):
        policy.resolve_destination(f"{roots['allowed']}/{suffix}")
