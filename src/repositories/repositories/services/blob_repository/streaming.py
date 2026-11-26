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
"""Bounded-memory streaming helpers: hashing writer, deadlines, staged files."""

from collections.abc import Callable
import hashlib
import os
from pathlib import Path
import time
from typing import BinaryIO

from repositories.exceptions.blob_repository import BlobRepositoryError

CHUNK_SIZE = 1024 * 1024


class TransferDeadline:
    """Total-time budget for one streamed transfer."""

    def __init__(self, timeout_sec: float) -> None:
        """Start the budget now."""
        self._timeout_sec = timeout_sec
        self._end = time.monotonic() + timeout_sec

    def check(self, what: str = 'transfer') -> None:
        """Raise if the budget is exhausted.

        Raises:
            BlobRepositoryError: If the deadline has passed.
        """
        if time.monotonic() > self._end:
            raise BlobRepositoryError(
                f'{what} exceeded transfer_timeout_sec ({self._timeout_sec:g} s)'
            )


class HashingWriter:
    """Writable sink that counts and SHA-256 hashes everything written to it."""

    def __init__(self, target: BinaryIO, max_bytes: int) -> None:
        """Wrap ``target``; writing more than ``max_bytes`` raises."""
        self._target = target
        self._max_bytes = max_bytes
        self._hash = hashlib.sha256()
        self._size = 0

    def write(self, data: bytes) -> int:
        """Hash and forward ``data``.

        Raises:
            BlobRepositoryError: If the total exceeds ``max_bytes``.
        """
        self._size += len(data)
        if self._size > self._max_bytes:
            raise BlobRepositoryError(
                f'content is larger than the expected {self._max_bytes} bytes'
            )
        self._hash.update(data)
        return self._target.write(data)

    @property
    def size_bytes(self) -> int:
        """Return the number of bytes written so far."""
        return self._size

    @property
    def sha256(self) -> str:
        """Lowercase hex SHA-256 of everything written so far."""
        return self._hash.hexdigest()


def copy_stream(source: BinaryIO, sink: BinaryIO | HashingWriter) -> None:
    """Copy ``source`` into ``sink`` in :data:`CHUNK_SIZE` chunks."""
    while chunk := source.read(CHUNK_SIZE):
        sink.write(chunk)


def write_new_file(
    path: Path, max_bytes: int, fill: Callable[[HashingWriter], None]
) -> HashingWriter:
    """Create ``path`` exclusively, let ``fill`` stream into it, then fsync.

    Args:
        path: File to create; it must not exist.
        max_bytes: Upper bound for the number of bytes written.
        fill: Callback writing the content into the given hashing writer.

    Returns:
        HashingWriter: The writer, exposing the final size and SHA-256.

    Raises:
        Exception: Whatever ``fill`` or the filesystem raises; ``path`` is then removed.
    """
    try:
        with open(path, 'xb') as handle:
            writer = HashingWriter(handle, max_bytes)
            fill(writer)
            handle.flush()
            os.fsync(handle.fileno())
        return writer
    except BaseException:
        path.unlink(missing_ok=True)
        raise


def verify(writer: HashingWriter, size_bytes: int, sha256: str, what: str) -> None:
    """Compare what was written with the expected size and checksum.

    Raises:
        BlobRepositoryError: On a size or SHA-256 mismatch.
    """
    if writer.size_bytes != size_bytes:
        raise BlobRepositoryError(
            f'{what}: size mismatch (expected {size_bytes} bytes, got {writer.size_bytes})'
        )
    if writer.sha256 != sha256:
        raise BlobRepositoryError(f'{what}: checksum mismatch')
