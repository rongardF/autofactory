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
"""Allowlist checks for upload source paths and fetch destination paths.

Paths are canonicalized with :func:`os.path.realpath`, so ``..`` segments and
symbolic links are resolved before the allowlist check and cannot escape the
configured roots. The node's own storage directory is never a valid source or
destination.
"""

from collections.abc import Iterable
import os
from pathlib import Path
import stat

from repositories.exceptions.blob_repository import BlobRepositoryError


class PathPolicy:
    """Resolves client paths and enforces the ``allowed_path_roots`` allowlist."""

    def __init__(self, allowed_roots: Iterable[str], storage_root: Path) -> None:
        """Canonicalize and check the allowed roots.

        Args:
            allowed_roots: Absolute directories clients may read from / write to.
            storage_root: The node's dedicated storage directory (always denied).

        Raises:
            BlobRepositoryError: If a root is relative or not an existing directory.
        """
        roots: list[Path] = []
        for raw in allowed_roots:
            if not os.path.isabs(raw):
                raise BlobRepositoryError(f"allowed path root '{raw}' is not absolute")
            resolved = Path(os.path.realpath(raw))
            if not resolved.is_dir():
                raise BlobRepositoryError(
                    f"allowed path root '{raw}' is not an existing directory"
                )
            roots.append(resolved)
        if not roots:
            raise BlobRepositoryError('allowed_path_roots must not be empty')
        self._roots = tuple(roots)
        self._storage_root = Path(os.path.realpath(storage_root))

    def resolve_source(self, raw: str) -> Path:
        """Resolve an upload source to a readable regular file inside a root.

        Raises:
            BlobRepositoryError: If the path is relative, missing, outside the
                allowed roots, not a regular file or not readable.
        """
        self._require_absolute(raw, 'source_path')
        try:
            resolved = Path(os.path.realpath(raw, strict=True))
        except OSError:
            raise BlobRepositoryError(f"source file not found: '{raw}'") from None
        self._require_allowed(resolved, raw, 'source_path')
        if not stat.S_ISREG(resolved.stat().st_mode):
            raise BlobRepositoryError(f"source_path is not a regular file: '{raw}'")
        if not os.access(resolved, os.R_OK):
            raise BlobRepositoryError(f"source file is not readable: '{raw}'")
        return resolved

    def resolve_destination(self, raw: str) -> Path:
        """Resolve a fetch destination inside a root.

        The parent directory must exist; the final component must be absent
        or an existing regular file (it is then overwritten).

        Raises:
            BlobRepositoryError: If the path is relative, has no file name, its
                directory is missing or not writable, it is outside the allowed
                roots, or it names a symlink or a non-regular entry.
        """
        self._require_absolute(raw, 'destination_path')
        name = os.path.basename(raw)
        if name in ('', '.', '..'):
            raise BlobRepositoryError(f"destination_path must name a file: '{raw}'")
        try:
            parent = Path(os.path.realpath(os.path.dirname(raw), strict=True))
        except OSError:
            raise BlobRepositoryError(
                f"destination directory does not exist: '{os.path.dirname(raw)}'"
            ) from None
        if not parent.is_dir():
            raise BlobRepositoryError(
                f"destination parent is not a directory: '{os.path.dirname(raw)}'"
            )
        destination = parent / name
        self._require_allowed(destination, raw, 'destination_path')
        check_destination_entry(destination)
        if not os.access(parent, os.W_OK | os.X_OK):
            raise BlobRepositoryError(
                f"destination directory is not writable: '{os.path.dirname(raw)}'"
            )
        return destination

    @staticmethod
    def _require_absolute(raw: str, field: str) -> None:
        if not raw or not os.path.isabs(raw):
            raise BlobRepositoryError(f"{field} must be an absolute path (got '{raw}')")

    def _require_allowed(self, resolved: Path, raw: str, field: str) -> None:
        if _is_within(resolved, self._storage_root):
            raise BlobRepositoryError(
                f"{field} must not be inside the node storage directory: '{raw}'"
            )
        if not any(_is_within(resolved, root) for root in self._roots):
            raise BlobRepositoryError(f"{field} is outside the allowed path roots: '{raw}'")


def check_destination_entry(destination: Path) -> None:
    """Reject an existing destination that is a symlink or not a regular file.

    Raises:
        BlobRepositoryError: If the entry is a symlink or a non-regular file.
    """
    try:
        mode = os.lstat(destination).st_mode
    except FileNotFoundError:
        return
    if stat.S_ISLNK(mode):
        raise BlobRepositoryError(f"destination is a symbolic link: '{destination}'")
    if not stat.S_ISREG(mode):
        raise BlobRepositoryError(f"destination exists and is not a regular file: '{destination}'")


def _is_within(path: Path, root: Path) -> bool:
    return path == root or root in path.parents
