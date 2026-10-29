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
"""Index of model files/folders cached in the node's download directory.

``ModelCache`` maps each cached ``model_id`` (a UUID4 entry name inside
``download_path``) to its absolute path. It is rebuilt from disk on configure
and updated after every successful download. All methods are thread-safe.
"""

import threading
from pathlib import Path

from repositories.utils.uuid_validator import is_uuid4


class ModelCache:
    """Thread-safe ``model_id`` -> absolute path index over ``download_path``."""

    def __init__(self, root: Path) -> None:
        """Bind the index to a download directory (not touched until used).

        Args:
            root: Absolute download directory holding cached model entries.
        """
        self._root = root
        self._entries: dict[str, Path] = {}
        self._lock = threading.Lock()

    @property
    def root(self) -> Path:
        """Path: The absolute download directory."""
        return self._root

    def ensure_root(self) -> None:
        """Create the download directory (and parents) if it does not exist.

        Raises:
            OSError: If the directory cannot be created.
        """
        self._root.mkdir(parents=True, exist_ok=True)

    def scan(self) -> int:
        """Rebuild the index from the download directory contents.

        Only direct children whose names are canonical lowercase UUID4 strings
        are indexed; everything else (e.g. ``local_storage.json`` or staging
        directories) is ignored.

        Returns:
            int: Number of indexed entries.
        """
        entries = {
            child.name: child.resolve()
            for child in self._root.iterdir()
            if is_uuid4(child.name) and child.name == child.name.lower()
        }
        with self._lock:
            self._entries = entries
            return len(self._entries)

    def target_path(self, model_id: str) -> Path:
        """Return the path at which ``model_id`` is (or will be) cached.

        Args:
            model_id: Canonical UUID4 of the model.

        Returns:
            Path: ``<download_path>/<model_id>``.
        """
        return self._root / model_id

    def get(self, model_id: str) -> Path | None:
        """Return the cached path of ``model_id`` if it still exists on disk.

        Stale entries (removed from disk externally) are dropped.

        Args:
            model_id: Canonical UUID4 of the model.

        Returns:
            Path | None: Absolute cached path, or None if not cached.
        """
        with self._lock:
            path = self._entries.get(model_id)
            if path is not None and not path.exists():
                del self._entries[model_id]
                path = None
            return path

    def register(self, model_id: str) -> Path:
        """Index a model that has just been placed in the download directory.

        Args:
            model_id: Canonical UUID4 of the model.

        Returns:
            Path: The absolute cached path.

        Raises:
            FileNotFoundError: If the model entry does not exist on disk.
        """
        path = self.target_path(model_id)
        if not path.exists():
            raise FileNotFoundError(f"cached model '{path}' does not exist")
        resolved = path.resolve()
        with self._lock:
            self._entries[model_id] = resolved
        return resolved

    def model_ids(self) -> list[str]:
        """Return the sorted ids of all indexed models.

        Returns:
            list[str]: Cached model ids.
        """
        with self._lock:
            return sorted(self._entries)

    def clear(self) -> None:
        """Drop every index entry (files on disk are left untouched)."""
        with self._lock:
            self._entries = {}
