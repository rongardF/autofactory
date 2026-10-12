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
"""In-memory ``CacheBackend`` implementation for local mode.

``LocalCacheBackend`` keeps every entry in a plain ``dict`` keyed by the cache
key, guarded by a ``threading.Lock`` because service callbacks and the expiry
sweep timer run concurrently under a ``MultiThreadedExecutor``. All contents
live only in process memory; they survive deactivate/activate cycling but are
lost on cleanup/shutdown or process exit.
"""

import threading
from datetime import datetime

from repositories.interfaces.cache_backend import CacheBackend
from repositories.models.cache_entry_dto import CacheEntryDTO


class LocalCacheBackend(CacheBackend):
    """A process-local, in-memory cache backend guarded by a lock."""

    def __init__(self) -> None:
        """Initialize an empty store."""
        self._entries: dict[str, CacheEntryDTO] = {}
        self._lock = threading.Lock()

    @staticmethod
    def _is_expired(entry: CacheEntryDTO, now: datetime) -> bool:
        """Return whether ``entry`` has expired as of ``now``.

        Args:
            entry: The entry to test.
            now: The current instant from the node clock.

        Returns:
            bool: True when the entry participates in expiry and its expiry
            instant has been reached, else False.
        """
        if not entry.expire_at_valid_till or entry.valid_till is None:
            return False
        return now >= entry.valid_till

    def store(self, entry: CacheEntryDTO) -> None:
        """Store an entry, overwriting any existing entry for the same key."""
        with self._lock:
            self._entries[entry.key] = entry

    def fetch(self, key: str, now: datetime) -> tuple[CacheEntryDTO | None, bool]:
        """Look up the entry for ``key``, purging it if it has expired."""
        with self._lock:
            entry = self._entries.get(key)
            if entry is None:
                return None, False
            if self._is_expired(entry, now):
                del self._entries[key]
                return entry, True
            return entry, False

    def delete(self, key: str) -> bool:
        """Delete whatever entry exists for ``key``, regardless of its type."""
        with self._lock:
            return self._entries.pop(key, None) is not None

    def sweep_expired(self, now: datetime) -> int:
        """Proactively remove every entry that has expired as of ``now``."""
        with self._lock:
            expired_keys = [
                key
                for key, entry in self._entries.items()
                if self._is_expired(entry, now)
            ]
            for key in expired_keys:
                del self._entries[key]
            return len(expired_keys)

    def clear(self) -> None:
        """Remove all entries from the backend."""
        with self._lock:
            self._entries.clear()
