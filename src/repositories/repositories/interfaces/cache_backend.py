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
"""Abstract storage backend contract for the ``station_cache`` node.

A backend owns the actual storage of cache entries scoped to one
``station_id``. The node translates ROS service requests into DTO operations
and delegates every storage concern to a backend instance, so swapping local
in-memory storage for the (future) cloud REST store means writing one subclass
without touching the service surface.
"""

from abc import ABC, abstractmethod
from datetime import datetime

from repositories.models.cache_entry_dto import CacheEntryDTO


class CacheBackend(ABC):
    """Contract between the ``station_cache`` node and a storage backend."""

    @abstractmethod
    def store(self, entry: CacheEntryDTO) -> None:
        """Store an entry, overwriting any existing entry for the same key.

        Keys are globally unique within the station: storing silently replaces
        any existing entry for ``entry.key`` regardless of its previous type,
        value, or expiry.

        Args:
            entry: The cache entry to persist.

        Returns:
            None.
        """

    @abstractmethod
    def fetch(self, key: str, now: datetime) -> tuple[CacheEntryDTO | None, bool]:
        """Look up the entry for ``key``, purging it if it has expired.

        Applies the lazy-expiry rule: if an entry exists but has expired as of
        ``now``, it is removed from storage. The purged entry is still returned
        alongside ``expired=True`` so the caller can inspect its stored type
        (type discrimination is the node's responsibility, not the backend's).

        Args:
            key: The key to look up.
            now: The current instant from the node clock, used for the expiry
                check.

        Returns:
            tuple[CacheEntryDTO | None, bool]: ``(entry, expired)``.
            ``(None, False)`` when no entry exists; ``(entry, False)`` when a
            live entry exists; ``(entry, True)`` when an entry existed but had
            expired (and was purged). The entry's type is **not** checked here.
        """

    @abstractmethod
    def delete(self, key: str) -> bool:
        """Delete whatever entry exists for ``key``, regardless of its type.

        Args:
            key: The key to remove.

        Returns:
            bool: True if a live entry existed and was removed, False if the
            key was already absent.
        """

    @abstractmethod
    def sweep_expired(self, now: datetime) -> int:
        """Proactively remove every entry that has expired as of ``now``.

        Args:
            now: The current instant from the node clock.

        Returns:
            int: The number of entries removed.
        """

    @abstractmethod
    def clear(self) -> None:
        """Remove all entries from the backend.

        Returns:
            None.
        """
