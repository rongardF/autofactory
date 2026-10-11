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
"""Stubbed cloud ``CacheBackend`` placeholder for the future REST store.

``CloudCacheBackend`` is the selected backend when ``local_cache = false``. It
is intentionally **not implemented** in this iteration: every method raises
``NotImplementedError`` so the node's error handling converts cloud calls into
``success=false`` / ``found=false`` results with a clear message. The intended
REST contract will drive the real implementation in a future iteration.
"""

from datetime import datetime

from repositories.interfaces.cache_backend import CacheBackend
from repositories.models.cache_entry_dto import CacheEntryDTO

# Shared message surfaced by every stubbed method so the node reports a
# consistent reason to callers.
_NOT_IMPLEMENTED_MESSAGE = 'cloud mode not implemented'


class CloudCacheBackend(CacheBackend):
    """Placeholder REST-backed cache backend (all methods unimplemented)."""

    def __init__(
        self,
        station_id: str,
        cache_service_url: str,
        request_timeout_sec: float,
    ) -> None:
        """Record the cloud configuration for the future REST implementation.

        Args:
            station_id: Scope for all keys; part of every REST path.
            cache_service_url: Base URL of the remote cache service.
            request_timeout_sec: HTTP timeout for remote calls.
        """
        self._station_id = station_id
        self._cache_service_url = cache_service_url
        self._request_timeout_sec = request_timeout_sec

    def store(self, entry: CacheEntryDTO) -> None:
        """Not implemented in this iteration.

        Raises:
            NotImplementedError: Always, until cloud mode is built.
        """
        raise NotImplementedError(_NOT_IMPLEMENTED_MESSAGE)

    def fetch(self, key: str, now: datetime) -> tuple[CacheEntryDTO | None, bool]:
        """Not implemented in this iteration.

        Raises:
            NotImplementedError: Always, until cloud mode is built.
        """
        raise NotImplementedError(_NOT_IMPLEMENTED_MESSAGE)

    def delete(self, key: str) -> bool:
        """Not implemented in this iteration.

        Raises:
            NotImplementedError: Always, until cloud mode is built.
        """
        raise NotImplementedError(_NOT_IMPLEMENTED_MESSAGE)

    def sweep_expired(self, now: datetime) -> int:
        """Not implemented; the remote service owns expiry in cloud mode.

        Raises:
            NotImplementedError: Always, until cloud mode is built.
        """
        raise NotImplementedError(_NOT_IMPLEMENTED_MESSAGE)

    def clear(self) -> None:
        """Not implemented in this iteration.

        Raises:
            NotImplementedError: Always, until cloud mode is built.
        """
        raise NotImplementedError(_NOT_IMPLEMENTED_MESSAGE)
