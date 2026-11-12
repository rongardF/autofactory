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
"""Stubbed remote ``ImageBackend`` placeholder for the future REST service.

``RemoteImageBackend`` is the selected backend when ``local_storage = false``.
It is intentionally **not implemented** in this iteration: upload, fetch and
delete raise ``NotImplementedError`` so the node converts remote calls into
``success=false`` results with a clear message.
"""

from repositories.interfaces.image_repository.image_backend import ImageBackend
from repositories.models.image_repository.image_payload_dto import ImagePayloadDTO

_NOT_IMPLEMENTED_MESSAGE = 'remote mode not implemented'


class RemoteImageBackend(ImageBackend):
    """Placeholder REST-backed image backend (operations unimplemented)."""

    def __init__(
        self,
        station_id: str,
        image_service_url: str,
        request_timeout_sec: float,
    ) -> None:
        """Record the remote configuration for the future REST implementation.

        Args:
            station_id: Namespace for all remote calls.
            image_service_url: Base URL of the remote image service.
            request_timeout_sec: HTTP timeout for remote calls.
        """
        self._station_id = station_id
        self._image_service_url = image_service_url
        self._request_timeout_sec = request_timeout_sec

    def initialize(self) -> None:
        """Nothing to acquire while the backend is a stub."""

    def shutdown(self) -> None:
        """Nothing to release while the backend is a stub."""

    def upload(self, payload: ImagePayloadDTO) -> str:
        """Not implemented in this iteration.

        Raises:
            NotImplementedError: Always, until remote mode is built.
        """
        raise NotImplementedError(_NOT_IMPLEMENTED_MESSAGE)

    def fetch(self, image_id: str) -> ImagePayloadDTO:
        """Not implemented in this iteration.

        Raises:
            NotImplementedError: Always, until remote mode is built.
        """
        raise NotImplementedError(_NOT_IMPLEMENTED_MESSAGE)

    def delete(self, image_id: str) -> None:
        """Not implemented in this iteration.

        Raises:
            NotImplementedError: Always, until remote mode is built.
        """
        raise NotImplementedError(_NOT_IMPLEMENTED_MESSAGE)
