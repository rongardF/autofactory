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
"""Cloud ``BlobBackend`` speaking the station-scoped blob REST contract.

REST contract (``{base}`` = ``{blob_service_url}/{station_id}/blob_repository/blobs``):

    POST   {base}                  multipart upload, server allocates the id
    PUT    {base}/{blob_id}        multipart upsert at the supplied id
    GET    {base}/{blob_id}        stream content
    GET    {base}/{blob_id}/info   descriptor JSON
    DELETE {base}/{blob_id}        delete (404 counts as deleted)

Multipart bodies carry a ``descriptor`` part (JSON: ``blob_name``,
``media_type``, ``size_bytes``, ``sha256``, ``metadata``) followed by a ``file``
part with the content. POST/PUT and ``/info`` respond with the stored descriptor
JSON (the same fields plus ``blob_id``, ``created_at`` and ``updated_at``).

Content is streamed in bounded chunks in both directions. Connection and
metadata requests use ``request_timeout_sec``; each content transfer must also
finish within ``transfer_timeout_sec``. The bearer token is read from the
``BLOB_REPOSITORY_AUTH_TOKEN`` environment variable and never logged.
"""

from collections.abc import Callable
import os
from pathlib import Path
from typing import BinaryIO
from urllib.parse import quote
import uuid

from pydantic import ValidationError

from repositories.exceptions.blob_repository import BlobNotFoundError, BlobRepositoryError
from repositories.interfaces.blob_repository.blob_backend import BlobBackend
from repositories.models.blob_repository.blob_descriptor_dto import (
    BlobDescriptorDTO,
    BlobUploadDescriptorDTO,
)
from repositories.services.blob_repository.streaming import CHUNK_SIZE, TransferDeadline
import requests

AUTH_TOKEN_ENV = 'BLOB_REPOSITORY_AUTH_TOKEN'

_OK_STATUSES = (200, 201)
_DELETE_OK_STATUSES = (200, 202, 204, 404)


class MultipartFileStream:
    """File-like ``multipart/form-data`` body that streams the content file.

    Exposes ``__len__`` so the HTTP client sends a ``Content-Length`` header
    instead of buffering, and checks the transfer deadline on every read.
    """

    def __init__(
        self,
        descriptor_json: bytes,
        content_path: Path,
        media_type: str,
        deadline: TransferDeadline,
    ) -> None:
        """Prepare the multipart framing around ``content_path``."""
        self.boundary = f'blob-{uuid.uuid4().hex}'
        self._prefix = (
            f'--{self.boundary}\r\n'
            'Content-Disposition: form-data; name="descriptor"\r\n'
            'Content-Type: application/json\r\n\r\n'
        ).encode() + descriptor_json + (
            f'\r\n--{self.boundary}\r\n'
            'Content-Disposition: form-data; name="file"; filename="blob"\r\n'
            f'Content-Type: {media_type}\r\n\r\n'
        ).encode()
        self._suffix = f'\r\n--{self.boundary}--\r\n'.encode()
        self._content_size = content_path.stat().st_size
        self._content_path = content_path
        self._deadline = deadline
        self._file: BinaryIO | None = None
        self._stage = 0
        self._offset = 0

    @property
    def content_type(self) -> str:
        """``Content-Type`` header value of the body."""
        return f'multipart/form-data; boundary={self.boundary}'

    def __len__(self) -> int:
        """Total body length in bytes."""
        return len(self._prefix) + self._content_size + len(self._suffix)

    def read(self, size: int = -1) -> bytes:
        """Return up to ``size`` bytes of the body (bounded by :data:`CHUNK_SIZE`)."""
        self._deadline.check('upload')
        limit = CHUNK_SIZE if size is None or size < 0 else min(size, CHUNK_SIZE)
        while self._stage < 3:
            if self._stage == 0:
                chunk = self._prefix[self._offset:self._offset + limit]
                if chunk:
                    self._offset += len(chunk)
                    return chunk
            elif self._stage == 1:
                if self._file is None:
                    self._file = open(self._content_path, 'rb')
                chunk = self._file.read(limit)
                if chunk:
                    return chunk
                self.close()
            else:
                chunk = self._suffix[self._offset:self._offset + limit]
                if chunk:
                    self._offset += len(chunk)
                    return chunk
            self._stage += 1
            self._offset = 0
        return b''

    def close(self) -> None:
        """Close the content file if open."""
        if self._file is not None:
            self._file.close()
            self._file = None


class RemoteBlobBackend(BlobBackend):
    """Blob backend backed by the cloud blob REST service."""

    def __init__(
        self,
        station_id: str,
        blob_service_url: str,
        request_timeout_sec: float,
        transfer_timeout_sec: float,
        session_factory: Callable[[], requests.Session] = requests.Session,
    ) -> None:
        """Record the cloud configuration; the HTTP session is opened by :meth:`initialize`.

        Args:
            station_id: Canonical UUID4 station scope; part of every REST path.
            blob_service_url: HTTPS base URL of the blob service.
            request_timeout_sec: Connect and metadata request timeout.
            transfer_timeout_sec: Total timeout for streaming one file.
            session_factory: Creates the HTTP session (injectable for tests).
        """
        self._base_url = (
            f"{blob_service_url.rstrip('/')}/{quote(station_id, safe='')}"
            '/blob_repository/blobs'
        )
        self._request_timeout_sec = request_timeout_sec
        self._transfer_timeout_sec = transfer_timeout_sec
        self._session_factory = session_factory
        self._session: requests.Session | None = None

    @property
    def is_local(self) -> bool:
        """Content lives in the cloud service."""
        return False

    @property
    def has_auth_token(self) -> bool:
        """Whether the bearer token environment variable is set."""
        return bool(os.environ.get(AUTH_TOKEN_ENV))

    def initialize(self) -> None:
        """Open the HTTP session and attach the bearer token, if configured."""
        session = self._session_factory()
        token = os.environ.get(AUTH_TOKEN_ENV)
        if token:
            session.headers['Authorization'] = f'Bearer {token}'
        self._session = session

    def shutdown(self) -> None:
        """Close the HTTP session; safe to call repeatedly."""
        session, self._session = self._session, None
        if session is not None:
            session.close()

    def upload(
        self,
        content_path: Path,
        descriptor: BlobUploadDescriptorDTO,
        blob_id: str | None = None,
        replaced: BlobDescriptorDTO | None = None,
    ) -> BlobDescriptorDTO:
        """POST (new id) or PUT (supplied id) the content with its descriptor."""
        body = MultipartFileStream(
            descriptor.model_dump_json().encode('utf-8'),
            content_path,
            descriptor.media_type,
            TransferDeadline(self._transfer_timeout_sec),
        )
        method, url = ('POST', self._base_url) if blob_id is None else ('PUT', self._url(blob_id))
        try:
            response = self._request(
                method,
                url,
                'upload',
                data=body,
                headers={'Content-Type': body.content_type, 'Content-Length': str(len(body))},
                timeout=self._timeouts(),
            )
        finally:
            body.close()
        with response:
            self._check_status(response, 'upload', _OK_STATUSES)
            stored = self._parse_descriptor(response, 'upload')
        if blob_id is not None and stored.blob_id != blob_id:
            raise BlobRepositoryError('cloud upload returned a different blob_id')
        if stored.size_bytes != descriptor.size_bytes or stored.sha256 != descriptor.sha256:
            raise BlobRepositoryError('cloud upload stored a different size or checksum')
        return stored

    def download(self, blob_id: str, sink: BinaryIO, max_bytes: int) -> None:
        """Stream the content of ``blob_id`` into ``sink``."""
        deadline = TransferDeadline(self._transfer_timeout_sec)
        response = self._request(
            'GET', self._url(blob_id), 'download', stream=True, timeout=self._timeouts()
        )
        with response:
            self._check_status(response, 'download', _OK_STATUSES, blob_id)
            received = 0
            try:
                for chunk in response.iter_content(chunk_size=CHUNK_SIZE):
                    deadline.check('download')
                    received += len(chunk)
                    if received > max_bytes:
                        raise BlobRepositoryError(
                            f'cloud download is larger than the expected {max_bytes} bytes'
                        )
                    sink.write(chunk)
            except requests.RequestException as exc:
                raise BlobRepositoryError(
                    f'cloud download failed: {type(exc).__name__}'
                ) from exc

    def info(self, blob_id: str) -> BlobDescriptorDTO:
        """Return the descriptor of ``blob_id``."""
        response = self._request(
            'GET', f'{self._url(blob_id)}/info', 'info', timeout=self._timeouts()
        )
        with response:
            self._check_status(response, 'info', _OK_STATUSES, blob_id)
            descriptor = self._parse_descriptor(response, 'info')
        if descriptor.blob_id != blob_id:
            raise BlobRepositoryError('cloud info returned a different blob_id')
        return descriptor

    def delete(self, blob_id: str) -> None:
        """Delete ``blob_id``; a missing blob counts as deleted."""
        response = self._request('DELETE', self._url(blob_id), 'delete', timeout=self._timeouts())
        with response:
            self._check_status(response, 'delete', _DELETE_OK_STATUSES, blob_id)

    # region: helpers

    def _url(self, blob_id: str) -> str:
        return f"{self._base_url}/{quote(blob_id, safe='')}"

    def _timeouts(self) -> tuple[float, float]:
        return (self._request_timeout_sec, self._request_timeout_sec)

    def _request(self, method: str, url: str, what: str, **kwargs) -> requests.Response:
        if self._session is None:
            raise BlobRepositoryError('cloud backend is not initialized')
        try:
            return self._session.request(method, url, **kwargs)
        except BlobRepositoryError:
            raise
        except requests.RequestException as exc:
            raise BlobRepositoryError(
                f'cloud {what} failed: {type(exc).__name__}'
            ) from exc

    @staticmethod
    def _check_status(
        response: requests.Response,
        what: str,
        accepted: tuple[int, ...],
        blob_id: str | None = None,
    ) -> None:
        """Map an HTTP status to success or an error (response bodies are not echoed)."""
        status = response.status_code
        if status in accepted:
            return
        if status == 404:
            raise BlobNotFoundError(f'blob not found: {blob_id}' if blob_id else 'not found')
        reasons = {
            400: 'request rejected as invalid',
            401: 'not authorized',
            403: 'not authorized',
            413: 'blob rejected as too large',
        }
        reason = reasons.get(status, 'service error')
        raise BlobRepositoryError(f'cloud {what} failed: {reason} (HTTP {status})')

    @staticmethod
    def _parse_descriptor(response: requests.Response, what: str) -> BlobDescriptorDTO:
        try:
            return BlobDescriptorDTO.model_validate_json(response.content)
        except ValidationError as exc:
            raise BlobRepositoryError(
                f'cloud {what} returned an invalid descriptor'
            ) from exc

    # endregion: helpers
