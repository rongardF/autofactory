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
"""Tests for ``RemoteBlobBackend`` with a mocked HTTP session (no network access)."""

from datetime import datetime, timezone
import hashlib
import io
import json
from pathlib import Path
from unittest.mock import MagicMock
import uuid

import pytest

from repositories.exceptions.blob_repository import BlobNotFoundError, BlobRepositoryError
from repositories.models.blob_repository.blob_descriptor_dto import (
    BlobDescriptorDTO,
    BlobUploadDescriptorDTO,
)
from repositories.services.blob_repository import remote_blob_backend, streaming
from repositories.services.blob_repository.remote_blob_backend import (
    AUTH_TOKEN_ENV,
    RemoteBlobBackend,
)
import requests

STATION_ID = '3f1c2a4e-8b7d-4c6a-9e2f-1a2b3c4d5e6f'
BASE = f'https://blobs.example.com/api/{STATION_ID}/blob_repository/blobs'


class FakeResponse:
    """Minimal ``requests.Response`` stand-in usable as a context manager."""

    def __init__(self, status: int, content: bytes = b'', chunks: list[bytes] | None = None):
        self.status_code = status
        self.content = content
        self._chunks = chunks if chunks is not None else [content]

    def iter_content(self, chunk_size: int):
        yield from self._chunks

    def __enter__(self) -> 'FakeResponse':
        return self

    def __exit__(self, *exc) -> None:
        return None


def _descriptor_json(blob_id: str, content: bytes, **overrides) -> bytes:
    now = datetime.now(timezone.utc)
    descriptor = BlobDescriptorDTO(
        blob_id=blob_id,
        blob_name='a.bin',
        size_bytes=len(content),
        sha256=hashlib.sha256(content).hexdigest(),
        created_at=now,
        updated_at=now,
    )
    return descriptor.model_copy(update=overrides).model_dump_json().encode()


def _upload_descriptor(content: bytes) -> BlobUploadDescriptorDTO:
    return BlobUploadDescriptorDTO(
        blob_name='a.bin',
        media_type='application/octet-stream',
        size_bytes=len(content),
        sha256=hashlib.sha256(content).hexdigest(),
        metadata={'k': 1},
    )


@pytest.fixture
def session() -> MagicMock:
    fake = MagicMock(spec=requests.Session)
    fake.headers = {}
    return fake


@pytest.fixture
def backend(session: MagicMock, monkeypatch) -> RemoteBlobBackend:
    monkeypatch.delenv(AUTH_TOKEN_ENV, raising=False)
    remote = RemoteBlobBackend(
        STATION_ID, 'https://blobs.example.com/api/', 2.5, 30.0, session_factory=lambda: session
    )
    remote.initialize()
    return remote


def test_initialize_sets_bearer_token_from_environment(session: MagicMock, monkeypatch) -> None:
    """Test that the token comes from BLOB_REPOSITORY_AUTH_TOKEN."""
    monkeypatch.setenv(AUTH_TOKEN_ENV, 'tok')
    remote = RemoteBlobBackend(STATION_ID, 'https://x', 1.0, 1.0, session_factory=lambda: session)

    remote.initialize()

    assert session.headers['Authorization'] == 'Bearer tok'


def test_no_authorization_header_without_token(backend: RemoteBlobBackend, session) -> None:
    """Test that no Authorization header is set when the variable is absent."""
    assert 'Authorization' not in session.headers


def test_upload_without_id_posts_streamed_multipart(
    backend: RemoteBlobBackend, session: MagicMock, tmp_path: Path, mocker
) -> None:
    """Test POST to the collection with a bounded-chunk multipart body."""
    mocker.patch.object(streaming, 'CHUNK_SIZE', 1000)
    mocker.patch.object(remote_blob_backend, 'CHUNK_SIZE', 1000)
    content = bytes(range(256)) * 20
    source = tmp_path / 'blob'
    source.write_bytes(content)
    blob_id = str(uuid.uuid4())
    captured: dict = {}

    def respond(method, url, **kwargs):
        body = kwargs['data']
        chunks = []
        while chunk := body.read(8192):
            chunks.append(chunk)
        captured.update(method=method, url=url, kwargs=kwargs, chunks=chunks)
        return FakeResponse(201, _descriptor_json(blob_id, content))

    session.request.side_effect = respond

    stored = backend.upload(source, _upload_descriptor(content))

    assert (captured['method'], captured['url']) == ('POST', BASE)
    assert max(len(c) for c in captured['chunks']) <= 1000
    body = b''.join(captured['chunks'])
    headers = captured['kwargs']['headers']
    assert int(headers['Content-Length']) == len(body)
    assert headers['Content-Type'].startswith('multipart/form-data; boundary=')
    assert content in body and b'name="descriptor"' in body and b'name="file"' in body
    assert captured['kwargs']['timeout'] == (2.5, 2.5)
    assert stored.blob_id == blob_id


def test_upload_with_id_puts_to_blob_url(
    backend: RemoteBlobBackend, session: MagicMock, tmp_path: Path
) -> None:
    """Test that a supplied id is upserted with PUT."""
    content = b'abc'
    source = tmp_path / 'blob'
    source.write_bytes(content)
    blob_id = str(uuid.uuid4())
    session.request.return_value = FakeResponse(200, _descriptor_json(blob_id, content))

    backend.upload(source, _upload_descriptor(content), blob_id)

    assert session.request.call_args.args[:2] == ('PUT', f'{BASE}/{blob_id}')


def test_upload_rejects_checksum_disagreement(
    backend: RemoteBlobBackend, session: MagicMock, tmp_path: Path
) -> None:
    """Test that a server descriptor with a different checksum fails the upload."""
    source = tmp_path / 'blob'
    source.write_bytes(b'abc')
    session.request.return_value = FakeResponse(
        201, _descriptor_json(str(uuid.uuid4()), b'abc', sha256='0' * 64)
    )

    with pytest.raises(BlobRepositoryError, match='different size or checksum'):
        backend.upload(source, _upload_descriptor(b'abc'))


def test_download_streams_chunks_into_sink(backend: RemoteBlobBackend, session) -> None:
    """Test that GET content is written chunk by chunk."""
    blob_id = str(uuid.uuid4())
    session.request.return_value = FakeResponse(200, chunks=[b'ab', b'cd'])
    sink = io.BytesIO()

    backend.download(blob_id, sink, 4)

    assert sink.getvalue() == b'abcd'
    assert session.request.call_args.args == ('GET', f'{BASE}/{blob_id}')
    assert session.request.call_args.kwargs['stream'] is True


def test_download_larger_than_expected_fails(backend: RemoteBlobBackend, session) -> None:
    """Test that more bytes than expected abort the download."""
    session.request.return_value = FakeResponse(200, chunks=[b'abc', b'def'])

    with pytest.raises(BlobRepositoryError, match='larger than the expected'):
        backend.download(str(uuid.uuid4()), io.BytesIO(), 4)


def test_download_exceeding_transfer_timeout_fails(
    backend: RemoteBlobBackend, session, mocker
) -> None:
    """Test that the total transfer deadline is enforced while streaming."""
    mocker.patch.object(streaming.time, 'monotonic', side_effect=[0.0, 1.0, 100.0])
    session.request.return_value = FakeResponse(200, chunks=[b'a', b'b'])
    remote = RemoteBlobBackend(STATION_ID, 'https://x', 1.0, 30.0, session_factory=lambda: session)
    remote.initialize()

    with pytest.raises(BlobRepositoryError, match='transfer_timeout_sec'):
        remote.download(str(uuid.uuid4()), io.BytesIO(), 10)


def test_info_parses_descriptor(backend: RemoteBlobBackend, session) -> None:
    """Test that /info returns a validated descriptor."""
    blob_id = str(uuid.uuid4())
    session.request.return_value = FakeResponse(200, _descriptor_json(blob_id, b'abc'))

    assert backend.info(blob_id).size_bytes == 3
    assert session.request.call_args.args == ('GET', f'{BASE}/{blob_id}/info')


def test_info_404_raises_not_found(backend: RemoteBlobBackend, session) -> None:
    """Test that 404 maps to BlobNotFoundError."""
    session.request.return_value = FakeResponse(404)

    with pytest.raises(BlobNotFoundError):
        backend.info(str(uuid.uuid4()))


def test_info_invalid_json_is_an_error(backend: RemoteBlobBackend, session) -> None:
    """Test that a malformed descriptor response is rejected."""
    session.request.return_value = FakeResponse(200, json.dumps({'x': 1}).encode())

    with pytest.raises(BlobRepositoryError, match='invalid descriptor'):
        backend.info(str(uuid.uuid4()))


@pytest.mark.parametrize('status', [200, 204, 404])
def test_delete_accepts_success_and_absent(backend: RemoteBlobBackend, session, status) -> None:
    """Test that delete succeeds for success statuses and for an absent blob."""
    session.request.return_value = FakeResponse(status)

    backend.delete(str(uuid.uuid4()))

    assert session.request.call_args.args[0] == 'DELETE'


@pytest.mark.parametrize(
    ('status', 'reason'),
    [(400, 'invalid'), (401, 'not authorized'), (413, 'too large'), (500, 'service error')],
)
def test_error_statuses_map_to_messages_without_body(
    backend: RemoteBlobBackend, session, status: int, reason: str
) -> None:
    """Test status mapping and that response bodies are never echoed."""
    session.request.return_value = FakeResponse(status, b'secret-token-in-body')

    with pytest.raises(BlobRepositoryError, match=reason) as info:
        backend.delete(str(uuid.uuid4()))
    assert 'secret' not in str(info.value)


def test_transport_errors_become_repository_errors(backend: RemoteBlobBackend, session) -> None:
    """Test that requests exceptions are converted at the backend boundary."""
    session.request.side_effect = requests.ConnectionError('refused')

    with pytest.raises(BlobRepositoryError, match='cloud info failed: ConnectionError'):
        backend.info(str(uuid.uuid4()))


def test_calls_before_initialize_or_after_shutdown_fail(
    backend: RemoteBlobBackend, session
) -> None:
    """Test that shutdown closes the session once and disables further calls."""
    backend.shutdown()
    backend.shutdown()

    session.close.assert_called_once()
    with pytest.raises(BlobRepositoryError, match='not initialized'):
        backend.info(str(uuid.uuid4()))
