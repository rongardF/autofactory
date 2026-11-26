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
"""Validation tests for ``BlobRepositoryConfigDTO``."""

from pydantic import ValidationError
import pytest

from repositories.models.blob_repository.blob_repository_config_dto import (
    BlobRepositoryConfigDTO,
)

STATION_ID = '3f1c2a4e-8b7d-4c6a-9e2f-1a2b3c4d5e6f'


def _config(**overrides) -> BlobRepositoryConfigDTO:
    values = {'station_id': STATION_ID, 'allowed_path_roots': ('/data',)}
    values.update(overrides)
    return BlobRepositoryConfigDTO(**values)


def test_defaults_match_spec() -> None:
    """Test the documented defaults: local mode, 3 GiB cache, 1 GiB blobs."""
    config = _config()

    assert config.local_storage is True
    assert config.local_storage_path == '/tmp/station/blobs'
    assert config.cache_size_bytes == 3 * 1024**3
    assert config.max_blob_size_bytes == 1024**3


@pytest.mark.parametrize(
    'station_id', ['', 'not-a-uuid', '3f1c2a4e-8b7d-1c6a-9e2f-1a2b3c4d5e6f']
)
def test_station_id_must_be_uuid4(station_id: str) -> None:
    """Test that a missing, malformed or non-v4 station_id is rejected."""
    with pytest.raises(ValidationError):
        _config(station_id=station_id)


def test_local_storage_path_must_be_absolute() -> None:
    """Test that a relative storage path is rejected."""
    with pytest.raises(ValidationError):
        _config(local_storage_path='blobs')


@pytest.mark.parametrize('roots', [(), ('relative',), ('',)])
def test_allowed_roots_must_be_non_empty_and_absolute(roots: tuple[str, ...]) -> None:
    """Test that the allowlist needs at least one absolute root."""
    with pytest.raises(ValidationError):
        _config(allowed_path_roots=roots)


@pytest.mark.parametrize(
    'field', ['request_timeout_sec', 'transfer_timeout_sec', 'cache_check_interval_sec']
)
def test_timeouts_and_interval_must_be_positive(field: str) -> None:
    """Test that zero timeouts and intervals are rejected."""
    with pytest.raises(ValidationError):
        _config(**{field: 0.0})


def test_max_blob_size_must_not_exceed_cache_size() -> None:
    """Test the max_blob_size_mb <= cache_size_mb relationship."""
    with pytest.raises(ValidationError, match='must not exceed'):
        _config(cache_size_mb=100, max_blob_size_mb=101)


@pytest.mark.parametrize('field', ['cache_size_mb', 'max_blob_size_mb'])
def test_sizes_must_be_positive(field: str) -> None:
    """Test that zero sizes are rejected."""
    with pytest.raises(ValidationError):
        _config(**{field: 0})


def test_cloud_mode_requires_url() -> None:
    """Test that cloud mode without a service URL is rejected."""
    with pytest.raises(ValidationError, match='blob_service_url is required'):
        _config(local_storage=False)


@pytest.mark.parametrize('url', ['http://blobs.example.com', 'ftp://x', 'https://'])
def test_cloud_mode_requires_https_url_with_host(url: str) -> None:
    """Test that cloud mode only accepts HTTPS URLs with a host."""
    with pytest.raises(ValidationError):
        _config(local_storage=False, blob_service_url=url)


def test_cloud_url_with_credentials_is_rejected_without_echoing_them() -> None:
    """Test that embedded credentials are rejected and not repeated in the error."""
    with pytest.raises(ValidationError) as info:
        _config(local_storage=False, blob_service_url='https://user:s3cret@blobs.example.com')

    messages = ' '.join(e['msg'] for e in info.value.errors(include_input=False))
    assert 'credentials' in messages
    assert 's3cret' not in messages


def test_valid_cloud_config_is_accepted() -> None:
    """Test a valid HTTPS cloud configuration."""
    config = _config(local_storage=False, blob_service_url='https://blobs.example.com/api')

    assert config.local_storage is False
