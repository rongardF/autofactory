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
"""Cloud (REST) model storage backend — HTTP transport stubbed.

``CloudModelStorageBackend`` is the selected backend when ``local_storage =
false``. Endpoint construction, response parsing and ZIP extraction are
implemented; only the HTTP transport (:meth:`_http_get`) is a stub that raises
``NotImplementedError`` until the storage service exists. The node converts
that into ``success=false`` results with a clear message.

REST contract:
    GET {storage_service_url}/{station_id}/object_model_storage/get_models_info
        -> JSON array of ObjectModelInfoDTO-shaped objects
    GET {storage_service_url}/{station_id}/object_model_storage/download?model_id=<uuid4>
        -> ZIP archive with the model file(s)
"""

from pathlib import Path
from urllib.parse import quote, urlencode

from pydantic import TypeAdapter

from repositories.interfaces.object_model_storage.model_storage_backend import (
    ModelStorageBackend,
)
from repositories.models.object_model_storage.object_model_info_dto import (
    ObjectModelInfoDTO,
)
from repositories.utils.zip_extractor import extract_zip_bytes

_MODELS_ADAPTER = TypeAdapter(list[ObjectModelInfoDTO])


class CloudModelStorageBackend(ModelStorageBackend):
    """Model source backed by the remote storage REST service."""

    def __init__(
        self,
        station_id: str,
        storage_service_url: str,
        request_timeout_sec: float,
    ) -> None:
        """Record the cloud configuration.

        Args:
            station_id: Canonical UUID4 station scope; part of every REST path.
            storage_service_url: Base URL of the remote storage service.
            request_timeout_sec: HTTP timeout for remote calls.
        """
        self._station_id = station_id
        self._base_url = storage_service_url.rstrip('/')
        self._request_timeout_sec = request_timeout_sec

    def fetch_models_info(self) -> list[ObjectModelInfoDTO]:
        """Fetch every model's metadata from the storage service.

        Returns:
            list[ObjectModelInfoDTO]: All models available to the station.

        Raises:
            NotImplementedError: Until the HTTP transport is implemented.
            pydantic.ValidationError: If the response is not a valid JSON array
                of model info objects.
        """
        payload = self._http_get(self._endpoint_url('get_models_info'))
        return _MODELS_ADAPTER.validate_json(payload)

    def download_model(self, model_id: str, target_dir: Path) -> None:
        """Download the model ZIP archive and extract it into ``target_dir``.

        Args:
            model_id: Canonical UUID4 of the model to download.
            target_dir: Directory to create with the extracted model contents.

        Returns:
            None.

        Raises:
            NotImplementedError: Until the HTTP transport is implemented.
            zipfile.BadZipFile: If the response is not a valid ZIP archive.
            ValueError: If an archive member escapes ``target_dir``.
        """
        url = f"{self._endpoint_url('download')}?{urlencode({'model_id': model_id})}"
        payload = self._http_get(url)
        extract_zip_bytes(payload, target_dir)

    def _endpoint_url(self, endpoint: str) -> str:
        """Build the absolute URL of a station-scoped storage endpoint.

        Args:
            endpoint: Endpoint name below ``object_model_storage/``.

        Returns:
            str: The absolute endpoint URL (without query string).
        """
        return (
            f'{self._base_url}/{quote(self._station_id)}/object_model_storage/'
            f'{endpoint}'
        )

    def _http_get(self, url: str) -> bytes:
        """Perform an HTTP GET and return the response body.

        Stub: the storage service does not exist yet, so no request is sent.

        Args:
            url: Absolute URL to request.

        Returns:
            bytes: The response body.

        Raises:
            NotImplementedError: Always, until cloud mode is implemented.
        """
        raise NotImplementedError(f'cloud mode not implemented (GET {url})')
