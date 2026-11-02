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
"""Local-disk model storage backend.

``LocalModelStorageBackend`` is the selected backend when
``local_storage = true``. Metadata is read from ``<download_path>/local_storage.json``
(a JSON array of ``ObjectModelInfoDTO``-shaped objects) on every call, and every
referenced model is expected to already exist in ``download_path``. No network
access is ever performed.
"""

from pathlib import Path

from pydantic import TypeAdapter

from repositories.interfaces.object_model_storage.model_storage_backend import (
    ModelStorageBackend,
)
from repositories.models.object_model_storage.object_model_info_dto import (
    ObjectModelInfoDTO,
)

LOCAL_STORAGE_FILE_NAME = 'local_storage.json'

_MODELS_ADAPTER = TypeAdapter(list[ObjectModelInfoDTO])


class LocalModelStorageBackend(ModelStorageBackend):
    """Model source backed by ``local_storage.json`` in the download directory."""

    def __init__(self, download_path: Path) -> None:
        """Record the directory that holds ``local_storage.json``.

        Args:
            download_path: The node's absolute download directory.
        """
        self._local_storage_file = download_path / LOCAL_STORAGE_FILE_NAME

    @property
    def local_storage_file(self) -> Path:
        """Path: Absolute path of the ``local_storage.json`` metadata file."""
        return self._local_storage_file

    def fetch_models_info(self) -> list[ObjectModelInfoDTO]:
        """Read and validate every model listed in ``local_storage.json``.

        Returns:
            list[ObjectModelInfoDTO]: All models listed in the file.

        Raises:
            FileNotFoundError: If ``local_storage.json`` does not exist.
            pydantic.ValidationError: If the file is not a valid JSON array of
                model info objects.
        """
        if not self._local_storage_file.is_file():
            raise FileNotFoundError(
                f"local metadata file '{self._local_storage_file}' not found"
            )
        return _MODELS_ADAPTER.validate_json(self._local_storage_file.read_bytes())

    def download_model(self, model_id: str, target_dir: Path) -> None:
        """Fail unconditionally, since local mode has no source to download from.

        Args:
            model_id: Canonical UUID4 of the requested model.
            target_dir: Expected location of the model in ``download_path``.

        Raises:
            FileNotFoundError: Always, since the model is listed in
                ``local_storage.json`` but missing from ``download_path``.
        """
        raise FileNotFoundError(
            f"model '{model_id}' is listed in {LOCAL_STORAGE_FILE_NAME} but not "
            f"present at '{target_dir}' (local mode cannot download)"
        )
