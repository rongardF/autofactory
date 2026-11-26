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
"""Abstract source-of-truth contract used by ``blob_repository``."""

from abc import ABC, abstractmethod
from pathlib import Path
from typing import BinaryIO

from repositories.models.blob_repository.blob_descriptor_dto import (
    BlobDescriptorDTO,
    BlobUploadDescriptorDTO,
)


class BlobBackend(ABC):
    """Blob storage operations of the source of truth (local store or cloud)."""

    @property
    @abstractmethod
    def is_local(self) -> bool:
        """Return True if the backend keeps content in the node's local store itself."""

    @abstractmethod
    def initialize(self) -> None:
        """Acquire any resources (connections, sessions) the backend needs."""

    @abstractmethod
    def shutdown(self) -> None:
        """Release the resources acquired by :meth:`initialize`; safe to repeat."""

    @abstractmethod
    def upload(
        self,
        content_path: Path,
        descriptor: BlobUploadDescriptorDTO,
        blob_id: str | None = None,
        replaced: BlobDescriptorDTO | None = None,
    ) -> BlobDescriptorDTO:
        """Store content streamed from ``content_path``.

        Args:
            content_path: Fully written content file matching ``descriptor``.
                A local backend may move it into its store.
            descriptor: Name, media type, size, checksum and user metadata.
            blob_id: UUID4 to create or replace; None allocates a new id.
            replaced: Descriptor of the blob being replaced, if known.

        Returns:
            BlobDescriptorDTO: The stored descriptor, including the id.
        """

    @abstractmethod
    def download(self, blob_id: str, sink: BinaryIO, max_bytes: int) -> None:
        """Stream the content of ``blob_id`` into ``sink`` in bounded chunks.

        Raises:
            BlobNotFoundError: If the blob does not exist.
            BlobRepositoryError: If more than ``max_bytes`` arrive or the transfer fails.
        """

    @abstractmethod
    def info(self, blob_id: str) -> BlobDescriptorDTO:
        """Return the descriptor of ``blob_id`` without transferring content.

        Raises:
            BlobNotFoundError: If the blob does not exist.
        """

    @abstractmethod
    def delete(self, blob_id: str) -> None:
        """Delete a blob; a blob that is already absent counts as deleted."""
