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
"""Abstract contract for the remote image service used by ``image_repository``."""

from abc import ABC, abstractmethod

from repositories.models.image_repository.image_payload_dto import ImagePayloadDTO


class ImageBackend(ABC):
    """Remote image service operations, scoped by the configured station."""

    @abstractmethod
    def initialize(self) -> None:
        """Acquire any resources (connections, sessions) the backend needs."""

    @abstractmethod
    def shutdown(self) -> None:
        """Release the resources acquired by :meth:`initialize`."""

    @abstractmethod
    def upload(self, payload: ImagePayloadDTO) -> str:
        """Upload an image and its metadata.

        Args:
            payload: The image to upload.

        Returns:
            str: The ``image_id`` assigned by the remote service.
        """

    @abstractmethod
    def fetch(self, image_id: str) -> ImagePayloadDTO:
        """Download an image and its metadata.

        Args:
            image_id: UUID4 identifier of the image.

        Returns:
            ImagePayloadDTO: The remote image.

        Raises:
            ImageNotFoundError: If the remote service does not know ``image_id``.
        """

    @abstractmethod
    def delete(self, image_id: str) -> None:
        """Delete an image; an image that is already absent counts as deleted.

        Args:
            image_id: UUID4 identifier of the image.
        """
