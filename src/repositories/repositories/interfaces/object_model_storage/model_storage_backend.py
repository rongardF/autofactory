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
"""Abstract source of object model metadata and model files.

A backend owns *where* model metadata and model files come from (a local
``local_storage.json`` file or the cloud storage service). The node handles the
ROS surface, filtering and the local cache index, and delegates every
source-specific concern to a backend instance.
"""

from abc import ABC, abstractmethod
from pathlib import Path

from repositories.models.object_model_storage.object_model_info_dto import (
    ObjectModelInfoDTO,
)


class ModelStorageBackend(ABC):
    """Contract between the ``object_model_storage`` node and a model source."""

    @abstractmethod
    def fetch_models_info(self) -> list[ObjectModelInfoDTO]:
        """Fetch the metadata of every model available to this station.

        Metadata is never cached: every call reads the source afresh.

        Returns:
            list[ObjectModelInfoDTO]: All models known to the source.

        Raises:
            Exception: Any failure reading or parsing the source; the node
                converts it into a failure result.
        """

    @abstractmethod
    def download_model(self, model_id: str, target_dir: Path) -> None:
        """Make the files of ``model_id`` available at ``target_dir``.

        Called only when the model is not already cached locally.

        Args:
            model_id: Canonical UUID4 of the model to download.
            target_dir: Directory to create with the model contents; it does
                not exist when this method is called.

        Returns:
            None.

        Raises:
            Exception: Any failure obtaining the model; the node converts it
                into a failure result.
        """
