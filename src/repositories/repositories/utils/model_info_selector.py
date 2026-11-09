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
"""Selection of object model infos for the ``get_models_info`` service."""

from collections.abc import Iterable

from repositories.models.object_model_storage.object_model_info_dto import (
    MappingValueDTO,
    ObjectModelInfoDTO,
)
from repositories.utils.uuid_validator import parse_uuid4


def select_models_info(
    models: Iterable[ObjectModelInfoDTO],
    model_id: str = '',
    filters: Iterable[MappingValueDTO] = (),
) -> list[ObjectModelInfoDTO]:
    """Select the models matching a ``get_models_info`` request.

    A non-empty ``model_id`` takes precedence and ``filters`` are ignored; an
    unknown (but valid) ``model_id`` yields an empty list. Otherwise every
    model carrying all ``filters`` pairs is returned.

    Args:
        models: All known models.
        model_id: Optional UUID4 of the single requested model.
        filters: Required key/value tags; ignored when ``model_id`` is set.

    Returns:
        list[ObjectModelInfoDTO]: The matching models.

    Raises:
        ValueError: If ``model_id`` is non-empty and not a canonical UUID4.
    """
    if model_id:
        wanted = parse_uuid4(model_id, 'model_id')
        return [model for model in models if model.model_id == wanted]
    filters = list(filters)
    return [model for model in models if model.matches_all(filters)]
