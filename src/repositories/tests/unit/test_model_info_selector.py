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
"""Unit tests for ``get_models_info`` model selection by ``model_id``."""

from unittest.mock import MagicMock

import pytest

from repositories.enums.object_model_storage.model_type_enum import ModelTypeEnum
from repositories.models.object_model_storage.object_model_info_dto import (
    MappingValueDTO,
    ObjectModelInfoDTO,
)
from repositories.msg import ObjectModelStorageMappingValue
from repositories.object_model_storage import ObjectModelStorage
from repositories.srv import ObjectModelStorageGetModelsInfo
from repositories.utils.model_info_selector import select_models_info

ID_A = '0b5c2d4e-1f3a-4b6c-8d7e-9f0a1b2c3d4e'
ID_B = '7e6d5c4b-3a2f-4e1d-9c8b-7a6f5e4d3c2b'
ID_UNKNOWN = '11111111-2222-4333-8444-555555555555'

TAG = MappingValueDTO(key='kind', value='product')


@pytest.fixture
def models() -> list[ObjectModelInfoDTO]:
    return [
        ObjectModelInfoDTO(model_id=ID_A, model_type=ModelTypeEnum.STL, filter_keys=(TAG,)),
        ObjectModelInfoDTO(model_id=ID_B, model_type=ModelTypeEnum.STL),
    ]


def test_model_id_hit(models):
    assert [m.model_id for m in select_models_info(models, ID_B)] == [ID_B]


def test_model_id_is_case_insensitive(models):
    assert [m.model_id for m in select_models_info(models, ID_B.upper())] == [ID_B]


def test_model_id_miss_returns_empty(models):
    assert select_models_info(models, ID_UNKNOWN) == []


def test_invalid_model_id_raises(models):
    with pytest.raises(ValueError):
        select_models_info(models, 'not-a-uuid')


def test_model_id_overrides_filter_keys(models):
    # The filter alone would select only ID_A.
    selected = select_models_info(models, ID_B, [TAG])
    assert [m.model_id for m in selected] == [ID_B]


def test_empty_model_id_uses_filter_keys(models):
    assert [m.model_id for m in select_models_info(models, '', [TAG])] == [ID_A]
    assert len(select_models_info(models)) == 2


def _call_handler(models, model_id='', filter_keys=()):
    node = MagicMock()
    node._fetch_models_info.return_value = models
    request = ObjectModelStorageGetModelsInfo.Request()
    request.model_id = model_id
    request.filter_keys = [
        ObjectModelStorageMappingValue(key=k, value=v) for k, v in filter_keys
    ]
    response = ObjectModelStorageGetModelsInfo.Response()
    return ObjectModelStorage._on_get_models_info(node, request, response)


def test_handler_model_id_hit(models):
    response = _call_handler(models, ID_A)
    assert response.success
    assert [m.model_id for m in response.models] == [ID_A]


def test_handler_model_id_miss_succeeds_with_empty_list(models):
    response = _call_handler(models, ID_UNKNOWN)
    assert response.success
    assert response.error_message == ''
    assert list(response.models) == []


def test_handler_invalid_model_id_fails(models):
    response = _call_handler(models, 'bogus')
    assert not response.success
    assert 'model_id' in response.error_message
    assert list(response.models) == []


def test_handler_model_id_overrides_filter_keys(models):
    response = _call_handler(models, ID_B, [('kind', 'product')])
    assert response.success
    assert [m.model_id for m in response.models] == [ID_B]


def test_handler_filter_keys_unchanged(models):
    response = _call_handler(models, '', [('kind', 'product')])
    assert response.success
    assert [m.model_id for m in response.models] == [ID_A]
