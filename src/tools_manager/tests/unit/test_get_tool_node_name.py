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
"""``~/get_tool_node_name`` callback: the tool must be present and have a configured node."""

from __future__ import annotations

from threading import RLock
from unittest.mock import Mock

import pytest

from tools_manager.model.slots_dto import SlotsDto
from tools_manager.model.tool_info_dto import ToolInfoDto
from tools_manager.model.tools_manager_config import ToolSlotDTO, ToolsManagerConfigDTO
from tools_manager.srv import GetToolNodeName
from tools_manager.tools_manager import ToolsManager


@pytest.fixture
def manager() -> ToolsManager:
    node = object.__new__(ToolsManager)
    node._tool_mounted_info_lock = RLock()
    node._slots_info_lock = RLock()
    node.get_logger = Mock()
    node._config = Mock()
    node._config.tools_manager_config = ToolsManagerConfigDTO.from_parameters(
        index=[1, 2, 3],
        tool_sn=['RACKED', 'MOUNTED', ''],
        node_name=['endtool_racked', 'endtool_mounted', ''],
    )
    node.tool_rack_slots = SlotsDto(
        tools_expected=[ToolSlotDTO(index=1, tool_sn='RACKED')],
        tools_mounted=[ToolInfoDto(index=1, tool_sn='RACKED')],
    )
    node.tool_mounted = ToolInfoDto(index=-1, tool_sn='MOUNTED')
    return node


def _call(node: ToolsManager, tool_sn: str) -> GetToolNodeName.Response:
    return node._get_tool_node_name_callback(
        GetToolNodeName.Request(tool_sn=tool_sn), GetToolNodeName.Response()
    )


@pytest.mark.parametrize(
    ('tool_sn', 'node_name'),
    [('RACKED', 'endtool_racked'), ('MOUNTED', 'endtool_mounted')],
)
def test_tool_on_rack_or_mounted_returns_node_name(
    manager: ToolsManager, tool_sn: str, node_name: str
) -> None:
    response = _call(manager, tool_sn)
    assert response.success is True
    assert response.node_name == node_name
    assert response.error_message == ''


def test_configured_tool_that_is_not_present_fails(manager: ToolsManager) -> None:
    manager.tool_mounted = None
    response = _call(manager, 'MOUNTED')
    assert response.success is False
    assert response.node_name == ''
    assert response.error_message != ''


@pytest.mark.parametrize('tool_sn', ['unknown', ''])
def test_unknown_serial_fails(manager: ToolsManager, tool_sn: str) -> None:
    response = _call(manager, tool_sn)
    assert response.success is False
    assert response.node_name == ''
    assert response.error_message != ''
