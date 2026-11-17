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
"""Tool action identity checks and swap choreography without live ROS nodes."""

from __future__ import annotations

from threading import RLock
from unittest.mock import Mock, call

import pytest
from builtin_interfaces.msg import Time
from std_srvs.srv import SetBool

from tools_manager.action import MountTool, UnmountTool
from tools_manager.model.slots_dto import SlotsDto
from tools_manager.model.tool_info_dto import ToolInfoDto
from tools_manager.model.tools_manager_config import ToolSlotDTO
from tools_manager.tools_manager import ToolsManager


@pytest.fixture
def manager(monkeypatch: pytest.MonkeyPatch) -> ToolsManager:
    """Provide real action methods with mocked motion, world and ROS services."""
    node = object.__new__(ToolsManager)
    node._tool_action_lock = RLock()
    node._tool_mounted_info_lock = RLock()
    node._slots_info_lock = RLock()
    node.tool_mounted = ToolInfoDto(index=-1, tool_sn='mounted')
    node.tool_rack_slots = SlotsDto(
        tools_expected=[
            ToolSlotDTO(index=1, tool_sn='mounted'),
            ToolSlotDTO(index=2, tool_sn='racked'),
        ],
        tools_mounted=[ToolInfoDto(index=2, tool_sn='racked')],
    )
    node._world_client_adapter = Mock()
    node._movement_controller_action_client = Mock()
    node._tool_mount_lock = Mock()
    node._config = Mock()
    monkeypatch.setattr(node, 'get_logger', Mock())
    clock = Mock()
    clock.now.return_value.to_msg.return_value = Time(sec=10)
    monkeypatch.setattr(node, 'get_clock', Mock(return_value=clock))
    monkeypatch.setattr(node, '_call_action', Mock(return_value=True))
    monkeypatch.setattr(node, '_call_service', Mock(return_value=True))
    monkeypatch.setattr(node, '_endtool_mounted', Mock())

    def attach_tool(model_id: str, link_name: str, **kwargs: object) -> None:
        if link_name != 'tool_mount_tcp':
            node.tool_mounted = None
            node.tool_rack_slots.tools_mounted.append(
                ToolInfoDto(index=1, tool_sn=model_id),
            )

    node._world_client_adapter.attach_to_link.side_effect = attach_tool
    return node


@pytest.mark.parametrize('tool_sn', ['racked', 'unknown', '', 'MOUNTED'])
def test_unmount_rejects_wrong_serial(manager: ToolsManager, tool_sn: str) -> None:
    """A wrong serial aborts before motion, world changes or lock operations."""
    goal = Mock(request=UnmountTool.Goal(tool_sn=tool_sn))

    result = manager._unmount_tool_callback(goal)

    assert result.success is False
    assert tool_sn in result.message
    assert 'mounted' in result.message
    manager.get_logger().error.assert_called_once_with(result.message)
    goal.abort.assert_called_once_with()
    goal.succeed.assert_not_called()
    manager._call_action.assert_not_called()
    manager._call_service.assert_not_called()
    assert manager._world_client_adapter.mock_calls == []
    assert manager._tool_mount_lock.mock_calls == []
    manager._endtool_mounted.assert_not_called()
    assert manager.tool_mounted.tool_sn == 'mounted'


def test_unmount_rejects_empty_mount(manager: ToolsManager) -> None:
    """An empty mount cannot be unmounted, even with a known serial."""
    manager.tool_mounted = None
    goal = Mock(request=UnmountTool.Goal(tool_sn='mounted'))

    result = manager._unmount_tool_callback(goal)

    assert result.success is False
    assert result.message == 'No tool is currently mounted.'
    goal.abort.assert_called_once_with()
    manager._call_action.assert_not_called()
    manager._call_service.assert_not_called()


def test_unmount_matching_serial(manager: ToolsManager) -> None:
    """A matching serial completes the existing unmount sequence."""
    goal = Mock(request=UnmountTool.Goal(tool_sn='mounted'))

    result = manager._unmount_tool_callback(goal)

    assert result.success is True
    goal.succeed.assert_called_once_with()
    goal.abort.assert_not_called()
    assert manager._call_action.call_count == 3
    manager._world_client_adapter.attach_to_link.assert_called_once_with(
        model_id='mounted', link_name='slot1_attached_link',
    )
    assert manager._call_service.call_args.args[1].data is False
    manager._endtool_mounted.assert_called_once_with('mounted', False)


def test_mount_swaps_current_tool_not_requested_tool(
    manager: ToolsManager,
) -> None:
    """A swap returns the current tool and only succeeds after mounting its replacement."""
    goal = Mock(request=MountTool.Goal(tool_sn='racked'))

    def operate_lock(client: object, request: SetBool.Request) -> bool:
        if request.data:
            manager.tool_mounted = ToolInfoDto(index=-1, tool_sn='racked')
        return True

    manager._call_service.side_effect = operate_lock

    result = manager._mount_tool_callback(goal)

    assert result.success is True
    assert goal.request.tool_sn == 'racked'
    goal.succeed.assert_called_once_with()
    goal.abort.assert_not_called()
    assert manager._call_action.call_count == 6
    assert manager._world_client_adapter.attach_to_link.call_args_list == [
        call(model_id='mounted', link_name='slot1_attached_link'),
        call(
            model_id='racked',
            link_name='tool_mount_tcp',
            rotation=(0.0, 0.0, 1.0, 0.0),
        ),
    ]
    assert manager._endtool_mounted.call_args_list == [
        call('mounted', False), call('racked', True),
    ]
