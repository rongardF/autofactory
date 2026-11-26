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
"""Tests for the simulated tool mount controller."""

from types import SimpleNamespace
from unittest.mock import Mock

from endtools.enumerator.tool_type_enum import ToolTypeEnum
from tools_manager.model.tool_info_dto import ToolInfoDto
from tools_manager.services.simulated_tool_mount_controller import SimulatedToolMountController


def _tag_frame(tool: ToolInfoDto) -> SimpleNamespace:
    return SimpleNamespace(data=list(bytes.fromhex(tool.to_tag_data().decode())))


def _tool() -> ToolInfoDto:
    return ToolInfoDto(
        tool_sn='def123',
        tool_type=ToolTypeEnum.VOLUMETRIC_DISPENSER,
        tool_part_number='KRC123',
        tool_part_revision='R1A',
    )


def test_tool_in_range_is_reported_only_while_lock_is_closed() -> None:
    controller = SimulatedToolMountController(Mock())
    controller._update_mounted_info(_tag_frame(_tool()))

    assert controller.get_mounted_tool_info().tool_sn == 'def123'
    assert controller.is_mounted()

    assert controller.lock_closed(False)
    assert controller.get_mounted_tool_info() is None
    assert not controller.is_mounted()

    assert controller.lock_closed(True)
    assert controller.get_mounted_tool_info().tool_sn == 'def123'


def test_tool_detected_while_unlocked_is_reported_once_locked() -> None:
    controller = SimulatedToolMountController(Mock())
    controller.lock_closed(False)
    controller._update_mounted_info(_tag_frame(_tool()))

    assert controller.get_mounted_tool_info() is None

    controller.lock_closed(True)
    assert controller.get_mounted_tool_info().tool_sn == 'def123'
