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
"""Validation of the observed tool inventory against the expected rack assignments."""

from __future__ import annotations

from collections import Counter
from collections.abc import Iterable

from tools_manager.model.tool_info_dto import ToolInfoDto
from tools_manager.model.tools_manager_config import ToolsManagerConfigDTO


def find_inventory_problems(
    expected: ToolsManagerConfigDTO,
    rack: Iterable[ToolInfoDto],
    mount: ToolInfoDto | None,
) -> list[str]:
    """List every way the observed inventory violates the expected assignments.

    A tool is valid on the mount, or in its home slot on the rack. Every
    configured tool must be present exactly once; nothing else may be present.

    :param expected: Expected tool assignments of all physical slots.
    :param rack: Tools observed on the rack, each carrying the slot index it was read from.
    :param mount: Tool observed on the tool mount, or ``None`` when the mount is empty.
    :returns: Actionable problem descriptions; empty when the inventory is valid.
    """
    problems: list[str] = []
    home_slot_by_sn = {slot.tool_sn: slot.index for slot in expected.assigned_slots}
    observed: list[tuple[str, ToolInfoDto]] = [(f'rack slot {tool.index}', tool) for tool in rack]
    if mount is not None:
        observed.append(('tool mount', mount))

    slot_counts = Counter(tool.index for tool in rack)
    for index, count in sorted(slot_counts.items()):
        if count > 1:
            problems.append(f'rack slot {index} reports {count} tools at the same time')

    for location, tool in observed:
        if tool.tool_sn not in home_slot_by_sn:
            problems.append(f"unknown tool '{tool.tool_sn}' detected in {location}; it is not in tools.tool_sn")
        elif location != 'tool mount' and tool.index != home_slot_by_sn[tool.tool_sn]:
            problems.append(
                f"tool '{tool.tool_sn}' is in {location} but its home slot is {home_slot_by_sn[tool.tool_sn]}"
            )

    for location, tool in observed:
        if location != 'tool mount' and tool.index not in expected.slot_indices:
            problems.append(f"rack slot {tool.index} (tool '{tool.tool_sn}') is not listed in tools.index")

    serial_locations: dict[str, list[str]] = {}
    for location, tool in observed:
        serial_locations.setdefault(tool.tool_sn, []).append(location)
    for tool_sn, locations in sorted(serial_locations.items()):
        if len(locations) > 1:
            problems.append(f"tool '{tool_sn}' is detected more than once: {', '.join(locations)}")

    for tool_sn, home_index in sorted(home_slot_by_sn.items()):
        if tool_sn not in serial_locations:
            problems.append(
                f"configured tool '{tool_sn}' (home slot {home_index}) is neither on the rack nor on the tool mount"
            )

    return problems
