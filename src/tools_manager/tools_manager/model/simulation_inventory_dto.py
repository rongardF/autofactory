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
"""Initial simulated tool inventory, built from the ``simulation.*`` parameters."""

from __future__ import annotations

from collections.abc import Sequence

from pydantic import BaseModel, ConfigDict, Field

from tools_manager.model.tool_info_dto import ToolInfoDto


class SimulationInventoryDTO(BaseModel):
    """Tools that are placed on the rack and on the tool mount when the simulation boots."""

    model_config = ConfigDict(frozen=True)

    slot_indices: tuple[int, ...] = Field(
        description='All rack slot indices listed for the simulation, occupied or empty.',
        default=(),
    )
    rack: dict[int, ToolInfoDto] = Field(
        description='Simulated tools by rack slot index; empty slots are omitted.',
        default_factory=dict,
    )
    mount: ToolInfoDto | None = Field(
        description='Simulated tool on the tool mount, None when the mount is empty.',
        default=None,
    )

    @classmethod
    def from_parameters(
        cls,
        rack_index: Sequence[int],
        rack_tag_data: Sequence[str],
        mount_tag_data: str,
    ) -> 'SimulationInventoryDTO':
        """Decode the hex-encoded RFID payloads of the ``simulation.*`` parameters.

        :param rack_index: Simulated rack slot indices.
        :param rack_tag_data: Hex-encoded RFID payload per slot; empty for an empty slot.
        :param mount_tag_data: Hex-encoded RFID payload of the mounted tool; empty for no tool.
        :raises ValueError: If the arrays differ in length, an index is repeated or a payload is invalid.
        """
        if len(rack_index) != len(rack_tag_data):
            raise ValueError(
                'simulation.tool_rack.index and simulation.tool_rack.tag_data must have the same length, '
                f'got {len(rack_index)} and {len(rack_tag_data)}'
            )
        if len(set(rack_index)) != len(rack_index):
            duplicates = sorted({index for index in rack_index if list(rack_index).count(index) > 1})
            raise ValueError(f'duplicate slot index in simulation.tool_rack.index: {duplicates}')

        rack: dict[int, ToolInfoDto] = {}
        for index, tag_data in zip(rack_index, rack_tag_data, strict=True):
            if not tag_data:
                continue
            try:
                rack[index] = ToolInfoDto.from_hex_tag_data(index, tag_data)
            except ValueError as error:
                raise ValueError(f'invalid simulation.tool_rack.tag_data for slot {index}: {error}') from error

        mount: ToolInfoDto | None = None
        if mount_tag_data:
            try:
                # index -1 means that the tool is not in any rack slot
                mount = ToolInfoDto.from_hex_tag_data(-1, mount_tag_data)
            except ValueError as error:
                raise ValueError(f'invalid simulation.tool_mount: {error}') from error

        return cls(slot_indices=tuple(rack_index), rack=rack, mount=mount)
