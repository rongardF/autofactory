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
"""Expected tool assignments of the rack, built from the ``tools.*`` parameters."""

from __future__ import annotations

from collections.abc import Sequence

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator


class ToolSlotDTO(BaseModel):
    """A physical rack slot and the tool expected to live in it."""

    model_config = ConfigDict(frozen=True)

    index: int = Field(description='Rack slot index.')
    tool_sn: str | None = Field(
        description='Serial number of the tool whose home is this slot; None when the slot is unassigned.',
        default=None,
    )
    node_name: str | None = Field(
        description='Name of the endtool node that belongs to the expected tool; None when the slot is unassigned.',
        default=None,
    )

    def __eq__(self, value: object) -> bool:
        return (
            isinstance(value, ToolSlotDTO)
            and self.index == value.index
            and self.tool_sn == value.tool_sn
        )

    def __hash__(self) -> int:
        return hash((self.index, self.tool_sn))

    @field_validator('tool_sn', 'node_name', mode='after')
    @classmethod
    def _empty_means_unassigned(cls, value: str | None) -> str | None:
        """Treat an empty string the same as no value."""
        return value or None

    @property
    def tool_lifted_frame(self) -> str:
        """Get the name of the frame for the tool lifted pose."""
        return f'slot{self.index}_lifted_link'

    @property
    def tool_attached_frame(self) -> str:
        """Get the name of the frame for the tool attached pose."""
        return f'slot{self.index}_attached_link'

    @property
    def tool_slide_in_frame(self) -> str:
        """Get the name of the frame for the tool slide in pose."""
        return f'slot{self.index}_slide_in_link'


class ToolsManagerConfigDTO(BaseModel):
    """Every physical rack slot together with its expected (home) tool assignment."""

    model_config = ConfigDict(frozen=True)

    slots: tuple[ToolSlotDTO, ...] = Field(description='All physical rack slots, assigned or not.')

    @classmethod
    def from_parameters(
        cls,
        index: Sequence[int],
        tool_sn: Sequence[str],
        node_name: Sequence[str],
    ) -> 'ToolsManagerConfigDTO':
        """Build the expected tool assignments from the three parallel ``tools.*`` arrays.

        :param index: Physical rack slot indices.
        :param tool_sn: Expected serial number per slot; empty for an unassigned slot.
        :param node_name: Endtool node name per slot; empty for an unassigned slot.
        :raises ValueError: If the arrays differ in length or the assignments are inconsistent.
        """
        lengths = {'tools.index': len(index), 'tools.tool_sn': len(tool_sn), 'tools.node_name': len(node_name)}
        if len(set(lengths.values())) != 1:
            raise ValueError(f'tools parameter arrays must have the same length, got {lengths}')

        return cls(
            slots=tuple(
                ToolSlotDTO(index=slot_index, tool_sn=slot_sn, node_name=slot_node)
                for slot_index, slot_sn, slot_node in zip(index, tool_sn, node_name, strict=True)
            ),
        )

    @model_validator(mode='after')
    def _validate_assignments(self) -> 'ToolsManagerConfigDTO':
        """Reject duplicate slots, serials and nodes, and serial/node names that are not set together."""
        for slot in self.slots:
            if (slot.tool_sn is None) != (slot.node_name is None):
                raise ValueError(
                    f'slot {slot.index}: tool serial number and node name must either both be set '
                    f'or both be empty'
                )

        for label, values in (
            ('slot index', [slot.index for slot in self.slots]),
            ('tool serial number', [slot.tool_sn for slot in self.slots if slot.tool_sn]),
            ('endtool node name', [slot.node_name for slot in self.slots if slot.node_name]),
        ):
            seen: set[object] = set()
            duplicates = sorted({value for value in values if value in seen or seen.add(value)}, key=str)
            if duplicates:
                raise ValueError(f'duplicate {label} in tools parameters: {duplicates}')
        return self

    @property
    def slot_indices(self) -> frozenset[int]:
        """All physical rack slot indices."""
        return frozenset(slot.index for slot in self.slots)

    @property
    def assigned_slots(self) -> tuple[ToolSlotDTO, ...]:
        """Slots that have an expected tool."""
        return tuple(slot for slot in self.slots if slot.tool_sn)

    def get_tool_slot_data(self, tool_sn: str) -> ToolSlotDTO:
        """Get the home slot of a tool.

        :param tool_sn: Serial number of the tool.
        :raises ValueError: If the serial number is not assigned to any slot.
        """
        for slot in self.slots:
            if slot.tool_sn == tool_sn:
                return slot

        raise ValueError(f'tool serial number {tool_sn} not found in any slot')
