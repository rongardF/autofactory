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
"""ToolsManagerNodeConfigDTO — Pydantic v2 DTO for tools manager node parameters."""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field, model_validator

from tools_manager.model.simulation_inventory_dto import SimulationInventoryDTO
from tools_manager.model.tools_manager_config import ToolsManagerConfigDTO


class ToolsManagerNodeConfigDTO(BaseModel):

    model_config = ConfigDict(frozen=True)

    tools_manager_config: ToolsManagerConfigDTO = Field(
        description='Expected tool assignments of the rack.',
    )
    simulation: SimulationInventoryDTO | None = Field(
        default=None,
        description='Initial simulated tool inventory; only set when running in simulation.',
    )
    simulated: bool = Field(
        default=True,
        description='Selects Gazebo spawn (true) vs UR payload path (false).',
    )
    world_name: str = Field(
        default='world',
        description='Gazebo world name.',
    )
    station_model_name: str = Field(
        default='station',
        description='The name of the station model.',
    )
    tool_mount_node_name: str = Field(
        description='Tool mount node name to communicate with.',
    )
    tool_rack_node_name: str = Field(
        description='Tool rack node name to communicate with.',
    )
    movement_controller_node_name: str = Field(
        default="movement_controller",
        description='Movement controller node name to communicate with.',
    )

    @model_validator(mode='after')
    def _simulation_slots_exist(self) -> 'ToolsManagerNodeConfigDTO':
        """Reject simulated tools placed in a slot that does not exist on the rack."""
        if self.simulation is None:
            return self
        unknown = sorted(set(self.simulation.slot_indices) - self.tools_manager_config.slot_indices)
        if unknown:
            raise ValueError(
                f'simulation.tool_rack.index contains slots that are not in tools.index: {unknown}'
            )
        return self
