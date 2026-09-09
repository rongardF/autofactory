# Copyright (c) 2026, Endtools Contributors
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
"""ToolRackNodeConfigDTO — Pydantic v2 DTO for tool rack node parameters."""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field

from tools_manager.model.rack_config import RackConfigDTO


class ToolMountNodeConfigDTO(BaseModel):

    model_config = ConfigDict(frozen=True)

    rack_config: RackConfigDTO = Field(
        description='Tool rack configuration.',
    )
    parent_frame_id: str = Field(
        default="station",
        description='Parent frame ID.',
    )
    
    simulated: bool = Field(
        default=True,
        description='Selects Gazebo spawn (true) vs UR payload path (false).',
    )
    tool_rack_node_name: str = Field(
        description='Tool rack node name to communicate with.',
    )
    endtool_node_names: list[str] = Field(
        default=[],
        description='Endtool node names to communicate with.',
    )
