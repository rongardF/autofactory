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
"""EndtoolDto — runtime state of an endtool the tools manager has taken under management."""

from pydantic import BaseModel, ConfigDict, Field

from tools_manager.model.tool_info_dto import ToolInfoDto
from tools_manager.model.tools_manager_config import ToolSlotDTO
from tools_manager.services.node_state_manager import NodeStateManager


class EndtoolDto(BaseModel):
    """An endtool node together with what has been initialised for it so far."""

    model_config = ConfigDict(extra='forbid', arbitrary_types_allowed=True)

    tool: ToolInfoDto = Field(description='Tool as detected at activation.')
    home_slot: ToolSlotDTO = Field(description='Rack slot the tool belongs to.')
    node_manager: NodeStateManager = Field(description='Lifecycle and parameter client of the endtool node.')
    model_name: str = Field(description='Model (resource directory) name read from the endtool node.')
    mounted: bool = Field(description='Whether the tool was on the tool mount at activation.')
    configured: bool = Field(description='Whether the endtool node was configured by the tools manager.', default=False)
    world_model_created: bool = Field(
        description='Whether the tool model exists in the planning scene and the simulation.',
        default=False,
    )
