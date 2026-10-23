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

from pydantic import BaseModel, ConfigDict, Field, computed_field


class PatternExecutorConfigDTO(BaseModel):
    """Immutable configuration shared by all pattern executor implementations."""

    model_config = ConfigDict(frozen=True, str_strip_whitespace=True)

    tool_sn: str = Field(
        ...,
        min_length=1,
        description='End-tool serial number used to derive calibrated TCP frame.',
    )
    endtool_node_name: str = Field(
        ...,
        min_length=1,
        description='Node name of the lifecycle-managed end tool.',
    )
    movement_action_name: str = Field(
        default='movement_controller/execute_trajectory',
        min_length=1,
        description='Action name of the movement controller execute action.',
    )
    movement_server_timeout_s: float = Field(
        default=5.0,
        gt=0.0,
        description='Timeout in seconds used for movement action server availability checks.',
    )
    endtool_service_timeout_s: float = Field(
        default=5.0,
        gt=0.0,
        description='Timeout in seconds used for end-tool lifecycle and parameter service calls.',
    )
    simulated: bool = Field(
        default=True,
        description='Simulation mode flag reserved for future simulation-specific behavior.',
    )

    @computed_field(return_type=str)
    @property
    def calibrated_tcp_frame(self) -> str:
        """Frame id used as TrajectoryPath.tool_frame for this executor instance."""
        return f'{self.tool_sn}_calibrated_tcp'
