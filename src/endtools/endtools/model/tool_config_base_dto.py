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
"""ToolConfigBaseDTO — Pydantic v2 DTO for tool parameters."""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field, field_validator

from endtools.enumerator.tool_type_enum import ToolTypeEnum


class ToolConfigBaseDTO(BaseModel):

    model_config = ConfigDict(frozen=True)

    tool_type: ToolTypeEnum = Field(
        description='Tool type, used to select the correct tool driver.',
    )
    tool_sn: str = Field(
        description='Tool serial number.',
    )

    tcp_frame_id: str = Field(
        default='tool0',
        description='Parent frame for the published TCP frames. Both the '
        '"{tool_sn}_calibrated_tcp" and "{tool_sn}_uncalibrated_tcp" frames are '
        'published relative to this frame.',
    )
    tcp_uncalibrated: list[float] = Field(
        default=[0.0837, 0.0, -0.267, 1.570797, 0.0, 1.570797],
        description='Nominal (uncalibrated) tip pose relative to tcp_frame_id as [x, y, z, roll, pitch, yaw] (m, rad).',
    )
    
    simulated: bool = Field(
        default=True,
        description='Selects Gazebo spawn (true) vs UR payload path (false).',
    )
    mounted: bool = Field(
        default=False,
        description='Whether the tool is mounted. Drives mount/unmount side-effects; read only at configure time.',
    )

    @field_validator('tcp_uncalibrated')
    @classmethod
    def _validate_tcp_length(cls, value: list[float]) -> list[float]:
        """Ensure the TCP transform has exactly 6 elements [x, y, z, roll, pitch, yaw]."""
        if len(value) != 6:
            raise ValueError(
                f'tcp_uncalibrated must have exactly 6 elements [x, y, z, roll, pitch, yaw], got {len(value)}'
            )
        return value

    @property
    def calibrated_tcp_frame_id(self) -> str:
        """Return the calibrated TCP frame ID based on the tool serial number."""
        return f"{self.tool_sn}_calibrated_tcp"

    @property
    def uncalibrated_tcp_frame_id(self) -> str:
        """Return the uncalibrated TCP frame ID based on the tool serial number."""
        return f"{self.tool_sn}_uncalibrated_tcp"
