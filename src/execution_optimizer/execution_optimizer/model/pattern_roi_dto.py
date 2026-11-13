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

"""Region of interest of one pattern."""

from pydantic import BaseModel, ConfigDict, Field

from execution_optimizer.model.vec3 import Vec3


class PatternRoiDTO(BaseModel):
    """Immutable region of interest (minimum-area rectangle) of one pattern."""

    model_config = ConfigDict(frozen=True)

    id: str = Field(..., description='Pattern id as received in the request.')
    index: int = Field(..., ge=0, description='Position of the pattern in the request.')
    normal: Vec3 = Field(
        ..., description='Unit surface normal in the product frame with a positive z component.'
    )
    corners: tuple[Vec3, ...] = Field(
        ...,
        min_length=1,
        max_length=4,
        description='ROI corners in metres in the product frame (4 for a rectangle, 2 for '
        'a segment, 1 for a point).',
    )
    area_m2: float = Field(..., ge=0.0, description='ROI area in square metres.')
    is_degenerate: bool = Field(
        default=False, description='True when the ROI is a segment or a point.'
    )
