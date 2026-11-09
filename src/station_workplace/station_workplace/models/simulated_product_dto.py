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
"""Validated ``spawn_product`` request."""

from typing import Annotated, Any

from pydantic import BaseModel, ConfigDict, Field, field_validator

from station_workplace.models.product_data_dto import ProductDataDTO
from station_workplace.utils.uuid_validator import parse_uuid4

FiniteFloat = Annotated[float, Field(allow_inf_nan=False)]


class SimulatedProductDTO(BaseModel):
    """Product to spawn into the simulation world."""

    model_config = ConfigDict(frozen=True, extra='forbid')

    product: ProductDataDTO = Field(description='Product identification data.')
    assembly_model_id: str = Field(description='UUID4 of the SDF product model.')
    pose: tuple[
        FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat
    ] = Field(
        description='Product pose [x, y, z, roll, pitch, yaw] (m, rad) relative to the '
        'carrier link of the carrier model.',
    )

    @field_validator('product')
    @classmethod
    def _require_ascii(cls, value: ProductDataDTO) -> ProductDataDTO:
        """Require ASCII product data (written to tags and labels)."""
        for name in ('product_number', 'product_revision', 'serial_number'):
            if not getattr(value, name).isascii():
                raise ValueError(f'{name} must be ASCII')
        return value

    @field_validator('assembly_model_id')
    @classmethod
    def _validate_assembly_model_id(cls, value: str) -> str:
        """Ensure the assembly model ID is a canonical UUID4 string."""
        return parse_uuid4(value, 'assembly_model_id')

    @field_validator('pose', mode='before')
    @classmethod
    def _to_tuple(cls, value: Any) -> Any:
        """Accept any sequence (e.g. ``array.array``) of numbers."""
        if isinstance(value, (str, bytes)):
            return value
        try:
            return tuple(float(v) for v in value)
        except TypeError:
            return value
