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
"""Metadata of a product object model."""

from typing import Annotated, Any

from pydantic import BaseModel, ConfigDict, Field, field_validator

from station_workplace.utils.uuid_validator import parse_uuid4

FiniteFloat = Annotated[float, Field(allow_inf_nan=False)]


class ModelMetadataDTO(BaseModel):
    """Workplace-relevant part of a product model's ``metadata_json``.

    Unknown keys are ignored so other consumers may add their own metadata.
    """

    model_config = ConfigDict(frozen=True, extra='ignore')

    carrier_model_id: str = Field(
        description='UUID4 of the object model of the carrier holding the product.',
    )
    estimated_product_origin_pose: tuple[
        FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat
    ] = Field(
        description='Product origin [x, y, z, roll, pitch, yaw] (m, rad) relative to '
        'the carrier link.',
    )

    @field_validator('carrier_model_id')
    @classmethod
    def _validate_carrier_model_id(cls, value: str) -> str:
        """Ensure the carrier model ID is a canonical UUID4 string."""
        return parse_uuid4(value, 'carrier_model_id')

    @field_validator('estimated_product_origin_pose', mode='before')
    @classmethod
    def _reject_booleans(cls, value: Any) -> Any:
        """Reject booleans, which would otherwise be coerced to numbers."""
        if isinstance(value, (list, tuple)) and any(isinstance(v, bool) for v in value):
            raise ValueError('pose elements must be numbers, not booleans')
        return value
