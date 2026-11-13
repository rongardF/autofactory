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

"""Configuration of the dispensing optimizer."""

import math

from pydantic import Field, field_validator

from execution_optimizer.model.optimizer_config_dto import OptimizerConfigDTO


class DispensingOptimizerConfigDTO(OptimizerConfigDTO):
    """Immutable dispensing optimizer configuration."""

    feature_resolution_range: tuple[float, float] = Field(
        default=(3.33, 4.16),
        description='Inclusive [low, high] feature resolution range in px/mm.',
    )
    max_angle_deviation: float = Field(
        default=20.0,
        ge=0.0,
        lt=90.0,
        description='Maximum angle in degrees between any ROI normal of a group and the '
        'mean normal of that group.',
    )
    operational_area: tuple[int, int, int, int] | None = Field(
        default=None,
        description='Inclusive [x_min, y_min, x_max, y_max] pixel rectangle (top-left '
        'origin) in which all ROI points must project; None means the whole image.',
    )
    planarity_tolerance_mm: float = Field(
        default=0.5,
        ge=0.0,
        description='Maximum distance in millimetres of pattern points from the fitted '
        'plane or line.',
    )

    @field_validator('feature_resolution_range')
    @classmethod
    def _validate_resolution_range(cls, value: tuple[float, float]) -> tuple[float, float]:
        low, high = value
        if not (math.isfinite(low) and math.isfinite(high) and 0.0 < low <= high):
            raise ValueError('must satisfy 0 < low <= high')
        return value

    @field_validator('operational_area')
    @classmethod
    def _validate_operational_area(
        cls, value: tuple[int, int, int, int] | None
    ) -> tuple[int, int, int, int] | None:
        if value is None:
            return value
        x_min, y_min, x_max, y_max = value
        if min(value) < 0:
            raise ValueError('all values must be >= 0')
        if not (x_min < x_max and y_min < y_max):
            raise ValueError('must satisfy x_min < x_max and y_min < y_max')
        return value

    @property
    def max_angle_rad(self) -> float:
        """Maximum normal deviation in radians."""
        return math.radians(self.max_angle_deviation)

    @property
    def planarity_tolerance_m(self) -> float:
        """Planarity tolerance in metres."""
        return self.planarity_tolerance_mm / 1000.0

    @property
    def resolution_px_per_m(self) -> tuple[float, float]:
        """Feature resolution range converted from px/mm to px/m."""
        low, high = self.feature_resolution_range
        return low * 1000.0, high * 1000.0
