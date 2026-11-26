# Copyright (c) 2026, Autofactory
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS WARRANTIES ARE DISCLAIMED.
"""Fluid material (for example putty) used by dispensing endtools."""

from typing import Annotated, Literal

from pydantic import Field

from repositories.enums.material_repository.material_type_enum import MaterialTypeEnum
from repositories.models.material_repository.material import Material


class FluidMaterial(Material):
    """A fluid material; all payload values must come from the repository."""

    material_type: Literal[MaterialTypeEnum.FLUID] = Field(
        description="Material type; always 'fluid'.",
    )
    density_kg_m3: float = Field(
        gt=0.0,
        description='Density of the material in kg/m^3.',
    )
    viscosity_pa_s: float = Field(
        gt=0.0,
        description='Dynamic viscosity of the material in Pa*s.',
    )
    color_rgba: list[Annotated[float, Field(ge=0.0, le=1.0)]] = Field(
        min_length=4,
        max_length=4,
        description='Color of the material as [red, green, blue, alpha], each '
        'channel in the range 0.0 to 1.0 (alpha 0.0 is fully transparent).',
    )
