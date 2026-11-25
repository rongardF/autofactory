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
"""Data models of the ``material_repository`` node.

``Material``, ``FluidMaterial`` and ``ScrewMaterial`` are the payload models
imported by the nodes consuming material data.
"""

from repositories.models.material_repository.fluid_material import FluidMaterial
from repositories.models.material_repository.material import Material
from repositories.models.material_repository.material_data_dto import (
    MaterialDataDTO,
    material_key,
)
from repositories.models.material_repository.material_repository_config_dto import (
    MaterialRepositoryConfigDTO,
)
from repositories.models.material_repository.screw_material import ScrewMaterial

__all__ = [
    'FluidMaterial',
    'Material',
    'MaterialDataDTO',
    'MaterialRepositoryConfigDTO',
    'ScrewMaterial',
    'material_key',
]
