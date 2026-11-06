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
"""Collision-relevant content of an expanded SDF model."""

from typing import Annotated, Literal, Union

from pydantic import BaseModel, ConfigDict, Field

from station_workplace.enums.geometry_type_enum import GeometryTypeEnum

FiniteFloat = Annotated[float, Field(allow_inf_nan=False)]
PositiveFloat = Annotated[float, Field(gt=0, allow_inf_nan=False)]
Pose6 = tuple[FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat]


class BoxGeometryDTO(BaseModel):
    """SDF ``<box>`` collision geometry."""

    model_config = ConfigDict(frozen=True, extra='forbid')

    type: Literal[GeometryTypeEnum.BOX] = Field(
        default=GeometryTypeEnum.BOX, description='Geometry type.')
    size: tuple[PositiveFloat, PositiveFloat, PositiveFloat] = Field(
        description='Box size along X, Y, Z [m].')


class CylinderGeometryDTO(BaseModel):
    """SDF ``<cylinder>`` collision geometry (axis along Z)."""

    model_config = ConfigDict(frozen=True, extra='forbid')

    type: Literal[GeometryTypeEnum.CYLINDER] = Field(
        default=GeometryTypeEnum.CYLINDER, description='Geometry type.')
    radius: PositiveFloat = Field(description='Cylinder radius [m].')
    length: PositiveFloat = Field(description='Cylinder length along Z [m].')


class SphereGeometryDTO(BaseModel):
    """SDF ``<sphere>`` collision geometry."""

    model_config = ConfigDict(frozen=True, extra='forbid')

    type: Literal[GeometryTypeEnum.SPHERE] = Field(
        default=GeometryTypeEnum.SPHERE, description='Geometry type.')
    radius: PositiveFloat = Field(description='Sphere radius [m].')


class MeshGeometryDTO(BaseModel):
    """SDF ``<mesh>`` collision geometry with its URI resolved to a local file."""

    model_config = ConfigDict(frozen=True, extra='forbid')

    type: Literal[GeometryTypeEnum.MESH] = Field(
        default=GeometryTypeEnum.MESH, description='Geometry type.')
    path: str = Field(min_length=1, description='Absolute path of the mesh file.')
    scale: tuple[PositiveFloat, PositiveFloat, PositiveFloat] = Field(
        default=(1.0, 1.0, 1.0), description='Mesh scale along X, Y, Z.')


CollisionGeometryDTO = Annotated[
    Union[BoxGeometryDTO, CylinderGeometryDTO, SphereGeometryDTO, MeshGeometryDTO],
    Field(discriminator='type'),
]


class SdfCollisionDTO(BaseModel):
    """One ``<collision>`` element of a link."""

    model_config = ConfigDict(frozen=True, extra='forbid')

    link_name: str = Field(description='Name of the link owning the collision.')
    name: str = Field(default='', description='Name of the collision element.')
    pose: Pose6 = Field(
        default=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        description='Collision pose [x, y, z, roll, pitch, yaw] (m, rad) in the link frame.')
    geometry: CollisionGeometryDTO = Field(description='Collision geometry.')


class SdfModelDTO(BaseModel):
    """Links and collision geometry of a single-model SDF document."""

    model_config = ConfigDict(frozen=True, extra='forbid')

    name: str = Field(description='Model name in the SDF document.')
    link_poses: dict[str, Pose6] = Field(
        default_factory=dict,
        description='Link poses [x, y, z, roll, pitch, yaw] (m, rad) in the model frame.')
    collisions: tuple[SdfCollisionDTO, ...] = Field(
        default=(), description='Collision elements of all links.')
