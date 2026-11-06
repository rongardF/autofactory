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
"""Pre-spawn intersection check of the product and carrier collision geometry."""

import itertools
import re
from typing import NamedTuple

import fcl
import numpy as np
import trimesh

from station_workplace.exceptions.spawn_error import SpawnError
from station_workplace.models.sdf_model_dto import (
    BoxGeometryDTO,
    CylinderGeometryDTO,
    MeshGeometryDTO,
    SdfCollisionDTO,
    SdfModelDTO,
    SphereGeometryDTO,
)
from station_workplace.utils.transformations import pose6_to_matrix

_MAX_CONTACTS = 1000
_UNITS_PATTERN = re.compile(r'^\s*([0-9.eE+-]+)\s*\*\s*meters?\s*$')


class _CollisionPart(NamedTuple):
    """One collision element placed in the carrier model frame."""

    name: str
    geometry: fcl.CollisionGeometry
    pose: np.ndarray
    is_mesh: bool


def _separation_directions() -> np.ndarray:
    """Return the 26 unit axis/diagonal directions of a frame."""
    directions = [
        np.array(d, dtype=float) for d in itertools.product((-1, 0, 1), repeat=3) if any(d)
    ]
    return np.array([d / np.linalg.norm(d) for d in directions])


class CollisionChecker:
    """Check that the product collision geometry does not penetrate the carrier.

    Pairs of primitives use FCL's exact penetration depth. FCL's depth of pairs
    involving a mesh is per triangle and therefore unusable; such a colliding
    pair is accepted only if moving the product by the tolerance along one of
    the 26 axis/diagonal directions of the carrier or product frame separates
    it. This never accepts a penetration deeper than the tolerance but may
    reject zero-clearance fits touching several non-axis-aligned faces. Meshes
    are triangle soups, so a mesh completely inside another geometry without
    crossing its surface is not detected.
    """

    _DIRECTIONS = _separation_directions()
    _STEP_MARGIN = 1e-6

    def __init__(self, penetration_tolerance: float) -> None:
        """Create the checker.

        :param penetration_tolerance: Largest penetration depth [m] treated as touching.
        """
        self._tolerance = penetration_tolerance
        self._request = fcl.CollisionRequest(num_max_contacts=_MAX_CONTACTS, enable_contact=True)

    def check(
        self, carrier: SdfModelDTO, product: SdfModelDTO, product_in_carrier: np.ndarray,
    ) -> None:
        """Raise if the product placed at ``product_in_carrier`` penetrates the carrier.

        :param carrier: Carrier model; its model frame is the reference frame.
        :param product: Product model.
        :param product_in_carrier: 4x4 pose of the product model frame in the
            carrier model frame.
        :raises SpawnError: If a model has no collision geometry, a geometry
            cannot be loaded or the models penetrate deeper than the tolerance.
        """
        carrier_parts = self._build_parts(carrier, np.eye(4))
        product_parts = self._build_parts(product, product_in_carrier)
        rotation = product_in_carrier[:3, :3]
        offsets = (self._tolerance + self._STEP_MARGIN) * np.vstack(
            (self._DIRECTIONS, self._DIRECTIONS @ rotation.T))
        for carrier_part in carrier_parts:
            for product_part in product_parts:
                result = self._collide(carrier_part, product_part, np.zeros(3))
                if not result.is_collision:
                    continue
                if carrier_part.is_mesh or product_part.is_mesh:
                    if not any(
                        not self._collide(carrier_part, product_part, offset).is_collision
                        for offset in offsets
                    ):
                        raise SpawnError(
                            f"product intersects carrier: '{product_part.name}' penetrates "
                            f"'{carrier_part.name}' deeper than the tolerance "
                            f'({self._tolerance:.4f} m)')
                    continue
                depth = max((c.penetration_depth for c in result.contacts), default=float('inf'))
                if depth > self._tolerance:
                    raise SpawnError(
                        f"product intersects carrier: '{product_part.name}' penetrates "
                        f"'{carrier_part.name}' by {depth:.4f} m "
                        f'(tolerance {self._tolerance:.4f} m)')

    def _collide(
        self, carrier_part: _CollisionPart, product_part: _CollisionPart, offset: np.ndarray,
    ) -> fcl.CollisionResult:
        carrier_object = fcl.CollisionObject(
            carrier_part.geometry,
            fcl.Transform(carrier_part.pose[:3, :3], carrier_part.pose[:3, 3]))
        product_object = fcl.CollisionObject(
            product_part.geometry,
            fcl.Transform(product_part.pose[:3, :3], product_part.pose[:3, 3] + offset))
        result = fcl.CollisionResult()
        fcl.collide(carrier_object, product_object, self._request, result)
        return result

    def _build_parts(self, model: SdfModelDTO, model_pose: np.ndarray) -> list[_CollisionPart]:
        if not model.collisions:
            raise SpawnError(f"model '{model.name}' has no collision geometry")
        parts = []
        for collision in model.collisions:
            link_pose = pose6_to_matrix(model.link_poses[collision.link_name])
            parts.append(_CollisionPart(
                name=f'{model.name}::{collision.link_name}::{collision.name}',
                geometry=self._geometry(collision),
                pose=model_pose @ link_pose @ pose6_to_matrix(collision.pose),
                is_mesh=isinstance(collision.geometry, MeshGeometryDTO),
            ))
        return parts

    @staticmethod
    def _geometry(collision: SdfCollisionDTO) -> fcl.CollisionGeometry:
        geometry = collision.geometry
        if isinstance(geometry, BoxGeometryDTO):
            return fcl.Box(*geometry.size)
        if isinstance(geometry, CylinderGeometryDTO):
            return fcl.Cylinder(geometry.radius, geometry.length)
        if isinstance(geometry, SphereGeometryDTO):
            return fcl.Sphere(geometry.radius)
        if isinstance(geometry, MeshGeometryDTO):
            return CollisionChecker._mesh(geometry)
        raise SpawnError(f'unsupported collision geometry {type(geometry).__name__}')

    @staticmethod
    def _mesh(geometry: MeshGeometryDTO) -> fcl.BVHModel:
        try:
            mesh = trimesh.load(geometry.path, force='mesh')
        except Exception as exc:  # noqa: BLE001 - any loader failure is a spawn failure
            raise SpawnError(f"failed to load mesh '{geometry.path}': {exc}") from exc
        if not isinstance(mesh, trimesh.Trimesh) or len(mesh.faces) == 0:
            raise SpawnError(f"mesh '{geometry.path}' contains no triangles")
        # Gazebo applies the COLLADA <unit meter="..."/>; trimesh only records it.
        unit = 1.0
        match = _UNITS_PATTERN.match(str(mesh.metadata.get('units', '') or ''))
        if match:
            unit = float(match.group(1))
        vertices = np.asarray(mesh.vertices, dtype=float) * unit * np.asarray(geometry.scale)
        faces = np.asarray(mesh.faces, dtype=np.int32)
        model = fcl.BVHModel()
        model.beginModel(len(vertices), len(faces))
        model.addSubModel(vertices, faces)
        model.endModel()
        return model
