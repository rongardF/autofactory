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

"""Pattern geometry: line / plane fitting and the minimum-area rectangle.

Pure numpy, all lengths in metres.
"""

import numpy as np

from execution_optimizer.exceptions import GeometryFailureEnum, PatternGeometryError
from execution_optimizer.model.pattern_points_dto import PatternPointsDTO
from execution_optimizer.model.pattern_roi_dto import PatternRoiDTO
from execution_optimizer.model.vec3 import Vec3

_DEDUP_DECIMALS = 9
# Lets a tolerance of exactly 0 still classify exactly collinear input as a line.
_LINE_SLACK_M = 1e-12
_HORIZONTAL_NORMAL_EPS = 1e-6
_AREA_TIE_SLACK = 1e-15
_SEGMENT_EPS_M = 1e-12


def to_vec3(values: np.ndarray) -> Vec3:
    """Convert a length-3 array to a plain float tuple."""
    return (float(values[0]), float(values[1]), float(values[2]))


def unique_points(points: np.ndarray) -> np.ndarray:
    """Drop duplicate rows (to 1 nm), keeping the first occurrence and the order."""
    rounded = np.round(points, _DEDUP_DECIMALS)
    _, first = np.unique(rounded, axis=0, return_index=True)
    return points[np.sort(first)]


def convex_hull_2d(points: np.ndarray) -> np.ndarray:
    """Convex hull of 2D points (Andrew's monotone chain).

    :param points: ``(N, 2)`` array.
    :returns: Hull vertices in counter-clockwise order without duplicates; fewer
        than three rows if the input is degenerate.
    """
    unique = sorted({(float(x), float(y)) for x, y in points})
    if len(unique) <= 2:
        return np.array(unique, dtype=float).reshape(-1, 2)

    def cross(o: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[tuple[float, float]] = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper: list[tuple[float, float]] = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    return np.array(lower[:-1] + upper[:-1], dtype=float)


def min_area_rectangle(points_2d: np.ndarray) -> tuple[float, np.ndarray]:
    """Minimum-area enclosing rectangle of 2D points.

    One side of an optimal rectangle is collinear with a convex hull edge, so
    every hull edge direction is tried.

    :param points_2d: ``(N, 2)`` array.
    :returns: ``(area, corners)`` with ``corners`` a ``(4, 2)`` array.
    """
    hull = convex_hull_2d(points_2d)
    if len(hull) < 3:
        low, high = hull[0], hull[-1]
        return 0.0, np.array([low, high, high, low])

    edges = np.roll(hull, -1, axis=0) - hull
    edges = edges / np.linalg.norm(edges, axis=1)[:, None]
    best: tuple[float, np.ndarray, np.ndarray, float, float, float, float] | None = None
    for along in edges:
        across = np.array([-along[1], along[0]])
        proj_a = hull @ along
        proj_b = hull @ across
        a_min, a_max = float(proj_a.min()), float(proj_a.max())
        b_min, b_max = float(proj_b.min()), float(proj_b.max())
        area = (a_max - a_min) * (b_max - b_min)
        if best is None or area < best[0] - _AREA_TIE_SLACK:
            best = (area, along, across, a_min, a_max, b_min, b_max)
    assert best is not None
    area, along, across, a_min, a_max, b_min, b_max = best
    corners = np.array([
        along * a_min + across * b_min,
        along * a_max + across * b_min,
        along * a_max + across * b_max,
        along * a_min + across * b_max,
    ])
    return area, corners


def build_pattern_roi(
    pattern: PatternPointsDTO, index: int, planarity_tolerance_m: float
) -> PatternRoiDTO:
    """Compute the region of interest of one pattern.

    :param pattern: Pattern points in the product frame (metres).
    :param index: Position of the pattern in the request.
    :param planarity_tolerance_m: Maximum point distance from the fitted line / plane.
    :raises PatternGeometryError: If the pattern is not planar or its normal is horizontal.
    """
    points = unique_points(np.asarray(pattern.points, dtype=float).reshape(-1, 3))
    centre = points.mean(axis=0)
    centred = points - centre

    if len(points) == 1:
        return _degenerate_roi(pattern.id, index, centre[None, :])

    _, _, vt = np.linalg.svd(centred, full_matrices=True)
    direction = vt[0]
    along = centred @ direction
    off_line = centred - np.outer(along, direction)
    if np.linalg.norm(off_line, axis=1).max() <= planarity_tolerance_m + _LINE_SLACK_M:
        ends = np.array([centre + along.min() * direction, centre + along.max() * direction])
        if along.max() - along.min() < _SEGMENT_EPS_M:
            ends = centre[None, :]
        return _degenerate_roi(pattern.id, index, ends)

    normal = vt[2]
    if np.abs(centred @ normal).max() > planarity_tolerance_m:
        raise PatternGeometryError(pattern.id, GeometryFailureEnum.NON_PLANAR)
    if normal[2] < 0.0:
        normal = -normal
    if normal[2] < _HORIZONTAL_NORMAL_EPS:
        raise PatternGeometryError(pattern.id, GeometryFailureEnum.HORIZONTAL_NORMAL)

    axis_u, axis_v = vt[0], vt[1]
    plane_points = np.stack([centred @ axis_u, centred @ axis_v], axis=1)
    area, rectangle = min_area_rectangle(plane_points)
    corners = centre + np.outer(rectangle[:, 0], axis_u) + np.outer(rectangle[:, 1], axis_v)
    return PatternRoiDTO(
        id=pattern.id,
        index=index,
        normal=to_vec3(normal),
        corners=tuple(to_vec3(corner) for corner in corners),
        area_m2=float(area),
        is_degenerate=False,
    )


def _degenerate_roi(pattern_id: str, index: int, corners: np.ndarray) -> PatternRoiDTO:
    """ROI of a collinear or single-point pattern: the product +Z normal and zero area."""
    return PatternRoiDTO(
        id=pattern_id,
        index=index,
        normal=(0.0, 0.0, 1.0),
        corners=tuple(to_vec3(corner) for corner in corners),
        area_m2=0.0,
        is_degenerate=True,
    )
