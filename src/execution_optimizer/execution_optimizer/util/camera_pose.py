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

"""Camera orientation and the group feasibility test.

For a fixed orientation every criterion (depth range from the feature
resolution, operational area in pixels) is linear in the camera position ``c``.
With ``v = p - c``, ``Z = f.v``, ``x = r.v``, ``y = d.v`` and ``Z > 0`` each
criterion has the form ``h.(p - c) - m >= 0``; rows are normalised so a slack is a
distance in metres. Pure numpy / scipy, all lengths in metres.
"""

from collections.abc import Sequence

import numpy as np
from scipy.optimize import linprog
from scipy.spatial.transform import Rotation

from execution_optimizer.model.camera_intrinsics_dto import CameraIntrinsicsDTO
from execution_optimizer.model.dispensing_optimizer_config_dto import (
    DispensingOptimizerConfigDTO,
)
from execution_optimizer.model.group_fit_dto import GroupFitDTO
from execution_optimizer.model.pattern_roi_dto import PatternRoiDTO
from execution_optimizer.util.geometry import to_vec3, unique_points

# A group is feasible only with at least this margin: above the LP solver
# tolerance, and it guarantees a strictly interior start for the centring.
FEASIBILITY_MARGIN_M = 1e-6
_MARGIN_CAP_M = 1.0
_ANGLE_SLACK_RAD = 1e-12
_MIN_ROLL_NORM = 1e-3
_PRODUCT_X = np.array([1.0, 0.0, 0.0])
_NEWTON_MAX_ITERATIONS = 50
_NEWTON_DECREMENT_TOL = 1e-14
_NEWTON_MIN_STEP = 1e-12
_VERIFY_RESOLUTION_REL_TOL = 1e-9
_VERIFY_PIXEL_TOL = 1e-6

AreaPx = tuple[int, int, int, int]
Axes = tuple[np.ndarray, np.ndarray, np.ndarray]


def mean_normal(normals: np.ndarray) -> np.ndarray:
    """Unit vector along the sum of the given unit normals."""
    total = normals.sum(axis=0)
    return total / np.linalg.norm(total)


def camera_axes(mean_unit_normal: np.ndarray) -> Axes | None:
    """Forward, image-right and image-down axes for a mean surface normal.

    The optical axis is anti-parallel to the normal; image-right is the product X
    axis projected onto the image plane.

    :returns: ``(forward, right, down)`` unit vectors with ``right x down = forward``,
        or ``None`` if the roll is undefined (normal almost along product X).
    """
    forward = -mean_unit_normal
    right = _PRODUCT_X - (_PRODUCT_X @ forward) * forward
    norm = np.linalg.norm(right)
    if norm < _MIN_ROLL_NORM:
        return None
    right = right / norm
    down = np.cross(forward, right)
    return forward, right, down


def depth_range_m(
    intrinsics: CameraIntrinsicsDTO, resolution_px_per_m: tuple[float, float]
) -> tuple[float, float]:
    """Allowed depth range along the optical axis for the feature resolution range."""
    low, high = resolution_px_per_m
    return (
        max(intrinsics.fx, intrinsics.fy) / high,
        min(intrinsics.fx, intrinsics.fy) / low,
    )


def constraint_rows(
    axes: Axes,
    intrinsics: CameraIntrinsicsDTO,
    area_px: AreaPx,
    z_min: float,
    z_max: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Normalised constraint rows ``h.(p - c) - m >= 0``.

    :returns: ``(h, m)`` with ``h`` of shape ``(6, 3)`` (unit rows) and ``m`` of shape ``(6,)``.
    """
    forward, right, down = axes
    x_min, y_min, x_max, y_max = area_px
    rows = np.array([
        forward,
        -forward,
        intrinsics.fx * right + (intrinsics.cx - x_min) * forward,
        (x_max - intrinsics.cx) * forward - intrinsics.fx * right,
        intrinsics.fy * down + (intrinsics.cy - y_min) * forward,
        (y_max - intrinsics.cy) * forward - intrinsics.fy * down,
    ])
    offsets = np.array([z_min, -z_max, 0.0, 0.0, 0.0, 0.0])
    norms = np.linalg.norm(rows, axis=1)
    return rows / norms[:, None], offsets / norms


def pose_quaternion(forward: np.ndarray, right: np.ndarray, down: np.ndarray) -> tuple[
    float, float, float, float
]:
    """Camera TCP orientation as ``(x, y, z, w)`` with ``w >= 0``.

    The TCP frame follows the Gazebo camera convention: +X optical axis, +Y image-left,
    +Z image-up.
    """
    matrix = np.stack([forward, -right, -down], axis=1)
    quaternion = Rotation.from_matrix(matrix).as_quat()
    if quaternion[3] < 0.0 or (
        quaternion[3] == 0.0 and next((q for q in quaternion if q != 0.0), 0.0) < 0.0
    ):
        quaternion = -quaternion
    return (
        float(quaternion[0]), float(quaternion[1]), float(quaternion[2]), float(quaternion[3])
    )


def analytic_centre(
    rows: np.ndarray, offsets: np.ndarray, start: np.ndarray
) -> np.ndarray:
    """Analytic centre of ``{c : offsets - rows @ c > 0}`` by damped Newton iterations.

    Maximises the sum of the log slacks, which gives a unique, centred and
    deterministic point.

    :param start: Strictly interior starting point.
    :returns: The centre, or ``start`` if the iteration cannot make progress.
    """
    centre = start.copy()
    if np.any(offsets - rows @ centre <= 0.0):
        return start
    for _ in range(_NEWTON_MAX_ITERATIONS):
        slack = offsets - rows @ centre
        weighted = rows / slack[:, None]
        gradient = weighted.sum(axis=0)
        hessian = weighted.T @ weighted
        try:
            step = -np.linalg.solve(hessian, gradient)
        except np.linalg.LinAlgError:
            break
        decrement = -float(gradient @ step)
        if not np.isfinite(decrement) or decrement < _NEWTON_DECREMENT_TOL:
            break
        objective = -np.log(slack).sum()
        alpha = 1.0
        while alpha >= _NEWTON_MIN_STEP:
            new_slack = offsets - rows @ (centre + alpha * step)
            if np.all(new_slack > 0.0) and (
                -np.log(new_slack).sum() <= objective - 0.25 * alpha * decrement
            ):
                break
            alpha *= 0.5
        else:
            break
        centre = centre + alpha * step
    if not np.all(np.isfinite(centre)) or np.any(offsets - rows @ centre <= 0.0):
        return start
    return centre


def fit_group(
    rois: Sequence[PatternRoiDTO],
    config: DispensingOptimizerConfigDTO,
    intrinsics: CameraIntrinsicsDTO,
    area_px: AreaPx,
) -> GroupFitDTO | None:
    """Find a camera position for a group of patterns, if one exists.

    The members must be passed sorted by request index so the result depends only
    on the member set.

    :returns: The placement, or ``None`` if the angle, roll, resolution or
        operational area criteria cannot all be satisfied.
    """
    normals = np.array([roi.normal for roi in rois], dtype=float)
    mean_unit = mean_normal(normals)
    angles = np.arccos(np.clip(normals @ mean_unit, -1.0, 1.0))
    if np.any(angles > config.max_angle_rad + _ANGLE_SLACK_RAD):
        return None
    axes = camera_axes(mean_unit)
    if axes is None:
        return None

    points = unique_points(np.array([corner for roi in rois for corner in roi.corners], float))
    z_min, z_max = depth_range_m(intrinsics, config.resolution_px_per_m)
    rows, offsets = constraint_rows(axes, intrinsics, area_px, z_min, z_max)

    # Row j * N + i encodes constraint j of point i: h_j.c + t <= h_j.p_i - m_j.
    count = len(points)
    matrix = np.empty((6 * count, 4))
    matrix[:, :3] = np.repeat(rows, count, axis=0)
    matrix[:, 3] = 1.0
    bound = (points @ rows.T - offsets).T.reshape(-1)
    result = linprog(
        [0.0, 0.0, 0.0, -1.0],
        A_ub=matrix,
        b_ub=bound,
        bounds=[(None, None)] * 3 + [(None, _MARGIN_CAP_M)],
        method='highs-ds',
    )
    if result.status != 0 or result.x is None or result.x[3] < FEASIBILITY_MARGIN_M:
        return None

    position = analytic_centre(matrix[:, :3], bound, result.x[:3])
    margin = float((bound - matrix[:, :3] @ position).min())
    return GroupFitDTO(
        position=to_vec3(position),
        forward=to_vec3(axes[0]),
        right=to_vec3(axes[1]),
        down=to_vec3(axes[2]),
        margin_m=margin,
    )


def verify_group(
    rois: Sequence[PatternRoiDTO],
    fit: GroupFitDTO,
    config: DispensingOptimizerConfigDTO,
    intrinsics: CameraIntrinsicsDTO,
    area_px: AreaPx,
) -> bool:
    """Independently re-check a group's pose against all criteria.

    Projects every ROI corner with the pinhole model and checks the depth,
    feature resolution, operational area and angle criteria.
    """
    normals = np.array([roi.normal for roi in rois], dtype=float)
    mean_unit = mean_normal(normals)
    if np.any(np.arccos(np.clip(normals @ mean_unit, -1.0, 1.0))
              > config.max_angle_rad + _ANGLE_SLACK_RAD):
        return False

    position = np.array(fit.position)
    forward, right, down = np.array(fit.forward), np.array(fit.right), np.array(fit.down)
    low, high = config.feature_resolution_range
    x_min, y_min, x_max, y_max = area_px
    for roi in rois:
        for corner in roi.corners:
            offset = np.array(corner) - position
            depth = float(forward @ offset)
            if depth <= 0.0:
                return False
            pixel_u = intrinsics.cx + intrinsics.fx * float(right @ offset) / depth
            pixel_v = intrinsics.cy + intrinsics.fy * float(down @ offset) / depth
            res_x = intrinsics.fx / depth / 1000.0
            res_y = intrinsics.fy / depth / 1000.0
            if not (low * (1.0 - _VERIFY_RESOLUTION_REL_TOL) <= min(res_x, res_y)
                    and max(res_x, res_y) <= high * (1.0 + _VERIFY_RESOLUTION_REL_TOL)):
                return False
            if not (x_min - _VERIFY_PIXEL_TOL <= pixel_u <= x_max + _VERIFY_PIXEL_TOL
                    and y_min - _VERIFY_PIXEL_TOL <= pixel_v <= y_max + _VERIFY_PIXEL_TOL):
                return False
    return True
