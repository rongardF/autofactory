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
"""Unit tests for ToolFrameResolver tool-frame to robot-link recomposition."""

import math

import pytest
from geometry_msgs.msg import Point, PoseStamped, TransformStamped

from movement_controller.utils.tool_frame_resolver import ToolFrameResolver

# Rigid offset giving the link's pose in the tool frame. Pure translation keeps
# the expected values easy to reason about; orientation coverage is provided by
# the quarter-turn case below.
_OFFSET_XYZ = (0.1, -0.2, 0.3)


def _offset(
    translation: tuple[float, float, float] = _OFFSET_XYZ,
    rotation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
) -> TransformStamped:
    """Build a TransformStamped for the link expressed in the tool frame."""
    offset = TransformStamped()
    offset.transform.translation.x = translation[0]
    offset.transform.translation.y = translation[1]
    offset.transform.translation.z = translation[2]
    (
        offset.transform.rotation.x,
        offset.transform.rotation.y,
        offset.transform.rotation.z,
        offset.transform.rotation.w,
    ) = rotation
    return offset


def _pose(
    position: tuple[float, float, float],
    rotation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
) -> PoseStamped:
    """Build a PoseStamped in the ``product_origin`` frame."""
    pose = PoseStamped()
    pose.header.frame_id = 'product_origin'
    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]
    pose.pose.position.z = position[2]
    (
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
    ) = rotation
    return pose


def _circumradius(
    p1: tuple[float, float, float],
    p2: tuple[float, float, float],
    p3: tuple[float, float, float],
) -> float:
    """Return the radius of the circle through three non-collinear points."""
    a = [p2[i] - p1[i] for i in range(3)]
    b = [p3[i] - p1[i] for i in range(3)]
    cross = (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )
    len_a = math.sqrt(sum(v * v for v in a))
    len_b = math.sqrt(sum(v * v for v in b))
    len_c = math.sqrt(sum((b[i] - a[i]) ** 2 for i in range(3)))
    len_cross = math.sqrt(sum(v * v for v in cross))
    return (len_a * len_b * len_c) / (2.0 * len_cross)


def _xyz(point: Point) -> tuple[float, float, float]:
    """Return a Point as a plain tuple."""
    return (point.x, point.y, point.z)


def test_compose_point_applies_translation_offset() -> None:
    """A CIRC arc point is shifted by the same rigid offset as the target pose."""
    target = _pose((0.5, 0.4, 0.3))
    arc_point = Point(x=0.51, y=0.4, z=0.3)

    resolved = ToolFrameResolver._compose_point(arc_point, target, _offset())

    assert resolved.x == pytest.approx(0.51 + _OFFSET_XYZ[0])
    assert resolved.y == pytest.approx(0.40 + _OFFSET_XYZ[1])
    assert resolved.z == pytest.approx(0.30 + _OFFSET_XYZ[2])


def test_compose_point_respects_target_orientation() -> None:
    """The offset is rotated by the target orientation, matching _compose."""
    # 90 degree rotation about Z maps the offset (0.1, -0.2, 0.3) -> (0.2, 0.1, 0.3).
    half = math.sqrt(0.5)
    target = _pose((0.0, 0.0, 0.0), rotation=(0.0, 0.0, half, half))

    resolved = ToolFrameResolver._compose_point(Point(), target, _offset())

    assert resolved.x == pytest.approx(0.2, abs=1e-9)
    assert resolved.y == pytest.approx(0.1, abs=1e-9)
    assert resolved.z == pytest.approx(0.3, abs=1e-9)


def test_compose_point_preserves_circ_arc_radius() -> None:
    """Resolving start, interim and goal together keeps the planned arc congruent.

    Regression test: when only ``target_pose`` was recomposed and ``circ_point``
    was left in tool-frame coordinates, PILZ received a circle inflated by the
    tool offset and failed planning with ``NO_IK_SOLUTION``.
    """
    radius = 0.003
    lift = 0.002
    cx, cy, cz = 0.225, -0.31, 0.144

    start = (cx + radius, cy, cz)
    interim = (cx, cy + radius, cz + lift / 4)
    goal = (cx - radius, cy, cz + lift / 2)
    tool_radius = _circumradius(start, interim, goal)

    offset = _offset()
    start_link = _xyz(ToolFrameResolver._compose(_pose(start), offset).pose.position)
    goal_link = _xyz(ToolFrameResolver._compose(_pose(goal), offset).pose.position)
    interim_link = _xyz(
        ToolFrameResolver._compose_point(
            Point(x=interim[0], y=interim[1], z=interim[2]), _pose(interim), offset
        )
    )

    assert _circumradius(start_link, interim_link, goal_link) == pytest.approx(
        tool_radius, rel=1e-9
    )


def test_unresolved_circ_point_inflates_arc_radius() -> None:
    """Guard the regression: mixing link poses with a tool-frame arc point distorts the arc."""
    radius = 0.003
    lift = 0.002
    cx, cy, cz = 0.225, -0.31, 0.144

    start = (cx + radius, cy, cz)
    interim = (cx, cy + radius, cz + lift / 4)
    goal = (cx - radius, cy, cz + lift / 2)

    offset = _offset()
    start_link = _xyz(ToolFrameResolver._compose(_pose(start), offset).pose.position)
    goal_link = _xyz(ToolFrameResolver._compose(_pose(goal), offset).pose.position)

    broken_radius = _circumradius(start_link, interim, goal_link)
    assert broken_radius > 10 * _circumradius(start, interim, goal)
