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
"""Pose and orientation conversion helpers."""

import math
from collections.abc import Sequence

from geometry_msgs.msg import Pose, Quaternion


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert roll-pitch-yaw angles to a quaternion.

    Uses the ZYX (yaw-pitch-roll) composition, as ``tf2`` does.

    :param roll: Rotation about the x axis [rad].
    :param pitch: Rotation about the y axis [rad].
    :param yaw: Rotation about the z axis [rad].
    :returns: The orientation as a ``geometry_msgs/Quaternion``.
    """
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


def pose6_to_pose(pose6: Sequence[float]) -> Pose:
    """Convert ``[x, y, z, roll, pitch, yaw]`` (m, rad) to a ``geometry_msgs/Pose``.

    :param pose6: The 6-DOF pose.
    :returns: The pose message.
    :raises ValueError: If ``pose6`` does not have exactly 6 elements.
    """
    if len(pose6) != 6:
        raise ValueError(f'pose must have 6 elements, got {len(pose6)}')
    x, y, z, roll, pitch, yaw = (float(value) for value in pose6)
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation = rpy_to_quaternion(roll, pitch, yaw)
    return pose


def identity_pose() -> Pose:
    """Return the identity pose (zero translation, unit quaternion)."""
    pose = Pose()
    pose.orientation.w = 1.0
    return pose
