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

import numpy as np
from geometry_msgs.msg import Pose, Quaternion, Transform


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


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return the 3x3 rotation ``Rz(yaw) @ Ry(pitch) @ Rx(roll)`` (as in ROS and SDF)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def pose6_to_matrix(pose6: Sequence[float]) -> np.ndarray:
    """Convert ``[x, y, z, roll, pitch, yaw]`` (m, rad) to a 4x4 homogeneous transform.

    :raises ValueError: If ``pose6`` does not have exactly 6 elements.
    """
    if len(pose6) != 6:
        raise ValueError(f'pose must have 6 elements, got {len(pose6)}')
    x, y, z, roll, pitch, yaw = (float(value) for value in pose6)
    matrix = np.eye(4)
    matrix[:3, :3] = rpy_to_matrix(roll, pitch, yaw)
    matrix[:3, 3] = (x, y, z)
    return matrix


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Return the 3x3 rotation of a (normalised) ``(x, y, z, w)`` quaternion.

    :raises ValueError: If the quaternion has zero length.
    """
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError('quaternion has zero length')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def transform_to_matrix(transform: Transform) -> np.ndarray:
    """Convert a ``geometry_msgs/Transform`` to a 4x4 homogeneous transform."""
    rotation = transform.rotation
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_to_matrix(rotation.x, rotation.y, rotation.z, rotation.w)
    translation = transform.translation
    matrix[:3, 3] = (translation.x, translation.y, translation.z)
    return matrix


def matrix_to_pose(matrix: np.ndarray) -> Pose:
    """Convert a 4x4 homogeneous transform to a ``geometry_msgs/Pose``."""
    r = matrix[:3, :3]
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        w, x, y, z = (
            0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s)
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        w, x, y, z = (
            (r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s)
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        w, x, y, z = (
            (r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s)
    else:
        s = 2.0 * math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        w, x, y, z = (
            (r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in matrix[:3, 3])
    pose.orientation = Quaternion(x=float(x), y=float(y), z=float(z), w=float(w))
    return pose
