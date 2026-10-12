# Copyright (c) 2026, Endtools Contributors
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
"""Pure transform helpers for the tool rack static frames.

These functions produce ROS messages but are node-free, so they can be unit
tested without an rclpy runtime.
"""

from __future__ import annotations

import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped

from tools_manager.model.rack_config import Pose6


def rpy_to_quaternion(
    roll: float, pitch: float, yaw: float
) -> tuple[float, float, float, float]:
    """Convert intrinsic roll-pitch-yaw (XYZ) Euler angles to a quaternion.

    Uses the standard ZYX (yaw-pitch-roll) composition so the result matches
    the convention used by ``tf2`` / ``tf_transformations``.

    :param roll: Rotation about the x axis (radians).
    :param pitch: Rotation about the y axis (radians).
    :param yaw: Rotation about the z axis (radians).
    :returns: Quaternion as ``(x, y, z, w)``.
    :rtype: tuple[float, float, float, float]
    """
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy

    return (x, y, z, w)


def pose6_to_transform_stamped(
    parent_frame_id: str,
    child_frame_id: str,
    pose: Pose6,
    stamp: Time | None = None,
) -> TransformStamped:
    """Build a :class:`TransformStamped` from a 6-DOF pose tuple.

    :param parent_frame_id: Frame the transform is expressed in (header frame).
    :param child_frame_id: Name of the frame the transform defines.
    :param pose: Pose as ``(x, y, z, roll, pitch, yaw)`` with metres and radians.
    :param stamp: Optional timestamp to place in the header; left as the default
        (zero) when ``None``.
    :returns: The populated transform message.
    :rtype: geometry_msgs.msg.TransformStamped
    """
    x, y, z, roll, pitch, yaw = pose

    transform = TransformStamped()
    if stamp is not None:
        transform.header.stamp = stamp
    transform.header.frame_id = parent_frame_id
    transform.child_frame_id = child_frame_id

    transform.transform.translation.x = float(x)
    transform.transform.translation.y = float(y)
    transform.transform.translation.z = float(z)

    qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)
    transform.transform.rotation.x = qx
    transform.transform.rotation.y = qy
    transform.transform.rotation.z = qz
    transform.transform.rotation.w = qw

    return transform
