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
"""Pure transform helpers for the calibrated TCP frame.

These functions are ROS-message-producing but node-free, so they can be unit
tested without an rclpy runtime.
"""

from __future__ import annotations

import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped

from endtools.model.dispensing_tool_config_dto import DispensingToolConfigDTO


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


def quaternion_multiply(
    q1: tuple[float, float, float, float],
    q2: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """Multiply two quaternions ``q1 ⊗ q2`` (Hamilton product).

    Both quaternions are expressed as ``(x, y, z, w)``.

    :param q1: Left-hand quaternion ``(x, y, z, w)``.
    :param q2: Right-hand quaternion ``(x, y, z, w)``.
    :returns: The product quaternion ``(x, y, z, w)``.
    :rtype: tuple[float, float, float, float]
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2

    return (x, y, z, w)


def _rotate_vector(
    q: tuple[float, float, float, float], v: tuple[float, float, float]
) -> tuple[float, float, float]:
    """Rotate a 3D vector by a quaternion ``(x, y, z, w)``.

    :param q: Rotation quaternion ``(x, y, z, w)``.
    :param v: Vector to rotate ``(x, y, z)``.
    :returns: The rotated vector ``(x, y, z)``.
    :rtype: tuple[float, float, float]
    """
    qx, qy, qz, qw = q
    vx, vy, vz = v

    # t = 2 * cross(q_xyz, v)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    # v' = v + qw * t + cross(q_xyz, t)
    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    rz = vz + qw * tz + (qx * ty - qy * tx)

    return (rx, ry, rz)


def compose_calibrated_tcp_transform(
    config: DispensingToolConfigDTO, parent_transform: TransformStamped
) -> TransformStamped:
    """Compose an incoming parent-frame transform with the static TCP offset.

    Given a transform ``base -> tcp_frame_id`` published on ``/tf`` (the
    ``parent_transform``), this returns the transform ``base ->
    calibrated_tcp_frame_id`` by applying the configured TCP offset (the ``tcp``
    matrix) to the parent transform. The resulting transform keeps the parent
    transform's ``header`` (frame_id and stamp) and re-parents the calibrated
    TCP frame under the same base frame.

    :param config: Tool configuration holding the ``tcp`` offset and frame IDs.
    :param parent_transform: Incoming ``base -> tcp_frame_id`` transform.
    :returns: The composed ``base -> calibrated_tcp_frame_id`` transform.
    :rtype: TransformStamped
    """
    x, y, z, roll, pitch, yaw = config.tcp
    q_offset = rpy_to_quaternion(roll, pitch, yaw)

    t_parent = parent_transform.transform.translation
    r_parent = parent_transform.transform.rotation
    q_parent = (r_parent.x, r_parent.y, r_parent.z, r_parent.w)

    # base -> calibrated_tcp rotation is the parent rotation composed with offset
    qx, qy, qz, qw = quaternion_multiply(q_parent, q_offset)

    # base -> calibrated_tcp translation is the parent translation plus the
    # offset translation expressed in the base frame (rotated by parent rotation)
    ox, oy, oz = _rotate_vector(q_parent, (x, y, z))

    composed = TransformStamped()
    composed.header.stamp = parent_transform.header.stamp
    composed.header.frame_id = parent_transform.header.frame_id
    composed.child_frame_id = config.calibrated_tcp_frame_id
    composed.transform.translation.x = t_parent.x + ox
    composed.transform.translation.y = t_parent.y + oy
    composed.transform.translation.z = t_parent.z + oz
    composed.transform.rotation.x = qx
    composed.transform.rotation.y = qy
    composed.transform.rotation.z = qz
    composed.transform.rotation.w = qw

    return composed


def build_tcp_transform(
    config: DispensingToolConfigDTO, stamp: Time
) -> TransformStamped:
    x, y, z, roll, pitch, yaw = config.tcp
    qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)

    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = config.tcp_frame_id
    transform.child_frame_id = config.calibrated_tcp_frame_id
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.translation.z = z
    transform.transform.rotation.x = qx
    transform.transform.rotation.y = qy
    transform.transform.rotation.z = qz
    transform.transform.rotation.w = qw

    return transform
