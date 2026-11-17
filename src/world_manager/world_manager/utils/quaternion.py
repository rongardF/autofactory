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
"""Quaternion and pose composition helpers."""

from math import sqrt

from world_manager.models.pose_dto import PoseDTO


def _multiply(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    """Hamilton product of two (x, y, z, w) quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate(q: tuple[float, float, float, float], v: tuple[float, float, float]) -> tuple[float, float, float]:
    """Rotate vector ``v`` by the unit quaternion ``q``."""
    qx, qy, qz, qw = q
    conj = (-qx, -qy, -qz, qw)
    x, y, z, _ = _multiply(_multiply(q, (v[0], v[1], v[2], 0.0)), conj)
    return x, y, z


def normalize(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    """Return the unit quaternion; raise ValueError for a zero quaternion."""
    norm = sqrt(sum(c * c for c in q))
    if norm == 0.0:
        raise ValueError('zero-length quaternion')
    return q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm


def compose(parent: PoseDTO, child: PoseDTO, frame_id: str) -> PoseDTO:
    """Express ``child`` (given in the frame of ``parent``) in the frame the parent is given in.

    ``parent`` is the pose of the child's frame in the target frame; the result carries ``frame_id``.
    """
    pq = normalize((parent.qx, parent.qy, parent.qz, parent.qw))
    cq = normalize((child.qx, child.qy, child.qz, child.qw))
    rx, ry, rz = _rotate(pq, (child.x, child.y, child.z))
    qx, qy, qz, qw = normalize(_multiply(pq, cq))
    return PoseDTO(
        frame_id=frame_id,
        x=parent.x + rx, y=parent.y + ry, z=parent.z + rz,
        qx=qx, qy=qy, qz=qz, qw=qw,
    )
