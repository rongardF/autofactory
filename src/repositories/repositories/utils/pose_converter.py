# Copyright (c) 2026, Autofactory
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS WARRANTIES ARE DISCLAIMED.
"""Helpers for converting meter/radian poses into ROS geometry messages."""

from collections.abc import Sequence
import math

from geometry_msgs.msg import Point, PoseStamped


def rpy_to_quaternion(
    roll: float, pitch: float, yaw: float
) -> tuple[float, float, float, float]:
    """Convert fixed-axis XYZ roll, pitch and yaw to quaternion components."""
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def pose6_to_pose_stamped(
    pose: Sequence[float], frame_id: str
) -> PoseStamped:
    """Convert a six-value XYZ/RPY pose into a stamped ROS pose."""
    if len(pose) != 6:
        raise ValueError(f'pose must contain exactly 6 numbers (got {len(pose)})')
    message = PoseStamped()
    message.header.frame_id = frame_id
    message.pose.position.x, message.pose.position.y, message.pose.position.z = pose[:3]
    (
        message.pose.orientation.x,
        message.pose.orientation.y,
        message.pose.orientation.z,
        message.pose.orientation.w,
    ) = rpy_to_quaternion(*pose[3:])
    return message


def point3_to_point(point: Sequence[float]) -> Point:
    """Convert a three-value point into a ROS point."""
    if len(point) != 3:
        raise ValueError(f'point must contain exactly 3 numbers (got {len(point)})')
    message = Point()
    message.x, message.y, message.z = point
    return message
