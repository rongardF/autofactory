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
"""Round-trip tests for ``OperationDTO`` message conversion."""

import math
from uuid import uuid4

import pytest

from repositories.models.operation_repository.operation_dto import OperationDTO
from repositories.utils.pose_converter import (
    pose6_to_pose_stamped,
    pose_stamped_to_pose6,
    quaternion_to_rpy,
    rpy_to_quaternion,
)


def _operation_data(pitch: float) -> dict:
    movement = {
        'id': str(uuid4()),
        'motion_type': 'LIN',
        'target_pose_m': [0.1, 0.2, 0.3, 0.4, pitch, 0.6],
        'blend_radius_m': None,
        'cartesian_speed_m_s': 0.05,
        'joint_speed_rad_s': None,
        'circ_type': None,
        'circ_point_m': None,
        'actions': [
            {'type': 'wait', 'timing': 'end', 'action_args': {'wait_time_s': 0.5}},
        ],
    }
    return {
        'id': str(uuid4()),
        'operation_type': 'dispensing',
        'operation_number': '10',
        'description': 'test',
        'start_assembly_model_id': str(uuid4()),
        'end_assembly_model_id': str(uuid4()),
        'material': {'product_number': 'M1', 'product_revision': 'A'},
        'end_tool': {'product_number': 'T1', 'product_revision': 'B'},
        'operation_args': {'speed': 1.5},
        'patterns': [
            {
                'id': str(uuid4()),
                'start_pose_m': [0.0, 0.0, 0.1, 0.0, 0.0, 0.0],
                'pattern_args': {'flowrate_m3_s': 1e-9},
                'movements': [movement],
            }
        ],
    }


def _same_orientation(first, second) -> bool:
    dot = (
        first.x * second.x + first.y * second.y + first.z * second.z + first.w * second.w
    )
    return abs(abs(dot) - 1.0) < 1e-9


@pytest.mark.parametrize('pitch', [0.3, math.pi / 2, -math.pi / 2])
def test_operation_round_trip(pitch):
    original = OperationDTO.from_operation_data(_operation_data(pitch))
    first = original.to_msg('frame_a')
    restored = OperationDTO.from_msg(first)
    second = restored.to_msg('frame_a')

    assert restored.id == original.id
    assert restored.material == original.material
    assert restored.end_tool == original.end_tool
    assert restored.operation_args == original.operation_args
    assert len(second.dispensing_patterns) == 1
    for before, after in zip(
        first.dispensing_patterns[0].movements, second.dispensing_patterns[0].movements
    ):
        assert after.motion_type == before.motion_type
        assert after.blend_radius_m == -1.0
        assert after.cartesian_speed_m_s == pytest.approx(0.05)
        assert after.target_pose.pose.position.x == pytest.approx(0.1)
        assert _same_orientation(
            before.target_pose.pose.orientation, after.target_pose.pose.orientation
        )
        assert [a.type for a in after.actions] == ['wait']


def test_from_msg_restamps_frame():
    message = OperationDTO.from_operation_data(_operation_data(0.2)).to_msg('old')
    restored = OperationDTO.from_msg(message)
    assert restored.to_msg('new').dispensing_patterns[0].start_pose.header.frame_id == 'new'


@pytest.mark.parametrize('pitch', [0.0, 0.7, -1.2, math.pi / 2, -math.pi / 2])
def test_quaternion_rpy_round_trip_preserves_orientation(pitch):
    quaternion = rpy_to_quaternion(0.4, pitch, -0.9)
    rpy = quaternion_to_rpy(*quaternion)
    restored = rpy_to_quaternion(*rpy)
    dot = sum(a * b for a, b in zip(quaternion, restored))
    assert abs(abs(dot) - 1.0) < 1e-9


def test_pose_stamped_to_pose6_inverse():
    pose = (0.1, -0.2, 0.3, 0.1, 0.2, 0.3)
    restored = pose_stamped_to_pose6(pose6_to_pose_stamped(pose, 'f'))
    assert restored == pytest.approx(pose)
