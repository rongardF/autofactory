# Copyright (c) 2026, Movement Controller Contributors
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
"""Unit tests for TrajectoryGrouper blend grouping algorithm."""

import pytest
from geometry_msgs.msg import PoseStamped

from movement_controller.enums.motion_type_enum import MotionTypeEnum
from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO
from movement_controller.utils.trajectory_grouper import TrajectoryGrouper

# Predefined valid UUID4 values for deterministic tests.
_UUID_A = 'aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa'
_UUID_B = 'bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb'
_UUID_C = 'cccccccc-cccc-4ccc-8ccc-cccccccccccc'
_UUID_D = 'dddddddd-dddd-4ddd-8ddd-dddddddddddd'
_UUIDT = [
    '00000000-0000-4000-8000-000000000000',
    '11111111-1111-4111-8111-111111111111',
    '22222222-2222-4222-8222-222222222222',
    '33333333-3333-4333-8333-333333333333',
    '44444444-4444-4444-8444-444444444444',
    '55555555-5555-4555-8555-555555555555',
    '66666666-6666-4666-8666-666666666666',
]


def _p(path_id: str, blend_radius: float) -> TrajectoryPathDTO:
    """Build a minimal valid TrajectoryPathDTO for grouper tests."""
    return TrajectoryPathDTO(
        path_id=path_id,
        motion_type=MotionTypeEnum.LIN,
        target_pose=PoseStamped(),
        blend_radius=blend_radius,
    )


def _pm(path_id: str, motion_type: MotionTypeEnum, blend_radius: float) -> TrajectoryPathDTO:
    """Build a minimal valid TrajectoryPathDTO with an explicit motion type."""
    return TrajectoryPathDTO(
        path_id=path_id,
        motion_type=motion_type,
        target_pose=PoseStamped(),
        blend_radius=blend_radius,
    )


def test_empty_paths_raises():
    """group() raises ValueError when the input list is empty."""
    with pytest.raises(ValueError, match='must not be empty'):
        TrajectoryGrouper.group([])


def test_single_path_always_new_group():
    """A single path always produces exactly one group."""
    groups = TrajectoryGrouper.group([_p(_UUID_A, 0.5)])
    assert len(groups) == 1
    assert len(groups[0]) == 1


def test_all_zero_blend_radius():
    """Each zero-blend-radius path starts its own group."""
    groups = TrajectoryGrouper.group([_p(_UUID_A, 0.0), _p(_UUID_B, 0.0), _p(_UUID_C, 0.0)])
    assert len(groups) == 3


def test_mixed_grouping_separated():
    """Test that mix of blend_radius values produces expected group sizes: 7 paths → 4 groups with sizes [1, 1, 4, 1]."""
    paths = [
        _p(_UUIDT[0], 0.5), _p(_UUIDT[1], 0.0), _p(_UUIDT[2], 0.0),
        _p(_UUIDT[3], 0.3), _p(_UUIDT[4], 0.3), _p(_UUIDT[5], 0.3),
        _p(_UUIDT[6], 0.0),
    ]
    groups = TrajectoryGrouper.group(paths)
    assert len(groups) == 3
    assert len(groups[0]) == 2  # (first path, always new group, blends with next)
    assert len(groups[1]) == 1   # (previous group ended by br=0, new group starts, no blend)
    assert len(groups[2]) == 4   # blended group of 4 paths (br>0 merges with previous group, then 2 more with br>0 merge into that)


def test_first_path_always_starts_new_group_even_with_positive_blend_radius():
    """First path always starts a new group regardless of its blend_radius. 
    Subsequent paths with br>0 all merge into that group.
    """
    groups = TrajectoryGrouper.group([_p(_UUID_A, 0.5), _p(_UUID_B, 0.3), _p(_UUID_C, 0.3)])
    assert len(groups) == 1
    assert len(groups[0]) == 3

def test_item_with_blend_radius_merged_with_next_group():
    """Test that a path with blend_radius > 0 merges with the next group, even if the next path has blend_radius=0."""
    groups = TrajectoryGrouper.group(
        [_p(_UUID_A, 0.5), _p(_UUID_B, 0.0), _p(_UUID_C, 0.3), _p(_UUID_D, 0.3)]
    )
    assert len(groups) == 2
    assert len(groups[0]) == 2
    assert groups[0][0].path_id == _UUID_A
    assert groups[0][1].path_id == _UUID_B
    assert len(groups[1]) == 2
    assert groups[1][0].path_id == _UUID_C
    assert groups[1][1].path_id == _UUID_D


def test_three_separate_groups():
    """br=0 on 3rd path splits into three groups."""
    groups = TrajectoryGrouper.group(
        [_p(_UUID_A, 0.0), _p(_UUID_B, 0.3), _p(_UUID_C, 0.0), _p(_UUID_D, 0.3)]
    )
    assert len(groups) == 3
    assert len(groups[0]) == 1   # A
    assert len(groups[1]) == 2   # B, C
    assert len(groups[2]) == 1   # D


def test_negative_blend_radius_treated_as_zero():
    """DTO normalises -0.5 → 0.0"""
    # After normalisation: A.blend_radius=0.0, B.blend_radius=0.3
    # A is i=0 → new group; B is i=1 with br=0.3>0 and previous br=0.0 → new group]
    groups = TrajectoryGrouper.group([_p(_UUID_A, -0.5), _p(_UUID_B, 0.3)])
    assert len(groups) == 2
    assert len(groups[0]) == 1
    assert len(groups[1]) == 1


def test_single_ptp_is_own_group():
    """A single PTP path forms exactly one single-item group."""
    groups = TrajectoryGrouper.group([_pm(_UUID_A, MotionTypeEnum.PTP, 0.0)])
    assert len(groups) == 1
    assert len(groups[0]) == 1
    assert groups[0][0].path_id == _UUID_A


def test_ptp_blend_radius_ignored_stays_isolated():
    """A PTP with blend_radius > 0 is still isolated; its blend is ignored."""
    groups = TrajectoryGrouper.group(
        [_pm(_UUID_A, MotionTypeEnum.PTP, 0.5), _pm(_UUID_B, MotionTypeEnum.LIN, 0.0)]
    )
    assert len(groups) == 2
    assert len(groups[0]) == 1
    assert groups[0][0].path_id == _UUID_A
    assert len(groups[1]) == 1
    assert groups[1][0].path_id == _UUID_B


def test_ptp_breaks_open_blended_group():
    """A PTP closes an open blended group before starting its own group."""
    # A(0.5, LIN) opens a blend; B is PTP → must not merge into A's group.
    groups = TrajectoryGrouper.group(
        [_pm(_UUID_A, MotionTypeEnum.LIN, 0.5), _pm(_UUID_B, MotionTypeEnum.PTP, 0.0)]
    )
    assert len(groups) == 2
    assert len(groups[0]) == 1
    assert groups[0][0].path_id == _UUID_A
    assert len(groups[1]) == 1
    assert groups[1][0].path_id == _UUID_B


def test_path_after_ptp_starts_new_group():
    """A LIN/CIRC path following a PTP starts a fresh group (never merges into PTP)."""
    groups = TrajectoryGrouper.group(
        [_pm(_UUID_A, MotionTypeEnum.PTP, 0.0), _pm(_UUID_B, MotionTypeEnum.LIN, 0.3)]
    )
    assert len(groups) == 2
    assert len(groups[0]) == 1
    assert groups[0][0].path_id == _UUID_A
    assert len(groups[1]) == 1
    assert groups[1][0].path_id == _UUID_B


def test_mixed_lin_circ_ptp_sequence():
    """LIN(blend)→LIN→PTP→CIRC(blend)→CIRC groups as [ [L,L], [PTP], [C,C] ]."""
    paths = [
        _pm(_UUIDT[0], MotionTypeEnum.LIN, 0.3),
        _pm(_UUIDT[1], MotionTypeEnum.LIN, 0.0),
        _pm(_UUIDT[2], MotionTypeEnum.PTP, 0.0),
        _pm(_UUIDT[3], MotionTypeEnum.CIRC, 0.3),
        _pm(_UUIDT[4], MotionTypeEnum.CIRC, 0.0),
    ]
    groups = TrajectoryGrouper.group(paths)
    assert len(groups) == 3
    assert [p.path_id for p in groups[0]] == [_UUIDT[0], _UUIDT[1]]
    assert [p.path_id for p in groups[1]] == [_UUIDT[2]]
    assert [p.path_id for p in groups[2]] == [_UUIDT[3], _UUIDT[4]]


def test_consecutive_ptp_each_own_group():
    """Consecutive PTP paths each form their own single-item group."""
    groups = TrajectoryGrouper.group(
        [
            _pm(_UUID_A, MotionTypeEnum.PTP, 0.0),
            _pm(_UUID_B, MotionTypeEnum.PTP, 0.0),
            _pm(_UUID_C, MotionTypeEnum.PTP, 0.0),
        ]
    )
    assert len(groups) == 3
    assert all(len(g) == 1 for g in groups)
    assert [g[0].path_id for g in groups] == [_UUID_A, _UUID_B, _UUID_C]
