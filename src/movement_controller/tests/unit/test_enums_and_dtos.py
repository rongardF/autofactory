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
"""Unit tests for MotionTypeEnum, FeedbackStatusEnum, CircTypeEnum, TrajectoryPathDTO, and TrajectoryGoalDTO."""

import pytest
from geometry_msgs.msg import PoseStamped
from pydantic import ValidationError

from movement_controller.enums.circ_type_enum import CircTypeEnum
from movement_controller.enums.feedback_status_enum import FeedbackStatusEnum
from movement_controller.enums.motion_type_enum import MotionTypeEnum
from movement_controller.models.trajectory_goal_dto import TrajectoryGoalDTO
from movement_controller.models.trajectory_path_dto import TrajectoryPathDTO

# Predefined valid UUID4 values for deterministic tests.
_UUID_A = 'aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa'
_UUID_B = 'bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb'


def _make_path(**overrides) -> TrajectoryPathDTO:
    """Build a minimal valid TrajectoryPathDTO with sensible defaults."""
    defaults = {
        'path_id': _UUID_A,
        'motion_type': MotionTypeEnum.LIN,
        'target_pose': PoseStamped(),
    }
    return TrajectoryPathDTO(**{**defaults, **overrides})

def _make_ros_path_msg(
    path_id=_UUID_A,
    motion_type='LIN',
    circ_type='',
):
    """Build a minimal mock TrajectoryPath ROS message for from_ros_msg tests."""
    from unittest.mock import MagicMock
    from geometry_msgs.msg import Point, PoseStamped
    msg = MagicMock()
    msg.path_id = path_id
    msg.motion_type = motion_type
    msg.circ_type = circ_type
    msg.target_pose = PoseStamped()
    msg.blend_radius = 0.0
    msg.cartesian_speed = 0.0
    msg.acceleration = 0.0
    msg.tool_frame = ''
    msg.circ_point = Point()
    return msg


def test_motion_type_enum_values():
    """MotionTypeEnum members equal the expected string values."""
    assert MotionTypeEnum.LIN == 'LIN'
    assert MotionTypeEnum.PTP == 'PTP'
    assert MotionTypeEnum.CIRC == 'CIRC'
    assert isinstance(MotionTypeEnum.LIN, str)
    assert len(MotionTypeEnum) == 3  # No extra members


def test_feedback_status_enum_values():
    """FeedbackStatusEnum members equal the expected string values."""
    assert FeedbackStatusEnum.EXECUTING == 'executing'
    assert FeedbackStatusEnum.COMPLETED == 'completed'
    assert isinstance(FeedbackStatusEnum.EXECUTING, str)
    assert len(FeedbackStatusEnum) == 2  # No extra members

def test_circ_type_enum_values():
    """CircTypeEnum members equal the expected string values."""
    assert CircTypeEnum.INTERIM == 'interim'
    assert CircTypeEnum.CENTER == 'center'
    assert isinstance(CircTypeEnum.INTERIM, str)
    assert len(CircTypeEnum) == 2  # No extra members


def test_trajectory_path_dto_valid():
    """A fully valid TrajectoryPathDTO can be constructed with correct field values."""
    dto = _make_path(path_id=_UUID_B, blend_radius=0.0)
    assert dto.path_id == _UUID_B
    assert dto.motion_type == MotionTypeEnum.LIN
    assert dto.blend_radius == 0.0


def test_trajectory_path_dto_invalid_motion_type():
    """An invalid motion_type string raises ValidationError."""
    with pytest.raises(ValidationError):
        _make_path(motion_type='INVALID')


def test_trajectory_path_dto_empty_path_id():
    """An empty path_id raises ValidationError."""
    with pytest.raises(ValidationError):
        _make_path(path_id='')


def test_trajectory_path_dto_invalid_uuid4_path_id():
    """A non-UUID4 path_id raises ValidationError."""
    with pytest.raises(ValidationError, match='UUID4'):
        _make_path(path_id='not-a-uuid')


def test_trajectory_path_dto_negative_blend_radius_normalised():
    """A negative blend_radius is normalised to 0.0."""
    dto = _make_path(blend_radius=-0.5)
    assert dto.blend_radius == 0.0


def test_trajectory_path_dto_zero_blend_radius_unchanged():
    """A zero blend_radius is preserved as 0.0."""
    dto = _make_path(blend_radius=0.0)
    assert dto.blend_radius == 0.0


def test_trajectory_path_dto_positive_blend_radius_unchanged():
    """A positive blend_radius is preserved unchanged."""
    dto = _make_path(blend_radius=0.3)
    assert dto.blend_radius == 0.3


def test_trajectory_path_dto_frozen():
    """TrajectoryPathDTO is immutable — assignment raises an error."""
    dto = _make_path()
    with pytest.raises((ValidationError, TypeError)):
        dto.path_id = 'new-id'


def test_trajectory_path_dto_circ_point_default():
    """circ_point defaults to an empty Point when not supplied."""
    from geometry_msgs.msg import Point
    dto = _make_path()
    assert isinstance(dto.circ_point, Point)


def test_trajectory_goal_dto_empty_paths():
    """TrajectoryGoalDTO raises ValidationError for an empty paths list."""
    with pytest.raises(ValidationError):
        TrajectoryGoalDTO(paths=[])


def test_trajectory_goal_dto_valid():
    """A TrajectoryGoalDTO with one valid path can be constructed."""
    path = _make_path()
    dto = TrajectoryGoalDTO(paths=[path])
    assert len(dto.paths) == 1


def test_trajectory_goal_dto_duplicate_path_ids():
    """TrajectoryGoalDTO raises ValidationError when two paths share the same path_id."""
    path1 = _make_path(path_id=_UUID_A)
    path2 = _make_path(path_id=_UUID_A)
    with pytest.raises(ValidationError, match='duplicate path_id'):
        TrajectoryGoalDTO(paths=[path1, path2])

def test_from_ros_msg_circ_empty_circ_type_raises():
    """Empty circ_type raises ValueError with path_id in the message."""
    msg = _make_ros_path_msg(motion_type='CIRC', circ_type='')
    with pytest.raises(ValueError, match='circ_type is empty'):
        TrajectoryPathDTO.from_ros_msg(msg)


def test_from_ros_msg_circ_invalid_circ_type_raises():
    """CIRC path with unrecognised circ_type raises ValueError."""
    msg = _make_ros_path_msg(motion_type='CIRC', circ_type='unknown')
    with pytest.raises(ValueError, match='invalid circ_type'):
        TrajectoryPathDTO.from_ros_msg(msg)


def test_from_ros_msg_circ_valid_interim():
    """CIRC path with circ_type='interim' converts successfully."""
    msg = _make_ros_path_msg(motion_type='CIRC', circ_type='interim')
    dto = TrajectoryPathDTO.from_ros_msg(msg)
    assert dto.circ_type == CircTypeEnum.INTERIM


def test_from_ros_msg_circ_valid_center():
    """CIRC path with circ_type='center' converts successfully."""
    msg = _make_ros_path_msg(motion_type='CIRC', circ_type='center')
    dto = TrajectoryPathDTO.from_ros_msg(msg)
    assert dto.circ_type == CircTypeEnum.CENTER


def test_from_ros_msg_non_circ_empty_circ_type_defaults_to_interim():
    """LIN path with circ_type='' still defaults to CircTypeEnum.INTERIM (non-CIRC unaffected)."""
    msg = _make_ros_path_msg(motion_type='LIN', circ_type='')
    dto = TrajectoryPathDTO.from_ros_msg(msg)
    assert dto.circ_type == CircTypeEnum.INTERIM
