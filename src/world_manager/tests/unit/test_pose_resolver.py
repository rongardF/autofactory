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
"""Unit tests for PoseResolver, quaternion helpers and Deadline."""

from math import isclose, sqrt
from time import sleep

from pytest import raises

from world_manager.exceptions import ModelValidationError, OperationTimeoutError
from world_manager.models import PoseDTO
from world_manager.services.pose_resolver import PoseResolver
from world_manager.utils import Deadline, compose, normalize

QUARTER_TURN_Z = (0.0, 0.0, sqrt(0.5), sqrt(0.5))


def test_empty_frame_is_treated_as_world():
    """An empty frame id means the pose is already in the world frame."""
    resolver = PoseResolver(lambda *_: 1 / 0, 'world', 1.0)
    result = resolver.to_world(PoseDTO(x=1.0))
    assert (result.frame_id, result.x) == ('world', 1.0)


def test_pose_in_other_frame_is_transformed():
    """A frame rotated 90 degrees about Z and offset by (1, 0, 0) maps child (1, 0, 0) to (1, 1, 0)."""
    frame = PoseDTO(frame_id='world', x=1.0, qx=QUARTER_TURN_Z[0], qy=QUARTER_TURN_Z[1],
                    qz=QUARTER_TURN_Z[2], qw=QUARTER_TURN_Z[3])
    resolver = PoseResolver(lambda target, source, timeout: frame, 'world', 1.0)
    result = resolver.to_world(PoseDTO(frame_id='station', x=1.0))
    assert isclose(result.x, 1.0, abs_tol=1e-9)
    assert isclose(result.y, 1.0, abs_tol=1e-9)
    assert result.frame_id == 'world'


def test_tf_failure_becomes_validation_error():
    """A failing TF lookup is reported as a rejected request naming the frame."""
    def lookup(*_):
        raise RuntimeError('no transform')

    with raises(ModelValidationError, match='station'):
        PoseResolver(lookup, 'world', 1.0).to_world(PoseDTO(frame_id='station'))


def test_compose_rotations_multiply():
    """Two quarter turns about Z compose to a half turn."""
    quarter = PoseDTO(qz=QUARTER_TURN_Z[2], qw=QUARTER_TURN_Z[3])
    result = compose(quarter, quarter, 'world')
    assert isclose(abs(result.qz), 1.0, abs_tol=1e-9)
    assert isclose(result.qw, 0.0, abs_tol=1e-9)


def test_zero_quaternion_cannot_be_normalized():
    """A zero quaternion is invalid."""
    with raises(ValueError):
        normalize((0.0, 0.0, 0.0, 0.0))


def test_deadline_expires_and_names_the_step():
    """Once expired, check raises naming the step that was about to run."""
    deadline = Deadline(0.01)
    sleep(0.03)
    with raises(OperationTimeoutError, match='creating x'):
        deadline.check('creating x')
    assert deadline.remaining() == 0.0
