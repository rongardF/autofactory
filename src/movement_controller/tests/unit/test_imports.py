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
"""Smoke test: verify all generated interfaces and Python sub-packages are importable."""


def test_action_execute_trajectory_importable():
    """Verify ExecuteTrajectory action type is importable after colcon build."""
    from movement_controller.action import ExecuteTrajectory  # noqa: F401
    assert hasattr(ExecuteTrajectory, 'Goal')
    assert hasattr(ExecuteTrajectory, 'Result')
    assert hasattr(ExecuteTrajectory, 'Feedback')


def test_msg_trajectory_path_importable():
    """Verify TrajectoryPath message type is importable."""
    from movement_controller.msg import TrajectoryPath  # noqa: F401


def test_python_subpackages_importable():
    """Verify Python sub-package stubs are importable."""
    import movement_controller  # noqa: F401
    from movement_controller import models  # noqa: F401
    from movement_controller import enums  # noqa: F401
    from movement_controller import utils  # noqa: F401
    from movement_controller import services  # noqa: F401
    from movement_controller import exceptions  # noqa: F401
