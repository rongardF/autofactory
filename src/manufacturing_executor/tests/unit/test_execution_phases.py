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
"""Tests of the ``execution_phase`` feedback of the manufacturing loop."""

from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest
from geometry_msgs.msg import Pose

from execution_optimizer.msg import PatternGroup
from manufacturing_executor.services import execution_service, manufacturing_loop
from manufacturing_executor.services.abort_context import AbortContext
from manufacturing_executor.services.execution_service import ExecutionService
from manufacturing_executor.services.manufacturing_loop import ManufacturingLoop


def _run_operation(*, mounted: bool, localized: bool, optimized: bool) -> list[str]:
    phases: list[str] = []
    executors = MagicMock()
    executors.is_mounted.return_value = mounted
    optimizer = MagicMock()
    optimizer.start.return_value = MagicMock(done=optimized, wait=MagicMock(return_value=[]))
    localization = MagicMock(result_ready=localized)
    loop = ManufacturingLoop(
        MagicMock(), MagicMock(), MagicMock(), MagicMock(), MagicMock(), MagicMock(),
        optimizer, executors, MagicMock(), MagicMock(),
    )
    with patch.object(manufacturing_loop.LocalizationRun, 'start', return_value=localization):
        loop._run_operation(
            MagicMock(), MagicMock(operation_id='op-1'), MagicMock(), MagicMock(),
            AbortContext(), phases.append,
        )
    return phases


def test_all_optional_phases_are_reported_when_they_happen() -> None:
    assert _run_operation(mounted=False, localized=False, optimized=False) == [
        'Localizing product',
        'Mounting the endtool',
        'Waiting localization calculations to finish',
        'Initializing endtool',
        'Performing execution optimization',
    ]


def test_skipped_phases_are_not_reported() -> None:
    assert _run_operation(mounted=True, localized=True, optimized=True) == [
        'Localizing product',
        'Initializing endtool',
    ]


@pytest.fixture
def execution() -> ExecutionService:
    capture = MagicMock()
    with (
        patch.object(execution_service, 'MovementSessionClient'),
        patch.object(execution_service, 'run_action'),
    ):
        yield ExecutionService(MagicMock(), MagicMock(), MagicMock(), capture)


def test_group_phases_skip_captures_of_groups_without_capture_poses(execution: ExecutionService) -> None:
    phases: list[str] = []
    groups = [
        PatternGroup(pattern_ids=['p1'], image_capture_poses=[Pose()]),
        PatternGroup(pattern_ids=['p2'], image_capture_poses=[]),
    ]
    execution.execute(
        groups,
        {'p1': SimpleNamespace(), 'p2': SimpleNamespace()},
        executor_node='executor',
        operation_number='10',
        operation_id='op-1',
        product=MagicMock(),
        ctx=AbortContext(),
        report_phase=phases.append,
    )
    assert phases == [
        "Capturing operation '10' pattern group 0 before image",
        "Executing operation '10' pattern group 0 patterns",
        "Capturing operation '10' pattern group 0 after image",
        "Executing operation '10' pattern group 1 patterns",
    ]
