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
"""Unit tests for the indexed operation queue of ``LocalManager``."""

import json
from uuid import uuid4

import pytest

from production_manager.services.local_manager import LocalManager
from repositories.enums.operation_repository import SCHEMA_VERSION

STATION = str(uuid4())
ASSEMBLY = str(uuid4())
OPERATIONS = [str(uuid4()), str(uuid4())]


class _Logger:
    def info(self, message: str) -> None:
        pass


@pytest.fixture
def manager(tmp_path) -> LocalManager:
    path = tmp_path / 'assembly.json'
    path.write_text(
        json.dumps(
            {
                'schema_version': SCHEMA_VERSION,
                'id': ASSEMBLY,
                'product_number': 'P1',
                'product_revision': 'A',
                'operations': [
                    {'id': operation, 'start_assembly_model_id': str(uuid4())}
                    for operation in OPERATIONS
                ],
            }
        )
    )
    return LocalManager(str(path), _Logger())


def _get(manager: LocalManager, serial: str = 'S1'):
    return manager.get_production_data('P1', 'A', serial, STATION)


def test_lowest_index_pending_operation_is_returned(manager):
    assert manager.set_operation(0, OPERATIONS[0], 'S1', STATION).success
    assert manager.set_operation(1, OPERATIONS[1], 'S1', STATION).success
    assert _get(manager).operation_id == OPERATIONS[0]


def test_gap_is_rejected(manager):
    assert not manager.set_operation(1, OPERATIONS[0], 'S1', STATION).success


def test_multi_operation_continuation(manager):
    manager.set_operation(0, OPERATIONS[0], 'S1', STATION)
    manager.set_operation(1, OPERATIONS[1], 'S1', STATION)

    first = manager.report_production(ASSEMBLY, OPERATIONS[0], True, STATION)
    assert first.success and not first.finished_in_station
    assert _get(manager).operation_id == OPERATIONS[1]

    second = manager.report_production(ASSEMBLY, OPERATIONS[1], True, STATION)
    assert second.success and second.finished_in_station
    assert not _get(manager).success


def test_failure_report_finishes_and_clears_queue(manager):
    manager.set_operation(0, OPERATIONS[0], 'S1', STATION)
    manager.set_operation(1, OPERATIONS[1], 'S1', STATION)
    result = manager.report_production(ASSEMBLY, OPERATIONS[0], False, STATION, 'bad')
    assert result.success and result.finished_in_station
    assert not _get(manager).success


def test_overwrite_resets_done_flag(manager):
    manager.set_operation(0, OPERATIONS[0], 'S1', STATION)
    manager.set_operation(1, OPERATIONS[1], 'S1', STATION)
    manager.report_production(ASSEMBLY, OPERATIONS[0], True, STATION)
    manager.set_operation(0, OPERATIONS[0], 'S1', STATION)
    assert _get(manager).operation_id == OPERATIONS[0]


def test_new_serial_requires_index_zero(manager):
    manager.set_operation(0, OPERATIONS[0], 'S1', STATION)
    assert not manager.set_operation(1, OPERATIONS[1], 'S2', STATION).success
    assert manager.set_operation(0, OPERATIONS[1], 'S2', STATION).success
    assert _get(manager, 'S2').operation_id == OPERATIONS[1]
    assert not _get(manager, 'S1').success


def test_report_unknown_operation_fails(manager):
    manager.set_operation(0, OPERATIONS[0], 'S1', STATION)
    assert not manager.report_production(ASSEMBLY, OPERATIONS[1], True, STATION).success
