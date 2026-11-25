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
"""Tests of the optional localization between operations of the same product."""

from unittest.mock import MagicMock, patch

import pytest

from manufacturing_executor.services import manufacturing_loop
from manufacturing_executor.services.abort_context import AbortContext
from manufacturing_executor.services.manufacturing_loop import ManufacturingLoop
from repositories.models.operation_repository.argument_dto import ArgumentDTO


def _loop(*, localize_per_operation: bool, frame: str | None) -> ManufacturingLoop:
    config = MagicMock(localize_per_operation=localize_per_operation)
    loop = ManufacturingLoop(
        MagicMock(), config, MagicMock(), MagicMock(), MagicMock(), MagicMock(),
        MagicMock(), MagicMock(), MagicMock(), MagicMock(),
    )
    loop._localized_frame = frame
    return loop


def _operation(**args: object) -> MagicMock:
    return MagicMock(operation_args=[ArgumentDTO.from_named_value(k, v) for k, v in args.items()] or None)


@pytest.mark.parametrize(
    ('per_operation', 'frame', 'args', 'expected'),
    [
        (False, None, {}, True),  # first operation of a product
        (False, None, {'skip_localization': True}, True),  # no frame to reuse
        (False, 'product', {}, False),
        (True, 'product', {}, True),
        (True, 'product', {'skip_localization': True}, False),
        (False, 'product', {'force_localization': True}, True),
        (False, 'product', {'skip_localization': True, 'force_localization': True}, True),
        (True, 'product', {'skip_localization': False}, True),  # false has no effect
        (False, 'product', {'force_localization': False}, False),
        (False, 'product', {'force_localization': 'yes'}, False),  # wrong type is ignored
        (True, 'product', {'skip_localization': 1}, True),
    ],
)
def test_should_localize(per_operation: bool, frame: str | None, args: dict, expected: bool) -> None:
    loop = _loop(localize_per_operation=per_operation, frame=frame)
    assert loop._should_localize(_operation(**args)) is expected


def test_skipped_localization_reuses_the_last_frame() -> None:
    loop = _loop(localize_per_operation=False, frame='product')
    loop._optimizer.start.return_value = MagicMock(done=True, wait=MagicMock(return_value=[]))
    operation = _operation()
    phases: list[str] = []
    with patch.object(manufacturing_loop.LocalizationRun, 'start') as start:
        loop._run_operation(
            MagicMock(), MagicMock(), operation, MagicMock(), AbortContext(), phases.append,
        )
    start.assert_not_called()
    operation.to_msg.assert_called_once_with('product')
    assert 'Localizing product' not in phases


def test_injection_is_not_reported_while_the_product_stays_in_the_station() -> None:
    loop = _loop(localize_per_operation=False, frame=None)
    results = iter([False, True])

    def manufacture(*_args: object) -> bool:
        try:
            return next(results)
        except StopIteration:
            raise RuntimeError('stop') from None

    phases: list[str] = []
    with patch.object(loop, '_manufacture', side_effect=manufacture), pytest.raises(RuntimeError):
        loop.run(AbortContext(), phases.append)
    assert phases == [
        'Waiting for product injection',  # first operation, product is injected
        'Waiting for product ejection',  # second operation was the last, no injection reported in between
        'Waiting for product injection',  # next product
    ]
