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
"""Step 13: image capture and pattern execution of all pattern groups."""

from rclpy.node import Node

from execution_optimizer.msg import PatternGroup
from manufacturing_executor.exceptions import SystemFailureError
from manufacturing_executor.models.manufacturing_executor_config_dto import (
    ManufacturingExecutorConfigDTO,
)
from manufacturing_executor.services.abort_context import AbortContext
from manufacturing_executor.services.async_calls import run_action
from manufacturing_executor.services.image_capture_service import ImageCaptureService
from manufacturing_executor.services.loop_clients import LoopClients
from manufacturing_executor.services.movement_session_client import MovementSessionClient
from pattern_executors.action import ExecuteDispensingPattern
from repositories.msg import OperationRepositoryDispensingPattern
from station_workplace.models.product_data_dto import ProductDataDTO


class ExecutionService:
    """Executes the pattern groups of an operation in one movement session."""

    def __init__(
        self,
        node: Node,
        config: ManufacturingExecutorConfigDTO,
        clients: LoopClients,
        image_capture: ImageCaptureService,
    ) -> None:
        self._node = node
        self._logger = node.get_logger()
        self._config = config
        self._clients = clients
        self._image_capture = image_capture

    def execute(
        self,
        groups: list[PatternGroup],
        patterns_by_id: dict[str, OperationRepositoryDispensingPattern],
        *,
        executor_node: str,
        operation_id: str,
        product: ProductDataDTO,
        ctx: AbortContext,
    ) -> None:
        """For every group: images, patterns, images.

        The movement session is closed when this returns or raises.

        :raises SystemFailureError: If a pattern is unknown or any step fails.
        """
        session = MovementSessionClient(self._node, self._config, self._clients, ctx)
        try:
            session.ensure_valid()
            self._logger.info(f'Movement session {session.session_id} opened')
            for number, group in enumerate(groups, start=1):
                self._logger.info(
                    f'Group {number}/{len(groups)}: {len(group.image_capture_poses)} capture pose(s), '
                    f'patterns={list(group.pattern_ids)}'
                )
                self._logger.info(f'Group {number}/{len(groups)}: capturing images before dispensing')
                self._capture_all(group, operation_id, product, session, ctx)
                self._logger.info(f'Group {number}/{len(groups)}: executing patterns')
                self._run_patterns(group, patterns_by_id, executor_node, operation_id, session, ctx)
                self._logger.info(f'Group {number}/{len(groups)}: capturing images after dispensing')
                self._capture_all(group, operation_id, product, session, ctx)
                self._logger.info(f'Group {number}/{len(groups)}: done')
        finally:
            self._logger.info(f'Closing movement session {session.session_id}')
            session.close()

    def _capture_all(
        self,
        group: PatternGroup,
        operation_id: str,
        product: ProductDataDTO,
        session: MovementSessionClient,
        ctx: AbortContext,
    ) -> None:
        for pose in group.image_capture_poses:
            self._image_capture.capture(
                pose, operation_id=operation_id, product=product, session=session, ctx=ctx
            )

    def _run_patterns(
        self,
        group: PatternGroup,
        patterns_by_id: dict[str, OperationRepositoryDispensingPattern],
        executor_node: str,
        operation_id: str,
        session: MovementSessionClient,
        ctx: AbortContext,
    ) -> None:
        missing = [pattern_id for pattern_id in group.pattern_ids if pattern_id not in patterns_by_id]
        if missing:
            raise SystemFailureError(f'The optimizer returned unknown pattern ids: {missing}')
        session.ensure_valid()
        goal = ExecuteDispensingPattern.Goal()
        goal.patterns = [patterns_by_id[pattern_id] for pattern_id in group.pattern_ids]
        goal.operation_id = operation_id
        goal.session_id = session.session_id
        outcome = run_action(
            self._clients.execute_dispensing_pattern(executor_node),
            goal,
            ctx=ctx,
            description='Pattern execution',
            accept_timeout_s=self._config.service_call_timeout_s,
            result_timeout_s=None,
        )
        outcome.ensure_success('Pattern execution')
        self._logger.info(f'{executor_node} executed {len(goal.patterns)} pattern(s)')
