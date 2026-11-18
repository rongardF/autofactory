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
"""The manufacturing loop."""

from rclpy.node import Node

from manufacturing_executor.exceptions import OperationFailureError
from manufacturing_executor.models.manufacturing_executor_config_dto import (
    ManufacturingExecutorConfigDTO,
)
from manufacturing_executor.services.abort_context import AbortContext
from manufacturing_executor.services.execution_service import ExecutionService
from manufacturing_executor.services.executor_service import ExecutorService
from manufacturing_executor.services.localization_run import LocalizationRun
from manufacturing_executor.services.loop_clients import LoopClients
from manufacturing_executor.services.optimizer_service import OptimizerService
from manufacturing_executor.services.production_service import ProductionService
from manufacturing_executor.services.routing_service import RoutingService
from manufacturing_executor.services.tool_selection_service import ToolSelectionService
from production_manager.model.production_data_dto import ProductionDataDTO
from repositories.enums.operation_repository import OperationTypeEnum
from repositories.models.operation_repository.operation_dto import OperationDTO
from station_workplace.models.product_data_dto import ProductDataDTO
from tools_manager.model.tool_info_dto import ToolInfoDto


class ManufacturingLoop:
    """Routes, localizes, mounts, executes and reports; the only class that knows the step order."""

    def __init__(
        self,
        node: Node,
        config: ManufacturingExecutorConfigDTO,
        clients: LoopClients,
        routing: RoutingService,
        production: ProductionService,
        tools: ToolSelectionService,
        optimizer: OptimizerService,
        executors: ExecutorService,
        execution: ExecutionService,
    ) -> None:
        self._node = node
        self._logger = node.get_logger()
        self._config = config
        self._clients = clients
        self._routing = routing
        self._production = production
        self._tools = tools
        self._optimizer = optimizer
        self._executors = executors
        self._execution = execution

    def run(self, ctx: AbortContext) -> None:
        """Run until the context is aborted or a system failure is raised."""
        eject = False
        while True:
            ctx.raise_if_aborted()
            if eject:
                self._routing.eject(ctx)
                eject = False
                continue
            product = self._routing.inject(ctx)
            eject = self._manufacture(product, ctx)

    def _manufacture(self, product: ProductDataDTO, ctx: AbortContext) -> bool:
        """Manufacture one operation.

        :returns: ``finished_in_station``: whether the product has to be ejected.
        """
        production = self._production.get_production_data(product, ctx)
        try:
            operation = self._production.get_operation(production, ctx)
            if operation.operation_type is not OperationTypeEnum.DISPENSING:
                raise OperationFailureError(
                    f'Unsupported operation type {operation.operation_type.value}'
                )
            tool = self._tools.find_tool(operation, ctx)
        except OperationFailureError as failure:
            self._logger.error(f'Operation {production.operation_id} failed: {failure}')
            return self._production.report(production, False, str(failure), ctx)
        self._run_operation(product, production, operation, tool, ctx)
        return self._production.report(production, True, '', ctx)

    def _run_operation(
        self,
        product: ProductDataDTO,
        production: ProductionDataDTO,
        operation: OperationDTO,
        tool: ToolInfoDto,
        ctx: AbortContext,
    ) -> None:
        cleanup = AbortContext.detached()
        localization = LocalizationRun.start(
            self._clients,
            production.assembly_model_id,
            ctx=ctx,
            accept_timeout_s=self._config.service_call_timeout_s,
            result_timeout_s=self._config.localization_result_timeout_s,
        )
        executor_node = ''
        try:
            nodes = self._tools.resolve_nodes(tool, ctx)
            executor_node = nodes.pattern_executor_node
            self._optimizer.select(nodes.optimizer_node, ctx)
            self._executors.deactivate_active(ctx)
            # No robot motion before the localizer released the movement session.
            localization.wait_collection_done(ctx)
            self._executors.mount(tool, ctx)
            frame = localization.wait_result()
            patterns = operation.to_msg(frame).dispensing_patterns
            optimization = self._optimizer.start(nodes.optimizer_node, patterns, ctx)
            self._executors.activate(executor_node, ctx)
            groups = optimization.wait()
            self._execution.execute(
                groups,
                {pattern.id: pattern for pattern in patterns},
                executor_node=executor_node,
                operation_id=production.operation_id,
                product=product,
                ctx=ctx,
            )
        finally:
            localization.cancel()
            if executor_node:
                try:
                    self._executors.deactivate(executor_node, cleanup)
                except Exception as error:  # noqa: BLE001 - cleanup must not hide the original error
                    self._logger.error(f'Could not deactivate {executor_node}: {error}')
