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

from collections.abc import Callable

from rclpy.node import Node

from manufacturing_executor.exceptions import OperationFailureError
from manufacturing_executor.models.manufacturing_executor_config_dto import (
    ManufacturingExecutorConfigDTO,
)
from manufacturing_executor.services.abort_context import AbortContext
from manufacturing_executor.services.execution_service import ExecutionService
from manufacturing_executor.services.executor_service import ExecutorService
from manufacturing_executor.services.idle_pose_service import IdlePoseService
from manufacturing_executor.services.localization_run import LocalizationRun
from manufacturing_executor.services.loop_clients import LoopClients
from manufacturing_executor.services.optimizer_service import OptimizerService
from manufacturing_executor.services.production_service import ProductionService
from manufacturing_executor.services.routing_service import RoutingService
from manufacturing_executor.services.tool_selection_service import ToolSelectionService
from manufacturing_executor.utils.localization_args import localization_override
from production_manager.model.production_data_dto import ProductionDataDTO
from repositories.enums.operation_repository import OperationTypeEnum
from repositories.models.operation_repository.operation_dto import OperationDTO
from station_workplace.models.product_data_dto import ProductDataDTO
from tools_manager.model.tool_info_dto import ToolInfoDto


class ManufacturingLoop:
    """Routes, localizes, mounts, executes and reports; the only class that knows theder."""

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
        idle_pose: IdlePoseService,
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
        self._idle_pose = idle_pose
        # Frame of the last successful localization of the product in the station.
        self._localized_frame: str | None = None

    def run(self, ctx: AbortContext, report_phase: Callable[[str], None]) -> None:
        """Run until the context is aborted or a system failure is raised.

        :param report_phase: Called with the description of each phase before it starts;
            phases that are skipped (e.g. no tool change needed) are not reported.
        """
        self._logger.info('Manufacturing loop started')
        self._localized_frame = None
        eject = False
        in_station = False
        while True:
            ctx.raise_if_aborted()
            if eject:
                # The move runs while the product is ejected; its failure aborts ctx and with it the wait.
                move = self._idle_pose.start_move(ctx)
                try:
                    self._logger.info(' Waiting for the product to be ejected')
                    report_phase('Waiting for product ejection')
                    self._routing.eject(ctx)
                    move.wait()
                finally:
                    move.close()
                eject = False
                continue
            if in_station:
                # The product is already there: routing returns at once, so there is no injection to report.
                self._logger.info(' Product is still in the station, fetching its next operation')
            else:
                self._logger.info(' Waiting for a product to be injected')
                report_phase('Waiting for product injection')
            product = self._routing.inject(ctx)
            eject = self._manufacture(product, ctx, report_phase)
            in_station = not eject
            if eject:
                self._localized_frame = None
            self._logger.info(
                f'Product {product.serial_number} '
                f'{"finished in this station, ejecting" if eject else "stays in the station for the next operation"}'
            )

    def _manufacture(
        self, product: ProductDataDTO, ctx: AbortContext, report_phase: Callable[[str], None]
    ) -> bool:
        """Manufacture one operation.

        :returns: ``finished_in_station``: whether the product has to be ejected.
        """
        self._logger.info(
            f' Requesting production data for product {product.product_number} '
            f'rev {product.product_revision} sn {product.serial_number}'
        )
        production = self._production.get_production_data(product, ctx)
        self._logger.info(
            f' Production data: assembly_id={production.assembly_id} '
            f'operation_id={production.operation_id} assembly_model_id={production.assembly_model_id}'
        )
        try:
            self._logger.info(f' Fetching operation {production.operation_id}')
            operation = self._production.get_operation(production, ctx)
            self._logger.info(
                f' Operation {production.operation_id}: type={operation.operation_type.value} '
                f'end_tool={operation.end_tool} material={operation.material}'
            )
            if operation.operation_type is not OperationTypeEnum.DISPENSING:
                raise OperationFailureError(
                    f'Unsupported operation type {operation.operation_type.value}'
                )
            self._logger.info(' Looking up a tool for the operation')
            tool = self._tools.find_tool(operation, ctx)
            self._logger.info(f' Selected tool {tool.tool_sn}')
        except OperationFailureError as failure:
            self._logger.error(f'Operation {production.operation_id} failed: {failure}')
            self._logger.info(f'Reporting operation {production.operation_id} as failed')
            return self._production.report(production, False, str(failure), ctx)
        self._run_operation(product, production, operation, tool, ctx, report_phase)
        self._logger.info(f'Reporting operation {production.operation_id} as successful')
        return self._production.report(production, True, '', ctx)

    def _should_localize(self, operation: OperationDTO) -> bool:
        """Decide whether the operation localizes the product or reuses the last frame."""
        override = localization_override(operation.operation_args, self._logger.warning)
        if self._localized_frame is None:
            if override is False:
                self._logger.warning('Localization skip requested but no localized frame exists, localizing')
            return True
        if override is not None:
            return override
        return self._config.localize_per_operation

    def _run_operation(
        self,
        product: ProductDataDTO,
        production: ProductionDataDTO,
        operation: OperationDTO,
        tool: ToolInfoDto,
        ctx: AbortContext,
        report_phase: Callable[[str], None],
    ) -> None:
        cleanup = AbortContext.detached()
        localization: LocalizationRun | None = None
        if self._should_localize(operation):
            self._logger.info(
                f' Starting localization for assembly model {production.assembly_model_id}'
            )
            report_phase('Localizing product')
            localization = LocalizationRun.start(
                self._clients,
                production.assembly_model_id,
                ctx=ctx,
                accept_timeout_s=self._config.service_call_timeout_s,
                result_timeout_s=self._config.localization_result_timeout_s,
            )
        else:
            self._logger.info(f' Skipping localization, reusing frame {self._localized_frame!r}')
        executor_node = ''
        try:
            self._logger.info(f' Resolving the nodes of tool {tool.tool_sn}')
            nodes = self._tools.resolve_nodes(tool, ctx)
            self._logger.info(
                f' Tool {tool.tool_sn}: endtool={nodes.endtool_node} '
                f'pattern_executor={nodes.pattern_executor_node} optimizer={nodes.optimizer_node}'
            )
            executor_node = nodes.pattern_executor_node
            self._logger.info(f' Selecting optimizer {nodes.optimizer_node}')
            self._optimizer.select(nodes.optimizer_node, ctx)
            self._logger.info(' Deactivating the active pattern executors')
            self._executors.deactivate_active(ctx)
            if localization is not None:
                # No robot motion before the localizer released the movement session.
                self._logger.info(' Waiting for the localization image collection to finish')
                localization.wait_collection_done(ctx)
                self._logger.info(' Localization image collection done, robot is free')
            if not self._executors.is_mounted(tool):
                self._logger.info(f' Mounting tool {tool.tool_sn}')
                report_phase('Mounting the endtool')
            self._executors.mount(tool, ctx)
            if localization is not None:
                self._logger.info(' Waiting for the localization result')
                if not localization.result_ready:
                    report_phase('Waiting localization calculations to finish')
                self._localized_frame = localization.wait_result()
                self._logger.info(f' Product localized in frame {self._localized_frame!r}')
            frame = self._localized_frame
            patterns = operation.to_msg(frame).dispensing_patterns
            self._logger.info(
                f'Requesting optimization of {len(patterns)} pattern(s) from '
                f'{nodes.optimizer_node}: {[pattern.id for pattern in patterns]}'
            )
            optimization = self._optimizer.start(nodes.optimizer_node, patterns, ctx)
            self._logger.info(f'Activating pattern executor {executor_node}')
            report_phase('Initializing endtool')
            self._executors.activate(executor_node, ctx)
            self._logger.info('Waiting for the optimization result')
            if not optimization.done:
                report_phase('Performing execution optimization')
            groups = optimization.wait()
            self._logger.info(f'Executing {len(groups)} pattern group(s) with {executor_node}')
            self._execution.execute(
                groups,
                {pattern.id: pattern for pattern in patterns},
                executor_node=executor_node,
                operation_number=operation.operation_number,
                operation_id=production.operation_id,
                product=product,
                ctx=ctx,
                report_phase=report_phase,
            )
        finally:
            if localization is not None:
                localization.cancel()
            if executor_node:
                try:
                    self._executors.deactivate(executor_node, cleanup)
                except Exception as error:  # noqa: BLE001 - cleanup must not hide the original error
                    self._logger.error(f'Could not deactivate {executor_node}: {error}')
