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
"""Steps 7, 10 and 12: selecting the optimizer and optimizing the patterns."""

from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node

from execution_optimizer.msg import PatternGroup
from execution_optimizer.srv import DispensingOptimizePatterns
from manufacturing_executor.enums.lifecycle_state_enum import LifecycleStateEnum
from manufacturing_executor.exceptions import SystemFailureError
from manufacturing_executor.models.manufacturing_executor_config_dto import (
    ManufacturingExecutorConfigDTO,
)
from manufacturing_executor.services.abort_context import AbortContext
from manufacturing_executor.services.async_calls import ServiceCall
from manufacturing_executor.services.lifecycle_client import LifecycleClientRegistry
from manufacturing_executor.services.loop_clients import LoopClients
from manufacturing_executor.utils.log_format import format_pose
from manufacturing_executor.utils.node_names import strip_slash
from repositories.msg import OperationRepositoryDispensingPattern


class OptimizationCall:
    """A running ``optimize_patterns`` call."""

    def __init__(
        self, call: ServiceCall, timeout_s: float, ctx: AbortContext, logger: RcutilsLogger
    ) -> None:
        self._call = call
        self._timeout_s = timeout_s
        self._ctx = ctx
        self._logger = logger

    def wait(self) -> list[PatternGroup]:
        """Wait for the optimization.

        :returns: The pattern groups.
        :raises SystemFailureError: On failure or timeout.
        """
        response = self._call.wait(self._timeout_s, self._ctx)
        if not response.success:
            raise SystemFailureError(f'Pattern optimization failed: {response.error_message}')
        groups = list(response.groups)
        self._logger.info(
            f'Optimization returned {len(groups)} group(s) in {response.optimization_time_s:.3f} s'
        )
        for number, group in enumerate(groups):
            self._logger.info(
                f'  group {number}: patterns={list(group.pattern_ids)} '
                f'capture_poses={[format_pose(pose) for pose in group.image_capture_poses]}'
            )
        return groups


class OptimizerService:
    """Keeps exactly one optimizer active and runs the optimization."""

    def __init__(
        self,
        node: Node,
        config: ManufacturingExecutorConfigDTO,
        clients: LoopClients,
        registry: LifecycleClientRegistry,
    ) -> None:
        self._logger = node.get_logger()
        self._config = config
        self._clients = clients
        self._registry = registry

    def select(self, optimizer_node: str, ctx: AbortContext) -> None:
        """Deactivate every other active optimizer and activate ``optimizer_node`` (it stays active)."""
        selected = strip_slash(optimizer_node)
        for name in self._config.execution_optimizer_nodes:
            if strip_slash(name) == selected:
                continue
            client = self._registry.get(name)
            if client.get_state(ctx) == LifecycleStateEnum.ACTIVE:
                self._logger.info(f'Deactivating optimizer {client.node_name}')
                client.deactivate(ctx)
        client = self._registry.get(selected)
        state = client.get_state(ctx)
        self._logger.info(f'Optimizer {client.node_name} is {state.value}')
        if state == LifecycleStateEnum.UNCONFIGURED:
            self._logger.info(f'Configuring optimizer {client.node_name}')
            client.configure(ctx)
            state = LifecycleStateEnum.INACTIVE
        if state == LifecycleStateEnum.INACTIVE:
            self._logger.info(f'Activating optimizer {client.node_name}')
            client.activate(ctx)
        elif state != LifecycleStateEnum.ACTIVE:
            raise SystemFailureError(f'Optimizer {client.node_name} is {state.value}')

    def start(
        self,
        optimizer_node: str,
        patterns: list[OperationRepositoryDispensingPattern],
        ctx: AbortContext,
    ) -> OptimizationCall:
        """Send the optimization request without waiting for the response."""
        request = DispensingOptimizePatterns.Request()
        request.patterns = patterns
        call = ServiceCall.start(
            self._clients.optimize_patterns(strip_slash(optimizer_node)),
            request,
            availability_timeout_s=self._config.service_call_timeout_s,
            ctx=ctx,
            description='optimize_patterns',
        )
        return OptimizationCall(call, self._config.optimization_timeout_s, ctx, self._logger)
