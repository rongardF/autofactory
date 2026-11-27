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
"""Steps 8 and 11: pattern executor activation and tool mounting."""

from rclpy.node import Node
from station_utils.enums import LifecycleStateEnum

from manufacturing_executor.exceptions import SystemFailureError
from manufacturing_executor.models.manufacturing_executor_config_dto import (
    ManufacturingExecutorConfigDTO,
)
from manufacturing_executor.services.abort_context import AbortContext
from manufacturing_executor.services.async_calls import run_action
from manufacturing_executor.services.node_manager_registry import NodeManagerRegistry
from manufacturing_executor.services.loop_clients import LoopClients
from manufacturing_executor.services.tool_inventory import ToolInventory
from tools_manager.action import MountTool
from tools_manager.model.tool_info_dto import ToolInfoDto


class ExecutorService:
    """Drives the pattern executors (and through them the endtools) and mounts tools."""

    def __init__(
        self,
        node: Node,
        config: ManufacturingExecutorConfigDTO,
        clients: LoopClients,
        registry: NodeManagerRegistry,
        inventory: ToolInventory,
    ) -> None:
        self._logger = node.get_logger()
        self._config = config
        self._clients = clients
        self._registry = registry
        self._inventory = inventory

    def deactivate_active(self, ctx: AbortContext) -> None:
        """Deactivate every active pattern executor."""
        for name in self._config.pattern_executor_nodes:
            self.deactivate(name, ctx)

    def deactivate(self, executor_node: str, ctx: AbortContext) -> None:
        """Deactivate the executor if it is active (this deactivates its endtool as well)."""
        client = self._registry.get(executor_node)
        if client.get_state(ctx) == LifecycleStateEnum.ACTIVE:
            self._logger.info(f'Deactivating pattern executor {client.node_name}')
            client.deactivate(ctx)

    def activate(self, executor_node: str, ctx: AbortContext) -> None:
        """Activate the executor (this activates its endtool as well)."""
        client = self._registry.get(executor_node)
        state = client.get_state(ctx)
        self._logger.info(f'Pattern executor {client.node_name} is {state.value}')
        if state == LifecycleStateEnum.UNCONFIGURED:
            self._logger.info(f'Configuring pattern executor {client.node_name}')
            client.configure(ctx)
            state = LifecycleStateEnum.INACTIVE
        if state == LifecycleStateEnum.INACTIVE:
            self._logger.info(f'Activating pattern executor {client.node_name}')
            client.activate(ctx)
        elif state != LifecycleStateEnum.ACTIVE:
            raise SystemFailureError(f'Pattern executor {client.node_name} is {state.value}')

    def is_mounted(self, tool: ToolInfoDto) -> bool:
        """Whether ``tool`` is the currently mounted tool."""
        return self._inventory.mounted_tool_sn() == tool.tool_sn

    def mount(self, tool: ToolInfoDto, ctx: AbortContext) -> None:
        """Mount the tool unless it already is the mounted one."""
        mounted = self._inventory.mounted_tool_sn()
        if mounted == tool.tool_sn:
            self._logger.info(f'Tool {tool.tool_sn} is already mounted')
            return
        self._logger.info(f'Mounting tool {tool.tool_sn} (currently mounted: {mounted or "none"})')
        goal = MountTool.Goal()
        goal.tool_sn = tool.tool_sn
        outcome = run_action(
            self._clients.mount_tool,
            goal,
            ctx=ctx,
            description=f'Mounting tool {tool.tool_sn}',
            accept_timeout_s=self._config.service_call_timeout_s,
            result_timeout_s=self._config.mount_tool_timeout_s,
        )
        outcome.ensure_success(f'Mounting tool {tool.tool_sn}')
        self._logger.info(f'Tool {tool.tool_sn} mounted')
