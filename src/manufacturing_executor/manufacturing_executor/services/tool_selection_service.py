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
"""Steps 4 and 6: finding the tool and the nodes that belong to it."""

from rclpy.node import Node

from manufacturing_executor.exceptions import OperationFailureError, SystemFailureError
from manufacturing_executor.models.manufacturing_executor_config_dto import (
    ManufacturingExecutorConfigDTO,
)
from manufacturing_executor.models.tool_nodes_dto import ToolNodesDTO
from manufacturing_executor.services.abort_context import AbortContext
from manufacturing_executor.services.async_calls import call_service
from manufacturing_executor.services.lifecycle_client import LifecycleClientRegistry
from manufacturing_executor.services.loop_clients import LoopClients
from manufacturing_executor.services.tool_inventory import ToolInventory
from manufacturing_executor.utils.node_names import strip_slash
from repositories.models.operation_repository.operation_dto import OperationDTO
from tools_manager.model.tool_info_dto import ToolInfoDto
from tools_manager.srv import GetToolNodeName


class ToolSelectionService:
    """Selects the tool for an operation and resolves its pattern executor and optimizer."""

    def __init__(
        self,
        node: Node,
        config: ManufacturingExecutorConfigDTO,
        clients: LoopClients,
        registry: LifecycleClientRegistry,
        inventory: ToolInventory,
    ) -> None:
        self._logger = node.get_logger()
        self._config = config
        self._clients = clients
        self._registry = registry
        self._inventory = inventory

    def find_tool(self, operation: OperationDTO, ctx: AbortContext) -> ToolInfoDto:
        """The tool with the end tool and the material of the operation (no motion).

        :raises OperationFailureError: If the station has no such tool.
        :raises SystemFailureError: If the tool inventory is not available.
        """
        self._inventory.wait_ready(self._config.service_call_timeout_s, ctx)
        tool = self._inventory.find(operation.end_tool, operation.material)
        if tool is None:
            self._logger.error(
                f'No tool for end tool {operation.end_tool} and material {operation.material} in the station'
            )
            raise OperationFailureError(
                'Incorrect routing, station does not have the required endtool or the material'
            )
        return tool

    def resolve_nodes(self, tool: ToolInfoDto, ctx: AbortContext) -> ToolNodesDTO:
        """Endtool, pattern executor and optimizer node names of the tool.

        :raises SystemFailureError: If a name cannot be resolved or is not configured for this node.
        """
        request = GetToolNodeName.Request()
        request.tool_sn = tool.tool_sn
        response = call_service(
            self._clients.get_tool_node_name,
            request,
            timeout_s=self._config.service_call_timeout_s,
            ctx=ctx,
            description='get_tool_node_name',
        )
        if not response.success:
            raise SystemFailureError(f'get_tool_node_name failed: {response.error_message}')
        endtool_node = response.node_name
        executor_node = self._registry.get(endtool_node).get_parameter('pattern_executor', ctx)
        self._require_listed(executor_node, self._config.pattern_executor_nodes, 'pattern executor')
        optimizer_node = self._registry.get(executor_node).get_parameter('execution_optimizer', ctx)
        self._require_listed(optimizer_node, self._config.execution_optimizer_nodes, 'execution optimizer')
        return ToolNodesDTO(
            tool_sn=tool.tool_sn,
            endtool_node=endtool_node,
            pattern_executor_node=strip_slash(executor_node),
            optimizer_node=strip_slash(optimizer_node),
        )

    @staticmethod
    def _require_listed(name: str, allowed: tuple[str, ...], label: str) -> None:
        if strip_slash(name) not in {strip_slash(item) for item in allowed}:
            raise SystemFailureError(f'The {label} {name!r} is not listed in the node parameters')
