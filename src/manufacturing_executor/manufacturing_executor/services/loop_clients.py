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
"""Clients of the nodes the manufacturing loop talks to."""

from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.client import Client
from rclpy.node import Node

from cameras.action import GetImage
from execution_optimizer.srv import DispensingOptimizePatterns
from localization.action import LocalizeProduct
from manufacturing_executor.enums.dependency_key_enum import DependencyKeyEnum
from manufacturing_executor.models.manufacturing_executor_config_dto import (
    ManufacturingExecutorConfigDTO,
)
from manufacturing_executor.utils.node_names import topic
from movement_controller.action import ExecuteTrajectory
from movement_controller.srv import CloseSession, InSession
from pattern_executors.action import ExecuteDispensingPattern
from production_manager.srv import GetProductionData, ReportProduction
from repositories.srv import ImageRepositoryUploadImage, OperationRepositoryGetOperation
from station_workplace.action import WaitRouting
from tools_manager.action import MountTool
from tools_manager.srv import GetToolNodeName


class LoopClients:
    """Service and action clients, all in one reentrant callback group.

    Clients that depend on the selected tool (optimizer, pattern executor) are created
    on first use and cached.
    """

    def __init__(self, node: Node, config: ManufacturingExecutorConfigDTO, callback_group: CallbackGroup) -> None:
        self._node = node
        self._group = callback_group
        dependencies = config.dependency_nodes

        def name(key: DependencyKeyEnum) -> str:
            return dependencies.node_name(key)

        self.wait_routing = ActionClient(
            node, WaitRouting, topic(name(DependencyKeyEnum.STATION_WORKPLACE), 'wait_routing'),
            callback_group=callback_group,
        )
        self.get_production_data: Client = node.create_client(
            GetProductionData, topic(name(DependencyKeyEnum.PRODUCTION_MANAGER), 'get_production_data'),
            callback_group=callback_group,
        )
        self.report_production: Client = node.create_client(
            ReportProduction, topic(name(DependencyKeyEnum.PRODUCTION_MANAGER), 'report_production'),
            callback_group=callback_group,
        )
        self.get_operation: Client = node.create_client(
            OperationRepositoryGetOperation, topic(name(DependencyKeyEnum.OPERATION_REPOSITORY), 'get_operation'),
            callback_group=callback_group,
        )
        self.localize_product = ActionClient(
            node, LocalizeProduct, topic(name(DependencyKeyEnum.LOCALIZER), 'localize_product'),
            callback_group=callback_group,
        )
        self.get_tool_node_name: Client = node.create_client(
            GetToolNodeName, topic(name(DependencyKeyEnum.TOOLS_MANAGER), 'get_tool_node_name'),
            callback_group=callback_group,
        )
        self.mount_tool = ActionClient(
            node, MountTool, topic(name(DependencyKeyEnum.TOOLS_MANAGER), 'mount_tool'),
            callback_group=callback_group,
        )
        movement = name(DependencyKeyEnum.MOVEMENT_CONTROLLER)
        self.execute_trajectory = ActionClient(
            node, ExecuteTrajectory, topic(movement, 'execute_trajectory'), callback_group=callback_group
        )
        self.in_session: Client = node.create_client(
            InSession, topic(movement, 'in_session'), callback_group=callback_group
        )
        self.close_session: Client = node.create_client(
            CloseSession, topic(movement, 'close_session'), callback_group=callback_group
        )
        self.get_image = ActionClient(
            node, GetImage, topic(name(DependencyKeyEnum.CAMERA), 'get_image'), callback_group=callback_group
        )
        self.upload_image: Client = node.create_client(
            ImageRepositoryUploadImage, topic(name(DependencyKeyEnum.IMAGE_REPOSITORY), 'upload_image'),
            callback_group=callback_group,
        )
        self._optimize: dict[str, Client] = {}
        self._execute_pattern: dict[str, ActionClient] = {}

    def optimize_patterns(self, optimizer_node: str) -> Client:
        """``optimize_patterns`` client of one optimizer."""
        if optimizer_node not in self._optimize:
            self._optimize[optimizer_node] = self._node.create_client(
                DispensingOptimizePatterns, topic(optimizer_node, 'optimize_patterns'),
                callback_group=self._group,
            )
        return self._optimize[optimizer_node]

    def execute_dispensing_pattern(self, executor_node: str) -> ActionClient:
        """``execute_dispensing_pattern`` client of one pattern executor."""
        if executor_node not in self._execute_pattern:
            self._execute_pattern[executor_node] = ActionClient(
                self._node, ExecuteDispensingPattern, topic(executor_node, 'execute_dispensing_pattern'),
                callback_group=self._group,
            )
        return self._execute_pattern[executor_node]

    def destroy(self) -> None:
        """Destroy every client."""
        for action_client in (
            self.wait_routing, self.localize_product, self.mount_tool, self.execute_trajectory,
            self.get_image, *self._execute_pattern.values(),
        ):
            action_client.destroy()
        for client in (
            self.get_production_data, self.report_production, self.get_operation, self.get_tool_node_name,
            self.in_session, self.close_session, self.upload_image, *self._optimize.values(),
        ):
            self._node.destroy_client(client)
        self._optimize.clear()
        self._execute_pattern.clear()
