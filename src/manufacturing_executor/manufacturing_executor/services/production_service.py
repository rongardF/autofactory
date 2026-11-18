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
"""Steps 2, 3 and 14: production data, operation and production report."""

from rclpy.node import Node

from manufacturing_executor.exceptions import OperationFailureError, SystemFailureError
from manufacturing_executor.services.abort_context import AbortContext
from manufacturing_executor.services.async_calls import call_service
from manufacturing_executor.services.loop_clients import LoopClients
from production_manager.model.production_data_dto import ProductionDataDTO
from production_manager.srv import GetProductionData, ReportProduction
from repositories.models.operation_repository.operation_dto import OperationDTO
from repositories.srv import OperationRepositoryGetOperation
from station_workplace.models.product_data_dto import ProductDataDTO


class ProductionService:
    """Talks to the production manager and the operation repository."""

    FETCH_OPERATION_FAILURE_REASON = 'Failed to fetch operation data for product'

    def __init__(self, node: Node, clients: LoopClients, timeout_s: float) -> None:
        self._logger = node.get_logger()
        self._clients = clients
        self._timeout_s = timeout_s

    def get_production_data(self, product: ProductDataDTO, ctx: AbortContext) -> ProductionDataDTO:
        """Assembly and operation of the product.

        :raises SystemFailureError: If the production manager does not provide them.
        """
        request = GetProductionData.Request()
        request.product_number = product.product_number
        request.product_revision = product.product_revision
        request.serial_number = product.serial_number
        response = call_service(
            self._clients.get_production_data,
            request,
            timeout_s=self._timeout_s,
            ctx=ctx,
            description='get_production_data',
        )
        if not response.success:
            raise SystemFailureError(f'get_production_data failed: {response.error_message}')
        return ProductionDataDTO(
            assembly_id=response.assembly_id,
            operation_id=response.operation_id,
            assembly_model_id=response.assembly_model_id,
            success=True,
        )

    def get_operation(self, production: ProductionDataDTO, ctx: AbortContext) -> OperationDTO:
        """The operation of the product.

        :raises OperationFailureError: If the operation cannot be fetched.
        :raises SystemFailureError: If the service is unavailable.
        """
        request = OperationRepositoryGetOperation.Request()
        request.operation_id = production.operation_id
        request.assembly_id = production.assembly_id
        response = call_service(
            self._clients.get_operation,
            request,
            timeout_s=self._timeout_s,
            ctx=ctx,
            description='get_operation',
        )
        if not response.success or not response.operation:
            self._logger.error(f'get_operation failed: {response.error_message}')
            raise OperationFailureError(self.FETCH_OPERATION_FAILURE_REASON)
        try:
            return OperationDTO.from_msg(response.operation[0])
        except ValueError as error:
            self._logger.error(f'Invalid operation data: {error}')
            raise OperationFailureError(self.FETCH_OPERATION_FAILURE_REASON) from error

    def report(self, production: ProductionDataDTO, success: bool, reason: str, ctx: AbortContext) -> bool:
        """Report the outcome of the operation.

        :returns: ``finished_in_station`` of the response.
        :raises SystemFailureError: If the report is not accepted.
        """
        request = ReportProduction.Request()
        request.assembly_id = production.assembly_id
        request.operation_id = production.operation_id
        request.success = success
        request.reason = reason
        response = call_service(
            self._clients.report_production,
            request,
            timeout_s=self._timeout_s,
            ctx=ctx,
            description='report_production',
        )
        if not response.success:
            raise SystemFailureError(f'report_production failed: {response.error_message}')
        return bool(response.finished_in_station)
