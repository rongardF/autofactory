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
"""Step 1: product routing through the station workplace."""

from rclpy.node import Node

from manufacturing_executor.enums.routing_operation_enum import RoutingOperationEnum
from manufacturing_executor.exceptions import SystemFailureError
from manufacturing_executor.services.abort_context import AbortContext
from manufacturing_executor.services.async_calls import run_action
from manufacturing_executor.services.loop_clients import LoopClients
from manufacturing_executor.services.product_monitor import ProductMonitor
from station_workplace.action import WaitRouting
from station_workplace.enums.constants import UNKNOWN
from station_workplace.models.product_data_dto import ProductDataDTO


class RoutingService:
    """Waits for a product to arrive (injection) or to depart (ejection)."""

    def __init__(
        self, node: Node, clients: LoopClients, monitor: ProductMonitor, accept_timeout_s: float
    ) -> None:
        self._logger = node.get_logger()
        self._clients = clients
        self._monitor = monitor
        self._accept_timeout_s = accept_timeout_s

    def inject(self, ctx: AbortContext) -> ProductDataDTO:
        """Wait for the next product and start watching that it stays in place.

        :raises SystemFailureError: If routing fails or the product is unknown.
        """
        result = self._route(RoutingOperationEnum.INJECTION, ctx)
        if not result.product_data:
            raise SystemFailureError('Routing returned no product data')
        product = ProductDataDTO.from_msg(result.product_data[0])
        if UNKNOWN in (product.product_number, product.product_revision, product.serial_number):
            raise SystemFailureError(f'An unknown product arrived: {product}')
        self._monitor.arm(
            product, lambda message: ctx.abort(SystemFailureError(message))
        )
        self._logger.info(f'Product arrived: {product}')
        return product

    def eject(self, ctx: AbortContext) -> None:
        """Wait for the product to depart; the product is no longer watched."""
        self._monitor.disarm()
        self._route(RoutingOperationEnum.EJECTION, ctx)
        self._logger.info('Product departed')

    def _route(self, operation: RoutingOperationEnum, ctx: AbortContext) -> WaitRouting.Result:
        goal = WaitRouting.Goal()
        goal.routing_operation = operation.value
        outcome = run_action(
            self._clients.wait_routing,
            goal,
            ctx=ctx,
            description=f'Routing ({operation.value})',
            accept_timeout_s=self._accept_timeout_s,
            result_timeout_s=None,
        )
        return outcome.ensure_success(f'Routing ({operation.value})')
