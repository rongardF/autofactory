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

from pydantic import BaseModel, ConfigDict, Field

from station_workplace.action import WaitRouting
from station_workplace.models.product_data_dto import ProductDataDTO


class RoutingResultDTO(BaseModel):
    """Outcome of a ``~/wait_routing`` goal."""

    model_config = ConfigDict(frozen=True, extra='forbid')

    success: bool = Field(default=False, description='True if the routing finished successfully.')
    error_message: str = Field(default='', description='Failure reason; empty on success.')
    product_data: ProductDataDTO | None = Field(
        default=None,
        description='Product in the workplace after an injection; None otherwise.',
    )
    routing_time_s: int = Field(
        default=0, ge=0,
        description='Time from goal start to completion [s], rounded down.',
    )

    def to_msg(self) -> WaitRouting.Result:
        """Convert to a ``WaitRouting.Result`` message."""
        result = WaitRouting.Result()
        result.routing_time_s = self.routing_time_s
        result.success = self.success
        result.error_message = self.error_message
        result.product_data = [] if self.product_data is None else [self.product_data.to_msg()]
        return result
