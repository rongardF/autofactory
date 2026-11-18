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
"""The movement session that keeps the robot reserved for the manufacturing loop."""

from rclpy.node import Node

from manufacturing_executor.exceptions import SystemFailureError
from manufacturing_executor.models.manufacturing_executor_config_dto import (
    ManufacturingExecutorConfigDTO,
)
from manufacturing_executor.services.abort_context import AbortContext
from manufacturing_executor.services.async_calls import call_service
from manufacturing_executor.services.loop_clients import LoopClients
from movement_controller.services.movement_session import MovementSession
from movement_controller.srv import CloseSession, InSession


class MovementSessionClient:
    """One movement session, opened for a whole operation and extended before every movement."""

    def __init__(
        self,
        node: Node,
        config: ManufacturingExecutorConfigDTO,
        clients: LoopClients,
        ctx: AbortContext,
    ) -> None:
        self._logger = node.get_logger()
        self._timeout_s = config.service_call_timeout_s
        self._clients = clients
        self._ctx = ctx
        self._detached = AbortContext.detached()
        self._session = MovementSession(
            self._call_in_session,
            self._call_close_session,
            lambda: node.get_clock().now().nanoseconds / 1e9,
            timeout_s=config.movement_session_timeout_s,
        )

    @property
    def session_id(self) -> str:
        """Session id for the goals."""
        return self._session.session_id

    def ensure_valid(self) -> None:
        """Open the session, or restart its timeout when it is about to expire.

        :raises SystemFailureError: If the session cannot be acquired or extended.
        """
        error = self._session.ensure_valid()
        if error:
            self._ctx.raise_if_aborted()
            raise SystemFailureError(f'Movement session: {error}')

    def close(self) -> None:
        """Release the session; errors are logged, never raised."""
        error = self._session.close()
        if error:
            self._logger.error(f'Movement session: {error}')

    def _call_in_session(self, request: InSession.Request) -> InSession.Response | None:
        try:
            return call_service(
                self._clients.in_session, request, timeout_s=self._timeout_s, ctx=self._ctx,
                description='in_session',
            )
        except SystemFailureError:
            return None

    def _call_close_session(self, request: CloseSession.Request) -> CloseSession.Response | None:
        try:
            return call_service(
                self._clients.close_session, request, timeout_s=self._timeout_s, ctx=self._detached,
                description='close_session',
            )
        except SystemFailureError:
            return None
