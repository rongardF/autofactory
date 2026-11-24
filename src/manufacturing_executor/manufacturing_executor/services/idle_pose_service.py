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
"""Park the robot at the idle pose while the product is ejected."""

from rclpy.node import Node

from manufacturing_executor.exceptions import ManufacturingError, SystemFailureError
from manufacturing_executor.models.manufacturing_executor_config_dto import (
    ManufacturingExecutorConfigDTO,
)
from manufacturing_executor.services.abort_context import AbortContext
from manufacturing_executor.services.async_calls import ChildGoal
from manufacturing_executor.services.loop_clients import LoopClients
from manufacturing_executor.services.movement_session_client import MovementSessionClient
from manufacturing_executor.utils.log_format import format_pose

_DESCRIPTION = 'Move to the idle pose'


class IdlePoseMove:
    """A running move to the idle pose.

    A failure of the move aborts the goal context as soon as it happens, which also ends
    whatever the loop is waiting for. :meth:`close` must always be called.
    """

    def __init__(self, session: MovementSessionClient, goal: ChildGoal, ctx: AbortContext, node: Node) -> None:
        self._session = session
        self._goal = goal
        self._ctx = ctx
        self._logger = node.get_logger()
        self._closed = False
        goal.add_done_callback(self._on_done)

    def wait(self) -> None:
        """Wait until the robot is at the idle pose.

        :raises SystemFailureError: If the move failed.
        """
        self._goal.wait(None).ensure_success(_DESCRIPTION)
        self._logger.info('Robot is at the idle pose')

    def close(self) -> None:
        """Cancel the move if it still runs and release the movement session; never raises."""
        self._closed = True
        self._goal.cancel(wait=True)
        self._goal.release()
        self._session.close()

    def _on_done(self) -> None:
        if self._closed:
            return
        try:
            self._goal.wait(0.0).ensure_success(_DESCRIPTION)
        except ManufacturingError as failure:
            self._logger.error(str(failure))
            self._ctx.abort(failure)
        except Exception as error:  # noqa: BLE001 - the callback must not raise into the executor
            self._logger.error(f'{_DESCRIPTION} failed: {error!r}')
            self._ctx.abort(SystemFailureError(f'{_DESCRIPTION} failed: {error}'))


class IdlePoseService:
    """Starts the move of the flange to the configured idle pose in its own movement session."""

    def __init__(self, node: Node, config: ManufacturingExecutorConfigDTO, clients: LoopClients) -> None:
        self._node = node
        self._logger = node.get_logger()
        self._config = config
        self._clients = clients

    def start_move(self, ctx: AbortContext) -> IdlePoseMove:
        """Start a PTP of ``flange_tcp`` to ``idle_pose`` in ``base_frame_id`` without waiting for it.

        :raises SystemFailureError: If the session cannot be acquired or the goal is not accepted.
        """
        pose = self._config.idle_pose.to_pose_stamped(self._config.base_frame_id)
        session = MovementSessionClient(self._node, self._config, self._clients, ctx)
        try:
            self._logger.info(
                f'Moving the robot ({self._config.flange_tcp}) to the idle pose {format_pose(pose)}'
            )
            goal = session.start_ptp(pose, self._config.flange_tcp, _DESCRIPTION, ctx)
        except BaseException:
            session.close()
            raise
        return IdlePoseMove(session, goal, ctx, self._node)
