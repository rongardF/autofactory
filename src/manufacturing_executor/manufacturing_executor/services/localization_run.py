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
"""Steps 5, 8 and 9: one run of the product localization."""

import threading

from action_msgs.msg import GoalStatus

from localization.action import LocalizeProduct
from manufacturing_executor.exceptions import SystemFailureError
from manufacturing_executor.services.abort_context import AbortContext
from manufacturing_executor.services.async_calls import ChildGoal
from manufacturing_executor.services.loop_clients import LoopClients


class LocalizationRun:
    """A running ``localize_product`` goal.

    The robot is free once the image collection is done (``collection_done`` feedback);
    the result follows after the calculation.
    """

    def __init__(self, goal: ChildGoal, result_timeout_s: float) -> None:
        self._goal = goal
        self._result_timeout_s = result_timeout_s
        self.collection_done = threading.Event()
        self._collection_failed = threading.Event()

    @classmethod
    def start(
        cls,
        clients: LoopClients,
        assembly_model_id: str,
        *,
        ctx: AbortContext,
        accept_timeout_s: float,
        result_timeout_s: float,
    ) -> 'LocalizationRun':
        """Send the goal and wait until it is accepted.

        :raises SystemFailureError: If the localizer rejects the goal.
        """
        holder: list[LocalizationRun] = []

        def on_feedback(message: LocalizeProduct.Impl.FeedbackMessage) -> None:
            if not holder:
                return
            run = holder[0]
            if message.feedback.collection_done:
                run.collection_done.set()
            else:
                run._collection_failed.set()

        goal = LocalizeProduct.Goal()
        goal.assembly_model_id = assembly_model_id
        child = ChildGoal.start(
            clients.localize_product,
            goal,
            ctx=ctx,
            description='Localization',
            accept_timeout_s=accept_timeout_s,
            feedback_callback=on_feedback,
        )
        run = cls(child, result_timeout_s)
        holder.append(run)
        return run

    def wait_collection_done(self, ctx: AbortContext) -> None:
        """Wait until the robot is free.

        :raises SystemFailureError: If the localization failed or ended during the image collection.
        """
        while not self.collection_done.is_set():
            ctx.raise_if_aborted()
            if self._goal.done.is_set():
                # The feedback may arrive after the result; a successful result implies the collection is done.
                outcome = self._goal.wait(0.0)
                if outcome.status == GoalStatus.STATUS_SUCCEEDED and outcome.result.success:
                    self.collection_done.set()
                    return
                raise SystemFailureError(
                    f'Localization ended before the images were collected: {outcome.error_text()}'
                )
            if self._collection_failed.is_set():
                raise SystemFailureError('Localization failed during the image collection')
            self._goal.done.wait(0.05)

    @property
    def result_ready(self) -> bool:
        """Whether the localization goal has already ended."""
        return self._goal.done.is_set()

    def wait_result(self) -> str:
        """Wait for the localization result.

        :returns: The localized product frame.
        :raises SystemFailureError: On failure or timeout.
        """
        outcome = self._goal.wait(self._result_timeout_s)
        result = outcome.ensure_success('Localization')
        return result.localized_product_frame

    def cancel(self) -> None:
        """Cancel the goal if it is still running (idempotent)."""
        self._goal.cancel(wait=True)
