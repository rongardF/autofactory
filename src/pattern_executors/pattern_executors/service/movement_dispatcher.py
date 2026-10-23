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

"""Dispatch dispensing movements to movement_controller with action-timed hooks."""

from __future__ import annotations

from copy import deepcopy
from threading import Event, Lock, Thread
from time import monotonic, sleep
from typing import Any, Callable

from movement_controller.action import ExecuteTrajectory
from pattern_executors.msg import DispensingMovement, PatternAction
from pydantic import BaseModel, ConfigDict, Field
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.lifecycle import LifecycleNode
from rclpy.task import Future

ActionExecutorType = Callable[[PatternAction, DispensingMovement], None]


class _PathActionState(BaseModel):
    """Track per-path execution state for idempotent start/end action firing."""

    model_config = ConfigDict(arbitrary_types_allowed=True)

    start_dispatched: bool = False
    end_dispatched: bool = False
    start_done: Event = Field(default_factory=Event)


class MovementDispatcher:
    """Split movement batches and execute them with feedback-timed action hooks."""

    def __init__(
        self,
        node: LifecycleNode,
        movement_action_client: ActionClient,
        movement_server_timeout_s: float,
    ) -> None:
        """Create dispatcher bound to a movement action client."""
        self._node = node
        self._movement_action_client = movement_action_client
        self._movement_server_timeout_s = movement_server_timeout_s
        self._state_lock = Lock()
        self._active_goal_handle: ClientGoalHandle | None = None

    def split_into_batches(
        self,
        movements: list[DispensingMovement],
    ) -> list[list[DispensingMovement]]:
        """Split movements into batches, cutting after any movement with wait action."""
        batches: list[list[DispensingMovement]] = []
        current_batch: list[DispensingMovement] = []

        for movement in movements:
            current_batch.append(movement)
            has_wait_action = any(
                action.type == PatternAction.TYPE_WAIT
                for action in movement.actions
            )
            if has_wait_action:
                batches.append(current_batch)
                current_batch = []

        if current_batch:
            batches.append(current_batch)

        return batches

    def execute_batches(
        self,
        movements: list[DispensingMovement],
        frame_id: str,
        tool_frame: str,
        action_executor: ActionExecutorType,
    ) -> ExecuteTrajectory.Result:
        """Execute all movement batches and fire start/end actions from feedback."""
        result = ExecuteTrajectory.Result()
        result.success = True
        result.error_message = ''
        result.trajectory_paths_completed = []

        batches = self.split_into_batches(movements)
        for batch in batches:
            batch_result = self._execute_single_batch(
                batch,
                frame_id,
                tool_frame,
                action_executor,
            )
            if not batch_result.success:
                return batch_result
            result.trajectory_paths_completed.extend(batch_result.trajectory_paths_completed)

        return result

    def cancel_active_goal(self, timeout_s: float | None = None) -> bool:
        """Cancel the current in-flight movement goal if one exists."""
        with self._state_lock:
            goal_handle = self._active_goal_handle

        if goal_handle is None:
            return True

        cancel_future = goal_handle.cancel_goal_async()
        cancel_response = self._wait_future(
            cancel_future,
            timeout_s if timeout_s is not None else self._movement_server_timeout_s,
            'cancel movement goal',
        )
        if cancel_response is None:
            return False

        with self._state_lock:
            self._active_goal_handle = None
        return True

    def _execute_single_batch(
        self,
        batch: list[DispensingMovement],
        frame_id: str,
        tool_frame: str,
        action_executor: ActionExecutorType,
    ) -> ExecuteTrajectory.Result:
        """Send one ExecuteTrajectory goal for a movement batch and process feedback."""
        movement_by_path_id = {
            movement.path.path_id: movement
            for movement in batch
        }
        action_state_by_path_id = {
            path_id: _PathActionState()
            for path_id in movement_by_path_id
        }

        goal = ExecuteTrajectory.Goal()
        goal.paths = []
        for movement in batch:
            stamped_path = deepcopy(movement.path)
            stamped_path.target_pose.header.frame_id = frame_id
            stamped_path.tool_frame = tool_frame
            goal.paths.append(stamped_path)

        if not self._movement_action_client.wait_for_server(
            timeout_sec=self._movement_server_timeout_s
        ):
            result = ExecuteTrajectory.Result()
            result.success = False
            result.error_message = 'Execute trajectory action server not available'
            result.trajectory_paths_completed = []
            return result

        callback_errors: list[Exception] = []
        callback_errors_lock = Lock()

        def _record_callback_error(error: Exception) -> None:
            with callback_errors_lock:
                callback_errors.append(error)

        def _run_actions_for_phase(path_id: str, phase: str) -> None:
            movement = movement_by_path_id[path_id]
            state = action_state_by_path_id[path_id]
            try:
                if phase == PatternAction.TIMING_END:
                    state.start_done.wait()

                for action in movement.actions:
                    if action.timing != phase:
                        continue
                    action_executor(action, movement)
            except Exception as error:  # noqa: BLE001
                _record_callback_error(error)
            finally:
                if phase == PatternAction.TIMING_START:
                    state.start_done.set()

        def _feedback_callback(feedback_msg: ExecuteTrajectory.Impl.FeedbackMessage) -> None:
            feedback = feedback_msg.feedback
            if feedback.status not in {PatternAction.TIMING_START, PatternAction.TIMING_END}:
                if feedback.status not in {'executing', 'completed'}:
                    return

            phase = PatternAction.TIMING_START if feedback.status == 'executing' else PatternAction.TIMING_END
            for path_id in feedback.trajectory_path_ids:
                movement = movement_by_path_id.get(path_id)
                if movement is None:
                    continue

                state = action_state_by_path_id[path_id]
                should_dispatch = False
                with self._state_lock:
                    if phase == PatternAction.TIMING_START and not state.start_dispatched:
                        state.start_dispatched = True
                        should_dispatch = True
                    elif phase == PatternAction.TIMING_END and not state.end_dispatched:
                        state.end_dispatched = True
                        should_dispatch = True

                if not should_dispatch:
                    continue

                thread = Thread(
                    target=_run_actions_for_phase,
                    args=(path_id, phase),
                    daemon=True,
                )
                thread.start()

        send_future = self._movement_action_client.send_goal_async(
            goal,
            feedback_callback=_feedback_callback,
        )
        goal_handle = self._wait_future(
            send_future,
            self._movement_server_timeout_s,
            'send movement goal',
        )
        if goal_handle is None:
            result = ExecuteTrajectory.Result()
            result.success = False
            result.error_message = 'Failed to send movement goal'
            result.trajectory_paths_completed = []
            return result

        if not goal_handle.accepted:
            result = ExecuteTrajectory.Result()
            result.success = False
            result.error_message = 'Movement goal was rejected by action server'
            result.trajectory_paths_completed = []
            return result

        with self._state_lock:
            self._active_goal_handle = goal_handle

        get_result_future = goal_handle.get_result_async()
        result_message = self._wait_future(
            get_result_future,
            0.0,
            'await movement goal result',
        )

        with self._state_lock:
            if self._active_goal_handle == goal_handle:
                self._active_goal_handle = None

        if result_message is None:
            result = ExecuteTrajectory.Result()
            result.success = False
            result.error_message = 'Timed out waiting for movement goal result'
            result.trajectory_paths_completed = []
            return result

        for state in action_state_by_path_id.values():
            state.start_done.wait()

        if callback_errors:
            callback_error = callback_errors[0]
            result = ExecuteTrajectory.Result()
            result.success = False
            result.error_message = f'Action callback failed: {callback_error}'
            result.trajectory_paths_completed = []
            return result

        movement_result = result_message.result
        if not movement_result.success:
            return movement_result

        return movement_result

    def _wait_future(
        self,
        future: Future,
        timeout_s: float,
        description: str,
    ) -> Any | None:
        """Wait for a future using wall-clock timing and return result or None."""
        deadline = None if timeout_s <= 0.0 else monotonic() + timeout_s

        while not future.done():
            if deadline is not None and monotonic() > deadline:
                self._node.get_logger().error(
                    f'Timed out while waiting to {description}'
                )
                return None
            sleep(0.01)

        return future.result()
