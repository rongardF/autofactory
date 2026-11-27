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
"""Common lifecycle base node of all Autofactory nodes.

Adds transition-error reporting on the ``/transition_error`` topic: every node
publishes why one of its lifecycle transitions failed. The errors are consumed by
:class:`~station_utils.services.node_manager.NodeManager`.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from lifecycle_msgs.msg import State
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy

from station_utils.msg import TransitionError

TRANSITION_ERROR_TOPIC = '/transition_error'
TRANSITION_ERROR_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

# Transitional state -> name of the transition being executed in it.
_TRANSITION_NAMES: dict[int, str] = {
    State.TRANSITION_STATE_CONFIGURING: 'configure',
    State.TRANSITION_STATE_CLEANINGUP: 'cleanup',
    State.TRANSITION_STATE_SHUTTINGDOWN: 'shutdown',
    State.TRANSITION_STATE_ACTIVATING: 'activate',
    State.TRANSITION_STATE_DEACTIVATING: 'deactivate',
}


class AutofactoryBaseNode(LifecycleNode):
    """``LifecycleNode`` that reports its failed transitions on ``/transition_error``."""

    def __init__(self, node_name: str, **kwargs: Any) -> None:
        """Create the ``/transition_error`` publisher.

        :param node_name: ROS 2 node name.
        :param kwargs: Forwarded to :class:`rclpy.lifecycle.LifecycleNode`.
        """
        super().__init__(node_name, **kwargs)
        # A regular (not lifecycle-managed) publisher, so errors can be published in any state.
        self._transition_error_publisher = self.create_publisher(
            TransitionError, TRANSITION_ERROR_TOPIC, TRANSITION_ERROR_QOS
        )
        self._wrap_transition_callbacks()

    def _publish_transition_error(self, message: str) -> None:
        """Publish why a lifecycle transition of this node failed.

        :param message: Short, human-readable reason of the failure.
        """
        msg = TransitionError()
        msg.message = message
        msg.source_node = self.get_fully_qualified_name()
        msg.stamp = self.get_clock().now().to_msg()
        self._transition_error_publisher.publish(msg)

    def _wrap_transition_callbacks(self) -> None:
        """Publish a generic transition error when a transition ends in ``ERROR``.

        rclpy keeps the transition callbacks in ``LifecycleNode._callbacks`` and turns
        an unhandled exception into ``ERROR`` without logging it. Wrapping the
        registered callbacks catches that case regardless of which methods the
        subclasses override.
        """
        for state_id, transition in _TRANSITION_NAMES.items():
            callback = self._callbacks.get(state_id)
            if callback is not None:
                self._callbacks[state_id] = self._guarded_transition(transition, callback)

    def _guarded_transition(
        self,
        transition: str,
        callback: Callable[[LifecycleState], TransitionCallbackReturn],
    ) -> Callable[[LifecycleState], TransitionCallbackReturn]:
        def guarded(previous_state: LifecycleState) -> TransitionCallbackReturn:
            try:
                result = callback(previous_state)
            except Exception as error:  # noqa: BLE001 - rclpy would swallow it silently
                self.get_logger().error(f"Unhandled exception in '{transition}' transition: {error!r}")
                self._publish_transition_error(f"'{transition}' transition raised {type(error).__name__}: {error}")
                return TransitionCallbackReturn.ERROR
            if result == TransitionCallbackReturn.ERROR:
                self._publish_transition_error(f"'{transition}' transition ended with an error")
            return result

        return guarded
