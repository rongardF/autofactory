# Copyright (c) 2026, Tools Manager Contributors
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
"""Drive the lifecycle and parameters of an external ROS 2 lifecycle node.

``NodeStateManager`` wraps the standard services a ``LifecycleNode`` exposes —
``<node>/change_state``, ``<node>/get_state``, ``<node>/set_parameters`` and
``<node>/get_parameters`` — so the owning node can configure, activate,
deactivate and clean up another node (and read/write its parameters) by name.

All service calls use ``call_async`` and poll the returned future with a
monotonic wall-clock deadline, so they can run on an executor thread without
nested spinning: the future is serviced by another thread of the owning node's
:class:`~rclpy.executors.MultiThreadedExecutor`.
"""

from __future__ import annotations

from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
import time

from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.client import Client as ServiceClient
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn


class NodeStateManager:
    """Manage lifecycle transitions and parameters of an external node by name."""

    def __init__(
        self,
        node: LifecycleNode,
        node_name: str,
        service_timeout_sec: float = 5.0,
    ) -> None:
        """Create the lifecycle and parameter service clients for the target node.

        :param node: Owning node used to create clients and access the logger.
        :param node_name: Name of the target lifecycle node to manage. A leading
            ``/`` is added when absent so the service names are fully qualified
            and independent of the owning node's namespace.
        :param service_timeout_sec: Timeout for service availability and results.
        """
        self._node = node
        self.node_name = node_name
        self._service_timeout_sec = service_timeout_sec

        # Service clients must live in their own callback group, separate from
        # the owning node's default (mutually exclusive) group. The lifecycle
        # transition callbacks (on_configure/on_activate) that drive these
        # clients run in that default group; if the clients shared it, the
        # MultiThreadedExecutor could not service their responses while a
        # transition callback is blocked polling the future, causing a deadlock.
        self._callback_group = ReentrantCallbackGroup()

        fully_qualified_name = node_name if node_name.startswith('/') else f'/{node_name}'

        self._change_state_client = node.create_client(
            ChangeState,
            f'{fully_qualified_name}/change_state',
            callback_group=self._callback_group,
        )
        self._get_state_client = node.create_client(
            GetState,
            f'{fully_qualified_name}/get_state',
            callback_group=self._callback_group,
        )
        self._set_parameters_client = node.create_client(
            SetParameters,
            f'{fully_qualified_name}/set_parameters',
            callback_group=self._callback_group,
        )
        self._get_parameters_client = node.create_client(
            GetParameters,
            f'{fully_qualified_name}/get_parameters',
            callback_group=self._callback_group,
        )

    def configure_node(self) -> TransitionCallbackReturn:
        """Transition the target node from ``unconfigured`` to ``inactive``.

        :returns: ``SUCCESS`` if the transition was accepted, ``FAILURE`` otherwise.
        """
        self._node.get_logger().info(f'Configuring node: {self.node_name}')
        return self._change_state(Transition.TRANSITION_CONFIGURE, 'configure')

    def activate_node(self) -> TransitionCallbackReturn:
        """Transition the target node from ``inactive`` to ``active``.

        :returns: ``SUCCESS`` if the transition was accepted, ``FAILURE`` otherwise.
        """
        self._node.get_logger().info(f'Activating node: {self.node_name}')
        return self._change_state(Transition.TRANSITION_ACTIVATE, 'activate')

    def deactivate_node(self) -> TransitionCallbackReturn:
        """Transition the target node from ``active`` to ``inactive``.

        :returns: ``SUCCESS`` if the transition was accepted, ``FAILURE`` otherwise.
        """
        self._node.get_logger().info(f'Deactivating node: {self.node_name}')
        return self._change_state(Transition.TRANSITION_DEACTIVATE, 'deactivate')

    def unconfigure_node(self) -> TransitionCallbackReturn:
        """Transition the target node from ``inactive`` back to ``unconfigured``.

        :returns: ``SUCCESS`` if the transition was accepted, ``FAILURE`` otherwise.
        """
        self._node.get_logger().info(f'Unconfiguring node: {self.node_name}')
        return self._change_state(Transition.TRANSITION_CLEANUP, 'cleanup')

    def get_node_state(self) -> str | None:
        """Query the current lifecycle state label of the target node.

        :returns: The current state label (e.g. ``'active'``), or ``None`` if the
            state could not be retrieved.
        """
        response = self._call(
            self._get_state_client, GetState.Request(), f'get state of {self.node_name}'
        )
        if response is None:
            return None
        current_state: State = response.current_state
        return current_state.label

    def set_parameter(self, parameter_name: str, parameter_value: ParameterValue) -> bool:
        """Set a single parameter on the target node.

        :param parameter_name: Name of the parameter to set.
        :param parameter_value: New value for the parameter.
        :returns: ``True`` if the node accepted the new value, ``False`` otherwise.
        """
        self._node.get_logger().info(
            f'Setting parameter {parameter_name} on node: {self.node_name}'
        )
        request = SetParameters.Request()
        request.parameters = [Parameter(name=parameter_name, value=parameter_value)]
        response = self._call(
            self._set_parameters_client,
            request,
            f'set parameter {parameter_name} on {self.node_name}',
        )
        if response is None or not response.results:
            return False
        return response.results[0].successful

    def get_parameter(self, parameter_name: str) -> ParameterValue | None:
        """Read a single parameter from the target node.

        :param parameter_name: Name of the parameter to read.
        :returns: The parameter value, or ``None`` if it could not be retrieved.
        """
        request = GetParameters.Request()
        request.names = [parameter_name]
        response = self._call(
            self._get_parameters_client,
            request,
            f'get parameter {parameter_name} from {self.node_name}',
        )
        if response is None or not response.values:
            return None
        return response.values[0]

    # Primary lifecycle state each managed transition should leave the node in.
    # Used to confirm a transition that actually took effect even when its
    # service response was lost: a freshly created client can hit a DDS
    # discovery race on the response path (the request is delivered, but the
    # target node replies before it has discovered the client's response
    # reader, so the reply is dropped) while the transition itself still
    # completes on the target node.
    _EXPECTED_STATE_AFTER_TRANSITION: dict[int, str] = {
        Transition.TRANSITION_CONFIGURE: 'inactive',
        Transition.TRANSITION_ACTIVATE: 'active',
        Transition.TRANSITION_DEACTIVATE: 'inactive',
        Transition.TRANSITION_CLEANUP: 'unconfigured',
    }

    def _change_state(self, transition_id: int, label: str) -> TransitionCallbackReturn:
        """Request a lifecycle transition and map the result to a callback return.

        :param transition_id: ``lifecycle_msgs/Transition`` id constant.
        :param label: Human-readable transition label for logging.
        :returns: ``SUCCESS`` if the transition was accepted, ``FAILURE`` otherwise.
        """
        request = ChangeState.Request()
        request.transition = Transition(id=transition_id, label=label)
        response = self._call(
            self._change_state_client,
            request,
            f'{label} {self.node_name}',
        )
        if response is not None and response.success:
            return TransitionCallbackReturn.SUCCESS

        # The call failed or its response was lost in transit. A lost response
        # does not mean the transition failed, so confirm the node's actual
        # state before giving up (DDS discovery has settled by now, so the
        # follow-up get_state call is no longer subject to the response race).
        expected_state = self._EXPECTED_STATE_AFTER_TRANSITION.get(transition_id)
        if expected_state is not None and self.get_node_state() == expected_state:
            self._node.get_logger().warning(
                f"Transition '{label}' response for '{self.node_name}' was not "
                f"received, but the node is already in '{expected_state}'; "
                f"treating the transition as successful."
            )
            return TransitionCallbackReturn.SUCCESS

        self._node.get_logger().error(
            f"Transition '{label}' on node '{self.node_name}' failed"
        )
        return TransitionCallbackReturn.FAILURE

    def _call(self, client: ServiceClient, request, description: str):
        """Call a service and wait for the result within the configured timeout.

        Uses ``call_async`` and polls the future so this can run on an executor
        thread without nested spinning (the response is serviced by another
        thread of the owning node's ``MultiThreadedExecutor``).

        The wait loop is measured against the monotonic wall clock and uses
        ``time.sleep``, deliberately **not** the node clock. These calls are
        often made from within a lifecycle transition callback that runs in the
        node's default (mutually exclusive) callback group. Under
        ``use_sim_time`` the ROS clock is driven by the ``/clock`` subscription,
        which lives in that same group; blocking it on the node clock would stop
        sim time from advancing and deadlock the wait forever. Wall-clock timing
        keeps the timeout honest regardless of sim time.

        :param client: Service client to call.
        :param request: Request message.
        :param description: Human-readable operation name for logging.
        :returns: The service response, or ``None`` on unavailability/timeout.
        """
        if not client.wait_for_service(timeout_sec=self._service_timeout_sec):
            self._node.get_logger().error(f'Service not available for: {description}')
            return None

        future = client.call_async(request)
        deadline = time.monotonic() + self._service_timeout_sec
        while not future.done():
            if time.monotonic() > deadline:
                self._node.get_logger().error(f'Service call timed out for: {description}')
                return None
            time.sleep(0.01)

        return future.result()
