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

"""Lifecycle and parameter service helper for an external end-tool node."""

from __future__ import annotations

import time

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.client import Client
from rclpy.lifecycle import LifecycleNode


class EndtoolLifecycleManager:
    """Manage lifecycle transitions and parameters of an end-tool node by name."""

    _EXPECTED_STATE_AFTER_TRANSITION: dict[int, str] = {
        Transition.TRANSITION_CONFIGURE: 'inactive',
        Transition.TRANSITION_ACTIVATE: 'active',
        Transition.TRANSITION_DEACTIVATE: 'inactive',
        Transition.TRANSITION_CLEANUP: 'unconfigured',
    }

    def __init__(
        self,
        node: LifecycleNode,
        node_name: str,
        service_timeout_sec: float = 5.0,
    ) -> None:
        """Initialize service clients for the target end-tool node."""
        self._node = node
        self._node_name = node_name
        self._service_timeout_sec = service_timeout_sec
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

    @property
    def node_name(self) -> str:
        """Return the managed end-tool node name."""
        return self._node_name

    def destroy(self) -> None:
        """Destroy created service clients and release callback resources."""
        self._node.destroy_client(self._change_state_client)
        self._node.destroy_client(self._get_state_client)
        self._node.destroy_client(self._set_parameters_client)
        self._node.destroy_client(self._get_parameters_client)

    def get_node_state(self) -> str | None:
        """Return current lifecycle state label for the end-tool node."""
        response = self._call(
            self._get_state_client,
            GetState.Request(),
            f'get state of {self._node_name}',
        )
        if response is None:
            return None
        return response.current_state.label

    def configure_node(self) -> bool:
        """Request transition to ``inactive`` from ``unconfigured``."""
        return self.change_state(Transition.TRANSITION_CONFIGURE, 'configure')

    def activate_node(self) -> bool:
        """Request transition to ``active`` from ``inactive``."""
        return self.change_state(Transition.TRANSITION_ACTIVATE, 'activate')

    def deactivate_node(self) -> bool:
        """Request transition to ``inactive`` from ``active``."""
        return self.change_state(Transition.TRANSITION_DEACTIVATE, 'deactivate')

    def cleanup_node(self) -> bool:
        """Request transition to ``unconfigured`` from ``inactive``."""
        return self.change_state(Transition.TRANSITION_CLEANUP, 'cleanup')

    def change_state(self, transition_id: int, label: str) -> bool:
        """Request lifecycle state transition and verify expected outcome."""
        request = ChangeState.Request()
        request.transition = Transition(id=transition_id, label=label)

        response = self._call(
            self._change_state_client,
            request,
            f'{label} {self._node_name}',
        )
        if response is not None and response.success:
            return True

        expected_state = self._EXPECTED_STATE_AFTER_TRANSITION.get(transition_id)
        if expected_state is not None and self.get_node_state() == expected_state:
            self._node.get_logger().warning(
                f"Transition '{label}' response for '{self._node_name}' was not "
                f"received, but the node is already in '{expected_state}'; "
                f"treating the transition as successful."
            )
            return True

        self._node.get_logger().error(
            f"Transition '{label}' on node '{self._node_name}' failed"
        )
        return False

    def set_parameter(self, parameter_name: str, parameter_value: ParameterValue) -> bool:
        """Set one parameter value on the end-tool node."""
        request = SetParameters.Request()
        request.parameters = [Parameter(name=parameter_name, value=parameter_value)]
        response = self._call(
            self._set_parameters_client,
            request,
            f'set parameter {parameter_name} on {self._node_name}',
        )

        if response is None or not response.results:
            return False
        return response.results[0].successful

    def set_parameters(self, parameters: dict[str, ParameterValue]) -> bool:
        """Set multiple parameter values on the end-tool node in one call."""
        request = SetParameters.Request()
        request.parameters = [
            Parameter(name=name, value=value)
            for name, value in parameters.items()
        ]
        response = self._call(
            self._set_parameters_client,
            request,
            f'set parameters on {self._node_name}',
        )

        if response is None or len(response.results) != len(parameters):
            return False
        return all(result.successful for result in response.results)

    def get_parameter(self, parameter_name: str) -> ParameterValue | None:
        """Get one parameter value from the end-tool node."""
        values = self.get_parameters([parameter_name])
        return values.get(parameter_name)

    def get_parameters(self, parameter_names: list[str]) -> dict[str, ParameterValue | None]:
        """Get multiple parameter values from the end-tool node."""
        request = GetParameters.Request()
        request.names = parameter_names
        response = self._call(
            self._get_parameters_client,
            request,
            f'get parameters from {self._node_name}',
        )

        if response is None:
            return {name: None for name in parameter_names}

        values = list(response.values)
        if len(values) != len(parameter_names):
            return {name: None for name in parameter_names}
        return {name: value for name, value in zip(parameter_names, values, strict=True)}

    def _call(self, client: Client, request, description: str):
        """Call service asynchronously and wait with wall-clock deadline."""
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
