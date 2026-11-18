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
"""Client for the lifecycle and parameter services of another node."""

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node

from manufacturing_executor.enums.lifecycle_state_enum import LifecycleStateEnum
from manufacturing_executor.exceptions import SystemFailureError
from manufacturing_executor.services.abort_context import AbortContext
from manufacturing_executor.services.async_calls import call_service
from manufacturing_executor.utils.node_names import fq_name


class LifecycleClient:
    """Reads the state and parameters of, and drives the transitions of, one node."""

    _EXPECTED_STATE = {
        Transition.TRANSITION_CONFIGURE: LifecycleStateEnum.INACTIVE,
        Transition.TRANSITION_ACTIVATE: LifecycleStateEnum.ACTIVE,
        Transition.TRANSITION_DEACTIVATE: LifecycleStateEnum.INACTIVE,
        Transition.TRANSITION_CLEANUP: LifecycleStateEnum.UNCONFIGURED,
    }
    _TRANSITION_LABEL = {
        Transition.TRANSITION_CONFIGURE: 'configure',
        Transition.TRANSITION_ACTIVATE: 'activate',
        Transition.TRANSITION_DEACTIVATE: 'deactivate',
        Transition.TRANSITION_CLEANUP: 'cleanup',
    }

    def __init__(
        self,
        node: Node,
        node_name: str,
        *,
        callback_group: CallbackGroup,
        call_timeout_s: float,
        transition_timeout_s: float,
    ) -> None:
        self._node = node
        self.node_name = fq_name(node_name)
        self._call_timeout_s = call_timeout_s
        self._transition_timeout_s = transition_timeout_s
        self._get_state = node.create_client(
            GetState, f'{self.node_name}/get_state', callback_group=callback_group
        )
        self._change_state = node.create_client(
            ChangeState, f'{self.node_name}/change_state', callback_group=callback_group
        )
        self._get_parameters = node.create_client(
            GetParameters, f'{self.node_name}/get_parameters', callback_group=callback_group
        )

    def destroy(self) -> None:
        """Destroy the service clients."""
        for client in (self._get_state, self._change_state, self._get_parameters):
            self._node.destroy_client(client)

    def get_state(self, ctx: AbortContext | None = None) -> LifecycleStateEnum:
        """Current lifecycle state.

        :raises SystemFailureError: If the state cannot be read or is not a primary state.
        """
        response = call_service(
            self._get_state,
            GetState.Request(),
            timeout_s=self._call_timeout_s,
            ctx=ctx or AbortContext.detached(),
            description=f'get_state of {self.node_name}',
        )
        try:
            return LifecycleStateEnum(response.current_state.label)
        except ValueError as error:
            raise SystemFailureError(
                f'{self.node_name} is in the transitional state {response.current_state.label!r}'
            ) from error

    def _transition(self, transition_id: int, ctx: AbortContext | None) -> None:
        context = ctx or AbortContext.detached()
        label = self._TRANSITION_LABEL[transition_id]
        request = ChangeState.Request()
        request.transition = Transition(id=transition_id, label=label)
        try:
            response = call_service(
                self._change_state,
                request,
                timeout_s=self._transition_timeout_s,
                ctx=context,
                description=f'{label} {self.node_name}',
            )
            succeeded = bool(response.success)
        except SystemFailureError:
            context.raise_if_aborted()
            succeeded = False
        # A lost response does not mean the transition failed: confirm the actual state.
        if not succeeded:
            try:
                succeeded = self.get_state(context) == self._EXPECTED_STATE[transition_id]
            except SystemFailureError:
                succeeded = False
        if not succeeded:
            raise SystemFailureError(f'Transition {label!r} of {self.node_name} failed')

    def configure(self, ctx: AbortContext | None = None) -> None:
        """``unconfigured`` -> ``inactive``."""
        self._transition(Transition.TRANSITION_CONFIGURE, ctx)

    def activate(self, ctx: AbortContext | None = None) -> None:
        """``inactive`` -> ``active``."""
        self._transition(Transition.TRANSITION_ACTIVATE, ctx)

    def deactivate(self, ctx: AbortContext | None = None) -> None:
        """``active`` -> ``inactive``."""
        self._transition(Transition.TRANSITION_DEACTIVATE, ctx)

    def cleanup(self, ctx: AbortContext | None = None) -> None:
        """``inactive`` -> ``unconfigured``."""
        self._transition(Transition.TRANSITION_CLEANUP, ctx)

    def get_parameter(self, name: str, ctx: AbortContext | None = None) -> str:
        """Read a string parameter.

        :raises SystemFailureError: If the parameter is not set, empty or not a string.
        """
        request = GetParameters.Request()
        request.names = [name]
        response = call_service(
            self._get_parameters,
            request,
            timeout_s=self._call_timeout_s,
            ctx=ctx or AbortContext.detached(),
            description=f'get_parameters of {self.node_name}',
        )
        if not response.values or response.values[0].type != ParameterType.PARAMETER_STRING:
            raise SystemFailureError(f'Parameter {name!r} of {self.node_name} is not set or not a string')
        value = response.values[0].string_value
        if not value:
            raise SystemFailureError(f'Parameter {name!r} of {self.node_name} is empty')
        return value


class LifecycleClientRegistry:
    """Caches one :class:`LifecycleClient` per node name."""

    def __init__(
        self, node: Node, *, callback_group: CallbackGroup, call_timeout_s: float, transition_timeout_s: float
    ) -> None:
        self._node = node
        self._callback_group = callback_group
        self._call_timeout_s = call_timeout_s
        self._transition_timeout_s = transition_timeout_s
        self._clients: dict[str, LifecycleClient] = {}

    def get(self, node_name: str) -> LifecycleClient:
        """The client of ``node_name``; created on first use."""
        key = fq_name(node_name)
        if key not in self._clients:
            self._clients[key] = LifecycleClient(
                self._node,
                key,
                callback_group=self._callback_group,
                call_timeout_s=self._call_timeout_s,
                transition_timeout_s=self._transition_timeout_s,
            )
        return self._clients[key]

    def destroy_all(self) -> None:
        """Destroy every client."""
        for client in self._clients.values():
            client.destroy()
        self._clients.clear()
