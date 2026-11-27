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
"""Per-node cache of station_utils ``NodeManager``s with this package's error semantics.

The flow code of this package raises :class:`SystemFailureError` and honours the
:class:`AbortContext` of the running goal. :class:`ManagedNode` adapts the result
objects of ``NodeManager`` to that: the abort event of the context is passed as the
``cancel_event`` so an abort ends a pending wait at once, and a failed result raises
the abort error (if any) or a :class:`SystemFailureError`.
"""

from rcl_interfaces.msg import ParameterType
from rclpy.node import Node
from station_utils.enums import LifecycleStateEnum
from station_utils.models import NodeManagerResultDTO
from station_utils.services.node_manager import NodeManager

from manufacturing_executor.exceptions import SystemFailureError
from manufacturing_executor.services.abort_context import AbortContext
from manufacturing_executor.utils.node_names import fq_name


def _raise_on_failure(result: NodeManagerResultDTO, ctx: AbortContext) -> None:
    """Raise the abort error of ``ctx``, or a SystemFailureError, if ``result`` failed."""
    if not result.success:
        ctx.raise_if_aborted()
        raise SystemFailureError(result.error_message)


class ManagedNode:
    """Reads the state and parameters of, and drives the transitions of, one node."""

    def __init__(self, manager: NodeManager) -> None:
        self._manager = manager

    @property
    def node_name(self) -> str:
        """Fully-qualified name of the node."""
        return self._manager.node_name

    def get_state(self, ctx: AbortContext | None = None) -> LifecycleStateEnum:
        """Current primary lifecycle state.

        :raises SystemFailureError: If the state cannot be read or is not a primary state.
        """
        context = ctx or AbortContext.detached()
        result = self._manager.get_state(context.event)
        _raise_on_failure(result, context)
        assert result.state is not None
        return result.state

    def configure(self, ctx: AbortContext | None = None) -> None:
        """``unconfigured`` -> ``inactive``."""
        context = ctx or AbortContext.detached()
        _raise_on_failure(self._manager.configure(context.event), context)

    def activate(self, ctx: AbortContext | None = None) -> None:
        """``inactive`` -> ``active``."""
        context = ctx or AbortContext.detached()
        _raise_on_failure(self._manager.activate(context.event), context)

    def deactivate(self, ctx: AbortContext | None = None) -> None:
        """``active`` -> ``inactive``."""
        context = ctx or AbortContext.detached()
        _raise_on_failure(self._manager.deactivate(context.event), context)

    def cleanup(self, ctx: AbortContext | None = None) -> None:
        """``inactive`` -> ``unconfigured``."""
        context = ctx or AbortContext.detached()
        _raise_on_failure(self._manager.cleanup(context.event), context)

    def get_parameter(self, name: str, ctx: AbortContext | None = None) -> str:
        """Read a string parameter.

        :raises SystemFailureError: If the parameter is not set, empty or not a string.
        """
        context = ctx or AbortContext.detached()
        result = self._manager.get_parameter(name, context.event)
        _raise_on_failure(result, context)
        value = result.value
        if value is None or value.type != ParameterType.PARAMETER_STRING:
            raise SystemFailureError(f'Parameter {name!r} of {self.node_name} is not set or not a string')
        if not value.string_value:
            raise SystemFailureError(f'Parameter {name!r} of {self.node_name} is empty')
        return value.string_value

    def destroy(self) -> None:
        """Release the underlying NodeManager."""
        self._manager.destroy()


class NodeManagerRegistry:
    """Caches one :class:`ManagedNode` per fully-qualified node name."""

    def __init__(self, node: Node, *, call_timeout_s: float, transition_timeout_s: float) -> None:
        self._node = node
        self._call_timeout_s = call_timeout_s
        self._transition_timeout_s = transition_timeout_s
        self._nodes: dict[str, ManagedNode] = {}

    def get(self, node_name: str) -> ManagedNode:
        """The managed node ``node_name``; created on first use."""
        key = fq_name(node_name)
        if key not in self._nodes:
            self._nodes[key] = ManagedNode(NodeManager(
                self._node,
                key,
                service_timeout_s=self._call_timeout_s,
                transition_timeout_s=self._transition_timeout_s,
            ))
        return self._nodes[key]

    def destroy_all(self) -> None:
        """Destroy every managed node."""
        for managed in self._nodes.values():
            managed.destroy()
        self._nodes.clear()
