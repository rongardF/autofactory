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
"""Brings the dependencies of the node into their required lifecycle state."""

from collections.abc import Callable

from rclpy.node import Node

from manufacturing_executor.enums.lifecycle_state_enum import LifecycleStateEnum
from manufacturing_executor.exceptions import ActivationFailedError, ManufacturingError
from manufacturing_executor.models.manufacturing_executor_config_dto import (
    ManufacturingExecutorConfigDTO,
)
from manufacturing_executor.services.lifecycle_client import LifecycleClientRegistry


class DependencyManager:
    """Verifies the external nodes and configures the nodes this node owns.

    Owned nodes are the localizer (configured and activated), the optimizers and the
    pattern executors (configured only; the loop activates them on demand).
    """

    def __init__(
        self, node: Node, config: ManufacturingExecutorConfigDTO, registry: LifecycleClientRegistry
    ) -> None:
        self._logger = node.get_logger()
        self._config = config
        self._registry = registry

    def activate(self) -> None:
        """Verify the external nodes and bring up the owned ones.

        On failure only what was done is undone.

        :raises ActivationFailedError: If any step fails.
        """
        undo: list[Callable[[], None]] = []
        try:
            self._verify_external_nodes()
            localizer = self._registry.get(self._config.dependency_nodes.localizer)
            self._require_state(localizer.node_name, localizer.get_state(), LifecycleStateEnum.UNCONFIGURED)
            localizer.configure()
            undo.append(localizer.cleanup)
            localizer.activate()
            undo.append(localizer.deactivate)
            for name in (*self._config.execution_optimizer_nodes, *self._config.pattern_executor_nodes):
                client = self._registry.get(name)
                self._require_state(client.node_name, client.get_state(), LifecycleStateEnum.UNCONFIGURED)
                client.configure()
                undo.append(client.cleanup)
        except ManufacturingError as error:
            self._logger.error(f'Activation failed: {error}')
            for step in reversed(undo):
                try:
                    step()
                except ManufacturingError as undo_error:
                    self._logger.error(f'Rollback step failed: {undo_error}')
            raise ActivationFailedError(str(error)) from error

    def deactivate(self) -> list[str]:
        """Release the owned nodes (best effort).

        :returns: Descriptions of the steps that failed; empty on full success.
        """
        failures: list[str] = []
        owned = (
            self._config.dependency_nodes.localizer,
            *self._config.execution_optimizer_nodes,
            *self._config.pattern_executor_nodes,
        )
        for name in owned:
            client = self._registry.get(name)
            try:
                state = client.get_state()
                if state == LifecycleStateEnum.ACTIVE:
                    client.deactivate()
                    state = LifecycleStateEnum.INACTIVE
                if state == LifecycleStateEnum.INACTIVE:
                    client.cleanup()
            except ManufacturingError as error:
                self._logger.error(f'Could not release {client.node_name}: {error}')
                failures.append(f'{client.node_name}: {error}')
        return failures

    def _verify_external_nodes(self) -> None:
        for key, name in self._config.dependency_nodes.lifecycle_nodes().items():
            client = self._registry.get(name)
            state = client.get_state()
            if state != LifecycleStateEnum.ACTIVE:
                raise ActivationFailedError(
                    f'Dependency {key.value} ({client.node_name}) is {state.value}, expected active'
                )

    @staticmethod
    def _require_state(node_name: str, state: LifecycleStateEnum, expected: LifecycleStateEnum) -> None:
        if state != expected:
            raise ActivationFailedError(f'{node_name} is {state.value}, expected {expected.value}')
