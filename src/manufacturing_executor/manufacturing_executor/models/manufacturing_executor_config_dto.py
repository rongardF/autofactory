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
"""Validated, frozen configuration of the manufacturing executor."""

from pydantic import BaseModel, ConfigDict, Field, model_validator

from manufacturing_executor.models.dependency_nodes_dto import DependencyNodesDTO


def _normalize(name: str) -> str:
    return name.strip().lstrip('/')


class ManufacturingExecutorConfigDTO(BaseModel):
    """Configuration built from the node parameters on ``configure``."""

    model_config = ConfigDict(frozen=True, str_strip_whitespace=True)

    camera_tcp: str = Field(
        default='camera_tcp_calibrated', min_length=1, description='TF frame used as tool_frame for image capture moves.'
    )
    dependency_nodes: DependencyNodesDTO = Field(
        default_factory=DependencyNodesDTO, description='Node names of all dependencies.'
    )
    execution_optimizer_nodes: tuple[str, ...] = Field(
        default=('dispensing_optimizer',), description='Optimizer nodes configured and cleaned up by this node.'
    )
    pattern_executor_nodes: tuple[str, ...] = Field(
        default=('dispensing_pattern_executor',), description='Pattern executor nodes configured and cleaned up by this node.'
    )
    service_call_timeout_s: float = Field(
        default=5.0, gt=0.0, description='Timeout of service calls and of wait_for_server / wait_for_service [s].'
    )
    image_capture_settling_s: float = Field(
        default=0.5, ge=0.0, description='Wait after each capture move before the image is taken [s].'
    )
    lifecycle_transition_timeout_s: float = Field(
        default=10.0, gt=0.0, description='Timeout of a lifecycle transition of another node [s].'
    )
    localization_result_timeout_s: float = Field(
        default=120.0, gt=0.0, description='Time allowed for the localization result [s].'
    )
    optimization_timeout_s: float = Field(
        default=60.0, gt=0.0, description='Time allowed for the pattern optimization [s].'
    )
    mount_tool_timeout_s: float = Field(
        default=300.0, gt=0.0, description='Time allowed for mounting a tool [s].'
    )
    image_capture_timeout_s: float = Field(
        default=10.0, gt=0.0, description='Time allowed for the camera get_image action and the image upload [s].'
    )
    movement_session_timeout_s: float = Field(
        default=60.0, gt=0.0, description='Requested timeout of the movement session [s].'
    )

    @model_validator(mode='after')
    def _validate_node_lists(self) -> 'ManufacturingExecutorConfigDTO':
        for label, names in (
            ('execution_optimizer_nodes', self.execution_optimizer_nodes),
            ('pattern_executor_nodes', self.pattern_executor_nodes),
        ):
            if not names:
                raise ValueError(f'{label} must not be empty')
            if any(not _normalize(name) for name in names):
                raise ValueError(f'{label} must not contain empty names')
            normalized = [_normalize(name) for name in names]
            if len(set(normalized)) != len(normalized):
                raise ValueError(f'{label} must not contain duplicates')
        optimizers = {_normalize(name) for name in self.execution_optimizer_nodes}
        executors = {_normalize(name) for name in self.pattern_executor_nodes}
        dependencies = {
            _normalize(name) for name in self.dependency_nodes.lifecycle_nodes().values()
        }
        dependencies.add(_normalize(self.dependency_nodes.localizer))
        overlap = (optimizers & executors) | ((optimizers | executors) & dependencies)
        if overlap:
            raise ValueError(f'node names are used more than once: {sorted(overlap)}')
        return self
