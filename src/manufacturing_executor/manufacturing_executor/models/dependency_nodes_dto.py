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
"""Names of the nodes the manufacturing executor depends on."""

from collections.abc import Sequence

from pydantic import BaseModel, ConfigDict, Field

from manufacturing_executor.enums.dependency_key_enum import DependencyKeyEnum


class DependencyNodesDTO(BaseModel):
    """Node names per dependency key; every key has a default."""

    model_config = ConfigDict(frozen=True, str_strip_whitespace=True)

    movement_controller: str = Field(
        default='movement_controller', min_length=1, description='Node serving execute_trajectory, in_session and close_session.'
    )
    tools_manager: str = Field(
        default='tools_manager', min_length=1, description='Node serving mount_tool and get_tool_node_name.'
    )
    image_repository: str = Field(
        default='image_repository', min_length=1, description='Node serving upload_image.'
    )
    production_manager: str = Field(
        default='production_manager', min_length=1, description='Node serving get_production_data and report_production.'
    )
    station_workplace: str = Field(
        default='workplace_rfid_scanner', min_length=1, description='Node serving wait_routing and publishing the workplace state.'
    )
    localizer: str = Field(
        default='mock_localizer', min_length=1, description='Lifecycle node serving localize_product; its lifecycle is driven by this node.'
    )
    camera: str = Field(
        default='cameras/basler_camera', min_length=1, description='Camera node (with namespace) serving get_image; not a lifecycle node.'
    )
    operation_repository: str = Field(
        default='operation_repository', min_length=1, description='Node serving get_operation.'
    )
    tool_rack: str = Field(
        default='tool_rack', min_length=1, description='Node publishing the tools on the rack.'
    )
    tool_mount: str = Field(
        default='tool_mount', min_length=1, description='Node publishing the mounted tool.'
    )

    @classmethod
    def from_parameters(cls, names: Sequence[str], values: Sequence[str]) -> 'DependencyNodesDTO':
        """Build the DTO from the parallel ``dependency_nodes.names`` / ``.values`` arrays.

        :raises ValueError: On a length mismatch, an unknown or duplicate key or an empty value.
        """
        if len(names) != len(values):
            raise ValueError(
                f'dependency_nodes.names ({len(names)}) and dependency_nodes.values ({len(values)}) '
                'must have the same length'
            )
        valid = {key.value for key in DependencyKeyEnum}
        mapping: dict[str, str] = {}
        for name, value in zip(names, values, strict=True):
            if name not in valid:
                raise ValueError(f'unknown dependency key {name!r}; valid keys: {sorted(valid)}')
            if name in mapping:
                raise ValueError(f'duplicate dependency key {name!r}')
            if not value.strip():
                raise ValueError(f'dependency key {name!r} has an empty node name')
            mapping[name] = value
        return cls(**mapping)

    def defaults_as_parameters(self) -> tuple[list[str], list[str]]:
        """Return the keys and node names as the two parallel parameter arrays."""
        return (
            [key.value for key in DependencyKeyEnum],
            [getattr(self, key.value) for key in DependencyKeyEnum],
        )

    def node_name(self, key: DependencyKeyEnum) -> str:
        """Node name of one dependency."""
        return getattr(self, key.value)

    def lifecycle_nodes(self) -> dict[DependencyKeyEnum, str]:
        """Dependencies that must already be ``active``; the localizer and the camera are excluded."""
        return {
            key: self.node_name(key)
            for key in DependencyKeyEnum
            if key not in (DependencyKeyEnum.LOCALIZER, DependencyKeyEnum.CAMERA)
        }
