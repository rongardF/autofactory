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
"""Frozen configuration shared by all station workplace nodes."""

from pydantic import BaseModel, ConfigDict, Field

ROS_NAME_PATTERN = r'^[A-Za-z_][A-Za-z0-9_]*$'
"""Pattern of a node name used to build absolute interface names."""


class SafetyObjectConfigDTO(BaseModel):
    """Size of the safety box attached while the product is not identified."""

    model_config = ConfigDict(frozen=True, extra='forbid')

    x: float = Field(default=0.25, gt=0, description='Box size along X [m].')
    y: float = Field(default=0.4, gt=0, description='Box size along Y [m].')
    z: float = Field(default=0.3, gt=0, description='Box size along Z [m].')


class CommonNodeConfigDTO(BaseModel):
    """Parameters shared by all workplace nodes, validated in ``on_configure``."""

    model_config = ConfigDict(frozen=True, extra='forbid')

    product_carrier_node_name: str = Field(
        default='conveyor', pattern=ROS_NAME_PATTERN,
        description='Product carrier node name.',
    )
    production_manager_node_name: str = Field(
        default='production_manager', pattern=ROS_NAME_PATTERN,
        description='Production manager node name.',
    )
    object_model_repository_node_name: str = Field(
        default='object_model_storage', pattern=ROS_NAME_PATTERN,
        description='Object model storage node name.',
    )
    collision_allowed_link: str = Field(
        default='conveyor_origin', min_length=1,
        description='Robot link the carrier model and the safety object may collide with.',
    )
    safety_object: SafetyObjectConfigDTO = Field(
        default_factory=SafetyObjectConfigDTO,
        description='Safety box size.',
    )
    service_call_timeout_sec: float = Field(
        default=5.0, gt=0,
        description='Timeout for service availability and response [s].',
    )

    @property
    def product_in_place_topic(self) -> str:
        """Return the product carrier ``product_in_place`` topic."""
        return f'/{self.product_carrier_node_name}/product_in_place'

    @property
    def get_production_data_service(self) -> str:
        """Return the production manager ``get_production_data`` service."""
        return f'/{self.production_manager_node_name}/get_production_data'

    @property
    def get_models_info_service(self) -> str:
        """Return the object model storage ``get_models_info`` service."""
        return f'/{self.object_model_repository_node_name}/get_models_info'

    @property
    def get_model_service(self) -> str:
        """Return the object model storage ``get_model`` service."""
        return f'/{self.object_model_repository_node_name}/get_model'
