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
"""Configuration of the simulated product spawning."""

from pydantic import BaseModel, ConfigDict, Field

from station_workplace.enums.constants import ROS_NAME_PATTERN


class SimulationConfigDTO(BaseModel):
    """``simulation.*`` parameters; used only when ``simulated`` is true."""

    model_config = ConfigDict(frozen=True, extra='forbid')

    world_name: str = Field(
        default='default', pattern=ROS_NAME_PATTERN,
        description='Gazebo world name used in /world/<world_name>/create/blocking and '
        '/remove/blocking.',
    )
    station_model_name: str = Field(
        default='station', pattern=ROS_NAME_PATTERN,
        description='Station model name; carrier SDF argument station_model_name.',
    )
    station_link: str = Field(
        default='carrier', pattern=ROS_NAME_PATTERN,
        description='Station link the carrier is spawned at and welded to; carrier SDF '
        'argument station_link and TF frame of the carrier spawn pose.',
    )
    carrier_model_name: str = Field(
        default='carrier_model', pattern=ROS_NAME_PATTERN,
        description='Carrier model name; carrier SDF argument model_name and product SDF '
        'argument carrier_model_name.',
    )
    carrier_link: str = Field(
        default='carrier_link', pattern=ROS_NAME_PATTERN,
        description='Carrier link the product is placed relative to and welded to; product '
        'SDF argument carrier_link.',
    )
    product_model_name: str = Field(
        default='product_model', pattern=ROS_NAME_PATTERN,
        description='Product model name; product SDF argument model_name.',
    )
    penetration_tolerance: float = Field(
        default=0.001, ge=0, allow_inf_nan=False,
        description='Product/carrier penetration depth [m] still treated as touching.',
    )

    @property
    def create_service(self) -> str:
        """Return the Gazebo world ``create/blocking`` service.

        The non-blocking ``create`` reports success once the request is queued,
        even if the entity is never created (e.g. name collision).
        """
        return f'/world/{self.world_name}/create/blocking'

    @property
    def remove_service(self) -> str:
        """Return the Gazebo world ``remove/blocking`` service."""
        return f'/world/{self.world_name}/remove/blocking'
