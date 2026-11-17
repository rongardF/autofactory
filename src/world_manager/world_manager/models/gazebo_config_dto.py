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
"""Configuration of the gazebo_world_manager node."""

from pydantic import BaseModel, ConfigDict, Field


class GazeboConfigDTO(BaseModel):
    """Parameters of ``gazebo_world_manager``."""

    model_config = ConfigDict(frozen=True)

    world_name: str = Field(min_length=1, description='Name of the Gazebo world (the <world name> in the SDF)')
    world_frame: str = Field(default='world', description='TF frame that coincides with the Gazebo world origin')
    service_timeout_sec: float = Field(default=5.0, gt=0, description='Timeout of a single gz-transport request')
    tf_timeout_sec: float = Field(default=3.0, gt=0, description='Timeout of a TF lookup')
    confirm_timeout_sec: float = Field(
        default=5.0, gt=0, description='Time to wait for a joint state event after an attach/detach request')
    republish_period_sec: float = Field(
        default=0.25, gt=0, description='Period at which an unconfirmed attach/detach request is republished')
    operation_timeout_sec: float = Field(
        default=10.0, gt=0, description='Total forward deadline of one mutating service call')
    rollback_timeout_sec: float = Field(
        default=1.0, gt=0, description='Per-joint confirmation timeout used while rolling back or detaching before removal')
