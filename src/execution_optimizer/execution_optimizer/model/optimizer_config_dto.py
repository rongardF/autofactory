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

"""Configuration shared by all optimizer nodes."""

from pydantic import BaseModel, ConfigDict, Field


class OptimizerConfigDTO(BaseModel):
    """Immutable configuration shared by all optimizer variants."""

    model_config = ConfigDict(frozen=True, str_strip_whitespace=True)

    product_frame_id: str = Field(
        default='product_origin',
        min_length=1,
        description='Frame of all pattern poses and of the output capture poses.',
    )
    camera_node_name: str = Field(
        default='basler_camera',
        min_length=1,
        description='Camera node serving the get_image action; the action name is '
        '"<camera_node_name>/get_image".',
    )
    camera_info_timeout_s: float = Field(
        default=5.0,
        gt=0.0,
        description='Timeout in seconds for the camera action server and for the '
        'get_image result when the camera intrinsics are fetched.',
    )

    @property
    def get_image_action_name(self) -> str:
        """Name of the camera image action."""
        return f'{self.camera_node_name}/get_image'
