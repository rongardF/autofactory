# Copyright (c) 2026, Movement Controller Contributors
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
"""Device-agnostic configuration envelope for the GPIO controller.

The envelope carries only the fields the base loader understands. The opaque
``device`` block is validated by the selected driver's own config DTO (e.g.
``URDeviceConfigDTO``), which also owns ``io_name`` uniqueness because it is the
only component that knows where names live inside the mapping.
"""

from pydantic import BaseModel, Field

from io_controllers.enums.device_type_enum import DeviceTypeEnum


class IOControllerConfigDTO(BaseModel, frozen=True):
    """Top-level, device-agnostic configuration for the GPIO controller node."""

    device_type: DeviceTypeEnum = Field(
        description='Selects which concrete driver the factory instantiates.',
    )
    publish_rate_hz: float = Field(
        default=10.0,
        gt=0.0,
        description='Fixed rate at which full io_state snapshots are published.',
    )
    device: dict = Field(
        description='Raw driver-specific configuration block, validated by the '
        'selected driver rather than the base loader.',
    )
