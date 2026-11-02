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

from pathlib import Path
from urllib.parse import urlparse

from pydantic import BaseModel, ConfigDict, Field, model_validator

from production_manager.utils.uuid_validator import is_uuid4


class ProductionManagerNodeConfig(BaseModel):
    model_config = ConfigDict(frozen=True)

    local_manager: bool = Field(description='Use the local assembly file backend.')
    assembly_file_path: str = Field(description='Absolute path to the local assembly JSON file.')
    station_id: str = Field(description='UUID4 identifying this production station.')
    manager_service_url: str = Field(description='Base URL of the production manager REST service.')

    @model_validator(mode='after')
    def validate_mode_configuration(self) -> 'ProductionManagerNodeConfig':
        if not is_uuid4(self.station_id):
            raise ValueError('station_id must be a valid UUID4.')
        if self.local_manager:
            path = Path(self.assembly_file_path)
            if not path.is_absolute():
                raise ValueError('assembly_file_path must be an absolute path in local manager mode.')
            if not path.is_file():
                raise ValueError(
                    f'assembly_file_path does not exist or is not a file: {self.assembly_file_path}'
                )
        else:
            parsed_url = urlparse(self.manager_service_url)
            if parsed_url.scheme not in ('http', 'https') or not parsed_url.netloc:
                raise ValueError(
                    'manager_service_url must be a valid http:// or https:// URL '
                    'in cloud backend mode.'
                )
        return self
