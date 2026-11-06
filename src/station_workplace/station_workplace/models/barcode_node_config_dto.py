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
"""Frozen configuration of the ``workplace_barcode_scanner`` node."""

import re
from typing import Any

from pydantic import BaseModel, ConfigDict, Field, field_validator

from station_workplace.models.common_node_config_dto import CommonNodeConfigDTO, ROS_NAME_PATTERN


class LabelPatternsDTO(BaseModel):
    """Regexes extracting the product data fields from a barcode label."""

    model_config = ConfigDict(frozen=True, extra='forbid', arbitrary_types_allowed=True)

    product_number: re.Pattern = Field(
        description='Regex with one capture group extracting the product number.')
    product_revision: re.Pattern = Field(
        description='Regex with one capture group extracting the product revision.')
    serial_number: re.Pattern = Field(
        description='Regex with one capture group extracting the serial number.')

    @field_validator('product_number', 'product_revision', 'serial_number', mode='before')
    @classmethod
    def _compile(cls, value: Any) -> re.Pattern:
        """Compile the pattern and require exactly one capture group."""
        if isinstance(value, re.Pattern):
            pattern = value
        else:
            if not isinstance(value, str) or not value:
                raise ValueError('pattern must be a non-empty string')
            try:
                pattern = re.compile(value)
            except re.error as exc:
                raise ValueError(f'invalid regex: {exc}') from exc
        if pattern.groups != 1:
            raise ValueError(
                f'pattern must have exactly one capture group, has {pattern.groups}')
        return pattern


class BarcodeNodeConfigDTO(CommonNodeConfigDTO):
    """Parameters of the ``workplace_barcode_scanner`` node."""

    barcode_scanner_node_name: str = Field(
        default='barcode_scanner', pattern=ROS_NAME_PATTERN,
        description='Barcode scanner node name.',
    )
    label_patterns: LabelPatternsDTO = Field(
        description='Regexes extracting the product data fields from the label.',
    )

    @property
    def label_data_topic(self) -> str:
        """Return the barcode scanner ``label_data`` topic."""
        return f'/{self.barcode_scanner_node_name}/label_data'

    @property
    def simulated_label_data_topic(self) -> str:
        """Return the barcode scanner ``simulated_label_data`` topic."""
        return f'/{self.barcode_scanner_node_name}/simulated_label_data'
