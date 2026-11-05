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
"""Product identification data."""

from pydantic import BaseModel, ConfigDict, Field

from station_workplace.enums.constants import UNKNOWN
from station_workplace.msg import ProductData


class ProductDataDTO(BaseModel):
    """Identification of the product in the workplace."""

    model_config = ConfigDict(frozen=True, extra='forbid')

    product_number: str = Field(min_length=1, description='Product number.')
    product_revision: str = Field(min_length=1, description='Product revision.')
    serial_number: str = Field(min_length=1, description='Product serial number.')

    @classmethod
    def unknown(cls) -> 'ProductDataDTO':
        """Return the product data of an unidentified product."""
        return cls(product_number=UNKNOWN, product_revision=UNKNOWN, serial_number=UNKNOWN)

    def to_msg(self) -> ProductData:
        """Convert to a ``ProductData`` message."""
        return ProductData(
            product_number=self.product_number,
            product_revision=self.product_revision,
            serial_number=self.serial_number,
        )
