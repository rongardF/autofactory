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
"""Internal representation of a single cache entry.

``CacheEntryDTO`` is the wire-independent, in-memory model a backend stores and
returns. ROS messages remain the service wire format; the node translates
between the generated ``*Cache`` messages and this DTO at the service boundary.
"""

from datetime import datetime

from pydantic import BaseModel, Field

from repositories.enums.cache_value_type_enum import CacheValueTypeEnum

# Union of every payload a typed cache entry can hold, matching the six message
# value types (string, bool, number, string[], bytes, number[]).
CacheValue = str | bool | float | list[str] | list[float] | bytes


class CacheEntryDTO(BaseModel, frozen=True):
    """An immutable snapshot of a single stored cache entry."""

    key: str = Field(
        description='Unique key within the station that identifies this entry.',
    )
    value_type: CacheValueTypeEnum = Field(
        description='Discriminates which typed value is stored in this entry.',
    )
    value: CacheValue = Field(
        description='The stored payload; its Python type matches value_type.',
    )
    valid_till: datetime | None = Field(
        default=None,
        description='Absolute expiry instant; None when the entry never expires.',
    )
    expire_at_valid_till: bool = Field(
        default=False,
        description='Whether this entry participates in expiry; if False, '
        'valid_till is ignored and the entry never expires.',
    )
    stored_at: datetime = Field(
        description='When the entry was written, retained for diagnostics.',
    )
