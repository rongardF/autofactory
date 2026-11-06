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
"""Decoding of raw product data bytes into text."""

import string

from station_workplace.exceptions.injection_error import InjectionError

_TRAILING_PADDING = '\x00' + string.whitespace


def decode_label(data: bytes, description: str = 'label') -> str:
    """Decode bytes as strict ASCII.

    :param data: Raw bytes (any bytes-like object, e.g. ``array.array('B')``).
    :param description: Name of the data, used in error messages.
    :returns: The decoded text.
    :raises InjectionError: If the bytes are not ASCII.
    """
    try:
        return bytes(data).decode('ascii')
    except UnicodeDecodeError as exc:
        raise InjectionError(f'{description} contains non-ASCII bytes: {exc}') from exc


def decode_rfid_field(data: bytes, description: str = 'field') -> str:
    """Decode an RFID tag field as ASCII without trailing NUL/whitespace padding.

    Leading whitespace is stripped as well.

    :param data: Raw bytes read from the tag.
    :param description: Name of the field, used in error messages.
    :returns: The decoded, non-empty text.
    :raises InjectionError: If the bytes are not ASCII or the result is empty.
    """
    text = decode_label(data, description).rstrip(_TRAILING_PADDING).strip()
    if not text:
        raise InjectionError(f'{description} is empty')
    return text
