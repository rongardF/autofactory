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
"""Helpers for the user supplied JSON metadata string."""

import json

from pydantic import JsonValue


def _reject_constant(name: str) -> None:
    raise ValueError(f'{name} is not valid JSON')


def parse_metadata(raw: str) -> JsonValue:
    """Parse a metadata string.

    Args:
        raw: Raw JSON text; an empty string means no metadata.

    Returns:
        JsonValue: The parsed value, or ``None`` for an empty string.

    Raises:
        ValueError: If ``raw`` is not valid JSON.
    """
    if not raw:
        return None
    try:
        return json.loads(raw, parse_constant=_reject_constant)
    except ValueError as exc:
        raise ValueError(f'metadata is not valid JSON: {exc}') from exc


def dump_metadata(value: JsonValue) -> str:
    """Serialize a stored metadata value in compact form.

    Args:
        value: The stored value; ``None`` means no metadata.

    Returns:
        str: Compact JSON text, or an empty string for ``None``.
    """
    if value is None:
        return ''
    return json.dumps(value, separators=(',', ':'), ensure_ascii=False)
