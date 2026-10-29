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
"""UUID4 validation helpers shared by the ``repositories`` nodes."""

import uuid


def parse_uuid4(value: str, field_name: str = 'value') -> str:
    """Validate that ``value`` is a canonical UUID4 string and normalize it.

    Only the canonical hyphenated form is accepted (case-insensitive); braces,
    ``urn:uuid:`` prefixes and un-hyphenated hex are rejected so the value is
    always safe to use as a file name or URL query value.

    Args:
        value: The raw string to validate.
        field_name: Name used in the error message.

    Returns:
        str: The lowercase canonical UUID4 string.

    Raises:
        ValueError: If ``value`` is not a canonical UUID4 string.
    """
    try:
        parsed = uuid.UUID(value)
    except (ValueError, AttributeError, TypeError) as exc:
        raise ValueError(
            f"{field_name} must be a valid UUID4 string (got '{value}')"
        ) from exc
    canonical = str(parsed)
    if canonical != value.lower():
        raise ValueError(
            f'{field_name} must be a canonical hyphenated UUID4 string '
            f"(got '{value}')"
        )
    if parsed.version != 4:
        raise ValueError(
            f'{field_name} must be a UUID4 string (got UUID version '
            f'{parsed.version})'
        )
    return canonical


def is_uuid4(value: str) -> bool:
    """Return whether ``value`` is a canonical UUID4 string.

    Args:
        value: The string to check.

    Returns:
        bool: True if :func:`parse_uuid4` accepts ``value``.
    """
    try:
        parse_uuid4(value)
    except ValueError:
        return False
    return True
