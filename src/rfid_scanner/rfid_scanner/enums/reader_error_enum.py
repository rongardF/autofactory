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
"""``error_Moby`` status codes reported by the SIMATIC RF200 reader."""

from enum import Enum


class ReaderErrorEnum(str, Enum):
    """Reader status codes as two-character uppercase hex strings."""

    OK = '00'
    PRESENCE_ERROR = '01'
    PARAMETER_ERROR = '05'
    AIR_INTERFACE_DISRUPTED = '06'
    MEMORY_NOT_WRITABLE = '0C'
    ADDRESS_ERROR = '0D'
    BUFFER_OVERFLOW = '13'
    SERIOUS_SYSTEM_FAULT = '14'
    RESET_PARAMETER_ERROR = '15'
    ONLY_RESET_PERMITTED = '18'
    PREVIOUS_COMMAND_ACTIVE = '19'
    COMMUNICATIONS_ERROR = '1C'
    WRONG_FRAME_LENGTH = '1E'
    CANCELLED_BY_RESET = '1F'

    @property
    def code(self) -> int:
        """Return the status code as a byte value."""
        return int(self.value, 16)

    @classmethod
    def from_code(cls, code: int) -> 'ReaderErrorEnum | None':
        """Return the member for a byte value, or ``None`` if unknown."""
        try:
            return cls(f'{code:02X}')
        except ValueError:
            return None

    @property
    def is_fatal(self) -> bool:
        """Return True if the reader is unusable until re-initialised."""
        return self in (
            ReaderErrorEnum.SERIOUS_SYSTEM_FAULT,
            ReaderErrorEnum.ONLY_RESET_PERMITTED,
            ReaderErrorEnum.COMMUNICATIONS_ERROR,
        )

    @property
    def description(self) -> str:
        """Return a human-readable description of the status code."""
        return _DESCRIPTIONS[self]


_DESCRIPTIONS: dict[ReaderErrorEnum, str] = {
    ReaderErrorEnum.OK: 'no error',
    ReaderErrorEnum.PRESENCE_ERROR:
        'presence error: tag left the field or is not present',
    ReaderErrorEnum.PARAMETER_ERROR: 'parameter error in command',
    ReaderErrorEnum.AIR_INTERFACE_DISRUPTED: 'air interface disrupted',
    ReaderErrorEnum.MEMORY_NOT_WRITABLE:
        'tag memory not writable or write protected',
    ReaderErrorEnum.ADDRESS_ERROR: 'address error',
    ReaderErrorEnum.BUFFER_OVERFLOW: 'reader buffer overflow',
    ReaderErrorEnum.SERIOUS_SYSTEM_FAULT: 'serious system fault in reader',
    ReaderErrorEnum.RESET_PARAMETER_ERROR: 'RESET parameter error',
    ReaderErrorEnum.ONLY_RESET_PERMITTED: 'only RESET permitted',
    ReaderErrorEnum.PREVIOUS_COMMAND_ACTIVE: 'previous command still active',
    ReaderErrorEnum.COMMUNICATIONS_ERROR:
        'antenna or communications error',
    ReaderErrorEnum.WRONG_FRAME_LENGTH: 'wrong frame length',
    ReaderErrorEnum.CANCELLED_BY_RESET: 'command cancelled by RESET',
}
