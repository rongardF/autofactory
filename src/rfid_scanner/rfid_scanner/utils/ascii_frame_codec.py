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
"""Encoder/decoder for the SIMATIC RF200 ASCII protocol framing.

A frame is ``STX`` + net data + optional ``LF`` + ``ETX``. Every net byte is
sent as two uppercase hex ASCII characters, high nibble first. Net byte 0 is
the frame length (net length - 1), byte 1 the command and byte 2 the status.
"""

from typing import Final

from rfid_scanner.enums.ascii_command_enum import AsciiCommandEnum
from rfid_scanner.exceptions.frame_error import FrameError
from rfid_scanner.models.ascii_frame_dto import AsciiFrameDTO

STX: Final[int] = 0x02
ETX: Final[int] = 0x03
LF: Final[int] = 0x0A
MAX_TRANSFER_BYTES: Final[int] = 248
MAX_FRAME_CHARS: Final[int] = 2 * 256

_HEX_CHARS: Final[frozenset[int]] = frozenset(b'0123456789ABCDEF')


def encode_frame(command: AsciiCommandEnum, params: bytes = b'') -> bytes:
    """Return the complete frame (STX..ETX) for a host command."""
    net = bytes([0, command.code, 0x00]) + params
    length = len(net) - 1
    if length > 0xFF:
        raise ValueError(f'frame too long: {length} net bytes')
    net = bytes([length]) + net[1:]
    return bytes([STX]) + net.hex().upper().encode('ascii') + bytes([ETX])


def decode_frame(payload: bytes) -> AsciiFrameDTO:
    """Decode the characters between STX and ETX into an AsciiFrameDTO."""
    if payload.endswith(bytes([LF])):
        payload = payload[:-1]
    if len(payload) % 2 != 0:
        raise FrameError(f'odd number of hex characters in frame: {payload!r}')
    if any(c not in _HEX_CHARS for c in payload):
        raise FrameError(f'invalid hex character in frame: {payload!r}')
    net = bytes.fromhex(payload.decode('ascii'))
    if len(net) < 3:
        raise FrameError(f'frame too short: {payload!r}')
    if net[0] != len(net) - 1:
        raise FrameError(
            f'frame length byte {net[0]} does not match net length {len(net)}')
    return AsciiFrameDTO(command=net[1], status=net[2], body=net[3:])


class FrameAssembler:
    """Splits a received byte stream into frame payloads."""

    def __init__(self) -> None:
        """Start with no partial frame."""
        self._buffer = bytearray()
        self._in_frame = False

    def reset(self) -> None:
        """Drop any partial frame."""
        self._buffer.clear()
        self._in_frame = False

    def feed(self, chunk: bytes) -> list[bytes]:
        """Consume received bytes and return all completed frame payloads."""
        frames: list[bytes] = []
        for byte in chunk:
            if not self._in_frame:
                if byte != STX:
                    raise FrameError(f'unexpected byte 0x{byte:02X} outside frame')
                self._in_frame = True
                self._buffer.clear()
            elif byte == STX:
                self.reset()
                raise FrameError('STX inside frame')
            elif byte == ETX:
                frames.append(bytes(self._buffer))
                self.reset()
            else:
                self._buffer.append(byte)
                if len(self._buffer) > MAX_FRAME_CHARS:
                    self.reset()
                    raise FrameError('receive buffer overflow')
        return frames
