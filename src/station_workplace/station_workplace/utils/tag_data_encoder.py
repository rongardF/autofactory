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
"""Encoding of product data into a simulated RFID tag payload."""

from station_workplace.models.product_data_dto import ProductDataDTO
from station_workplace.models.rfid_node_config_dto import TagRegistersDTO

_FIELDS = ('product_number', 'product_revision', 'serial_number')


def encode_tag_data(product: ProductDataDTO, registers: TagRegistersDTO) -> str:
    """Encode the product data as the hex string of a simulated RFID tag memory.

    The memory is ``max(start_address + length)`` bytes long and NUL filled; each
    field's ASCII bytes are written at its register start address, NUL padded to
    the register length.

    :param product: Product data to write.
    :param registers: Tag registers (start address and length) of each field.
    :returns: The tag memory as an uppercase hex string.
    :raises ValueError: If a field is not ASCII, does not fit its register or
        two registers overlap.
    """
    size = max(
        getattr(registers, field).start_address + getattr(registers, field).length
        for field in _FIELDS)
    memory = bytearray(size)
    owner: list[str | None] = [None] * size
    for field in _FIELDS:
        register = getattr(registers, field)
        value = getattr(product, field)
        try:
            data = value.encode('ascii')
        except UnicodeEncodeError as exc:
            raise ValueError(f'{field} must be ASCII') from exc
        if len(data) > register.length:
            raise ValueError(
                f"{field} '{value}' is {len(data)} bytes long and does not fit its "
                f'{register.length}-byte tag register')
        for address in range(register.start_address, register.start_address + register.length):
            if owner[address] is not None:
                raise ValueError(f'tag registers of {owner[address]} and {field} overlap')
            owner[address] = field
        memory[register.start_address:register.start_address + len(data)] = data
    return memory.hex().upper()
