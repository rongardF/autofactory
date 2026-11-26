from pydantic import BaseModel, Field

from tools_manager.msg import ToolInfo

from endtools.enumerator.tool_type_enum import ToolTypeEnum


class ToolInfoDto(BaseModel):
    """Information about a single tool in the rack."""

    index: int = Field(description='Slot index, value -1 indicates not installed on rack.', default=-1)
    tool_sn: str = Field(description='Serial number of the tool mounted to the slot.', default='')
    tool_type: ToolTypeEnum = Field(description='Type of the tool mounted to the slot.', default=ToolTypeEnum.UNKNOWN)
    tool_part_number: str = Field(description='Part number of the tool mounted to the slot.', default='')
    tool_part_revision: str = Field(description='Part revision of the tool mounted to the slot.', default='')
    material_part_number: str = Field(
        description="Product number of the material used by the tool; filled from the endtool's material parameters, not from the RFID tag.",
        default='',
    )
    material_part_revision: str = Field(
        description="Product revision of the material used by the tool; filled from the endtool's material parameters, not from the RFID tag.",
        default='',
    )

    def __eq__(self, value: object) -> bool:
        return (
            isinstance(value, ToolInfoDto)
            and self.index == value.index
            and self.tool_sn == value.tool_sn
            and self.tool_type == value.tool_type
            and self.tool_part_number == value.tool_part_number
            and self.tool_part_revision == value.tool_part_revision
            and self.material_part_number == value.material_part_number
            and self.material_part_revision == value.material_part_revision
        )

    @staticmethod
    def from_msg(tool_info_msg: 'ToolInfo') -> 'ToolInfoDto':
        """Convert a ToolInfo message to a ToolInfoDto."""
        return ToolInfoDto(
            index=tool_info_msg.index,
            tool_sn=tool_info_msg.tool_sn,
            tool_type=ToolTypeEnum(tool_info_msg.tool_type),
            tool_part_number=tool_info_msg.tool_part_number,
            tool_part_revision=tool_info_msg.tool_part_revision,
            material_part_number=tool_info_msg.material_part_number,
            material_part_revision=tool_info_msg.material_part_revision,
        )

    @staticmethod
    def to_msg(tool_info_dto: 'ToolInfoDto') -> 'ToolInfo':
        """Convert a ToolInfoDto to a ToolInfo message."""
        return ToolInfo(
            index=tool_info_dto.index,
            tool_sn=tool_info_dto.tool_sn,
            tool_type=tool_info_dto.tool_type.value,
            tool_part_number=tool_info_dto.tool_part_number,
            tool_part_revision=tool_info_dto.tool_part_revision,
            material_part_number=tool_info_dto.material_part_number,
            material_part_revision=tool_info_dto.material_part_revision,
        )

    @staticmethod
    def from_tag_data(index: int, tag_data: bytes) -> 'ToolInfoDto':
        """Build a ToolInfoDto from a hex-encoded RFID tag payload.

        :param index: Slot index the tag was read from.
        :param tag_data: Hex-encoded tag payload as ASCII bytes.
        :returns: The parsed tool information.
        :raises ValueError: If the payload is not valid hex or not a valid tool tag.
        """
        try:
            hex_text = bytes(tag_data).decode('ascii')
        except UnicodeDecodeError as error:
            raise ValueError(f'RFID tag payload is not hex-encoded text: {error}') from error
        return ToolInfoDto.from_hex_tag_data(index, hex_text)

    @staticmethod
    def from_hex_tag_data(index: int, hex_text: str) -> 'ToolInfoDto':
        """Build a ToolInfoDto from a hex-encoded RFID tag payload string.

        This is the single decoding and validation path for RFID payloads. The
        decoded payload holds four fixed-width 16-byte fields (null/space padded),
        in order: serial number, tool type, tool part number and tool part revision.
        The material is not stored on the tag: ``material_part_number`` and
        ``material_part_revision`` stay empty here and are filled in by the nodes
        that publish the tool information.

        :param index: Slot index the tag was read from (``-1`` for the tool mount).
        :param hex_text: Hex-encoded tag payload.
        :returns: The parsed tool information.
        :raises ValueError: If the payload is not valid hex, is shorter than the four
            fields, is not text, has an empty serial number or has an unknown tool type.
        """
        field_size = 16
        field_count = 4
        try:
            raw = bytes.fromhex(hex_text)
        except ValueError as error:
            raise ValueError(f'RFID tag payload is not valid hex: {error}') from error
        if len(raw) < field_size * field_count:
            raise ValueError(
                f'RFID tag payload has {len(raw)} bytes, expected at least {field_size * field_count}'
            )

        try:
            fields = [
                raw[start:start + field_size].decode('utf-8').replace('\x00', '').strip()
                for start in range(0, field_size * field_count, field_size)
            ]
        except UnicodeDecodeError as error:
            raise ValueError(f'RFID tag payload fields are not valid text: {error}') from error
        serial, tool_type, tool_pn, tool_rev = fields
        if not serial:
            raise ValueError('RFID tag payload has an empty serial number')
        return ToolInfoDto(
            index=index,
            tool_sn=serial,
            tool_type=ToolTypeEnum(tool_type),
            tool_part_number=tool_pn,
            tool_part_revision=tool_rev,
        )

    def to_tag_data(self) -> bytes:
        """Convert the tool information to a hex-encoded RFID tag payload.

        The payload is a hex string whose decoded form is four fixed-width
        16-byte fields (null/space padded), in order: 
            serial number
            tool_type
            tool_part_number
            tool_part_revision

        The material is not part of the tag.

        :returns: The hex-encoded tag payload bytes.
        :rtype: bytes
        """
        field_size = 16
        fields = [
            self.tool_sn,
            self.tool_type.value,
            self.tool_part_number,
            self.tool_part_revision,
        ]
        padded_fields = [f.ljust(field_size, '\x00') for f in fields]
        concatenated = ''.join(padded_fields)
        return concatenated.encode().hex().encode()

    @property
    def tool_lifted_frame(self) -> str:
        """Get the name of the frame for the tool lifted pose."""
        if self.index < 0:
            raise ValueError("Tool is not assigned to a valid slot.")
        return f'slot{self.index}_lifted_link'

    @property
    def tool_attached_frame(self) -> str:
        """Get the name of the frame for the tool attached pose."""
        if self.index < 0:
            raise ValueError("Tool is not assigned to a valid slot.")
        return f'slot{self.index}_attached_link'

    @property
    def tool_slide_in_frame(self) -> str:
        """Get the name of the frame for the tool slide in pose."""
        if self.index < 0:
            raise ValueError("Tool is not assigned to a valid slot.")
        return f'slot{self.index}_slide_in_link'