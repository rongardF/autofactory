from pydantic import BaseModel, Field

from tools_manager.msg import ToolInfo
from tools_manager.utils.hex_to_string import hex_to_string

from endtools.enumerator.tool_type_enum import ToolTypeEnum


class ToolInfoDto(BaseModel):
    """Information about a single tool in the rack."""

    index: int = Field(description='Slot index, value -1 indicates not installed on rack.', default=-1)
    tool_sn: str = Field(description='Serial number of the tool mounted to the slot.', default='')
    tool_type: ToolTypeEnum = Field(description='Type of the tool mounted to the slot.', default=ToolTypeEnum.UNKNOWN)
    tool_part_number: str = Field(description='Part number of the tool mounted to the slot.', default='')
    tool_part_revision: str = Field(description='Part revision of the tool mounted to the slot.', default='')
    material_part_number: str = Field(description='Part number of the material used by the tool.', default='')
    material_part_revision: str = Field(description='Part revision of the material used by the tool.', default='')

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

        The payload is a hex string whose decoded form is six fixed-width
        16-byte fields (null/space padded), in order: serial number, tool
        type, tool part number, tool part revision, material part number and
        material part revision.

        :param index: Slot index the tag was read from.
        :param tag_data: Raw hex-encoded tag payload bytes.
        :returns: The parsed tool information.
        :rtype: ToolInfoDto
        """
        field_size = 16
        decoded = hex_to_string(bytes(tag_data).decode())
        fields = [
            decoded[start:start + field_size].replace('\x00', '').strip()
            for start in range(0, field_size * 6, field_size)
        ]
        serial, tool_type, tool_pn, tool_rev, material_pn, material_rev = fields
        return ToolInfoDto(
            index=index,
            tool_sn=serial,
            tool_type=ToolTypeEnum(tool_type),
            tool_part_number=tool_pn,
            tool_part_revision=tool_rev,
            material_part_number=material_pn,
            material_part_revision=material_rev,
        )

    def to_tag_data(self) -> bytes:
        """Convert the tool information to a hex-encoded RFID tag payload.

        The payload is a hex string whose decoded form is six fixed-width
        16-byte fields (null/space padded), in order: 
            serial number
            tool_type
            tool_part_number
            tool_part_revision
            material_part_number
            material_part_revision

        :returns: The hex-encoded tag payload bytes.
        :rtype: bytes
        """
        field_size = 16
        fields = [
            self.tool_sn,
            self.tool_type.value,
            self.tool_part_number,
            self.tool_part_revision,
            self.material_part_number,
            self.material_part_revision,
        ]
        padded_fields = [f.ljust(field_size, '\x00') for f in fields]
        concatenated = ''.join(padded_fields)
        return concatenated.encode().hex().encode()

    @property
    def tool_lifted_frame(self) -> str:
        """Get the name of the frame for the tool lifted pose."""
        return f'slot{self.index}_lifted_link'

    @property
    def tool_attached_frame(self) -> str:
        """Get the name of the frame for the tool attached pose."""
        return f'slot{self.index}_attached_link'

    @property
    def tool_slide_in_frame(self) -> str:
        """Get the name of the frame for the tool slide in pose."""
        return f'slot{self.index}_slide_in_link'