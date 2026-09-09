from pydantic import BaseModel, Field

from tools_manager.msg import ToolInfo

from endtools.enumerators.tool_type_enum import ToolTypeEnum


class ToolInfoDto(BaseModel):
    """Information about a single tool in the rack."""

    slot_id: str = Field(description='Unique identifier for the slot.')
    tool_sn: str = Field(description='Serial number of the tool mounted to the slot.')
    tool_type: ToolTypeEnum = Field(description='Type of the tool mounted to the slot.')
    tool_part_number: str = Field(description='Part number of the tool mounted to the slot.')
    tool_part_revision: str = Field(description='Part revision of the tool mounted to the slot.')
    material_part_number: str = Field(description='Part number of the material used by the tool.')
    material_part_revision: str = Field(description='Part revision of the material used by the tool.')

    def __eq__(self, value: object) -> bool:
        return (
            isinstance(value, ToolInfoDto)
            and self.slot_id == value.slot_id
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
            slot_id=tool_info_msg.slot_id,
            tool_sn=tool_info_msg.tool_sn,
            tool_type=ToolTypeEnum(tool_info_msg.tool_type),
            tool_part_number=tool_info_msg.tool_part_number,
            tool_part_revision=tool_info_msg.tool_part_revision,
            material_part_number=tool_info_msg.material_part_number,
            material_part_revision=tool_info_msg.material_part_revision,
        )

    @property
    def tool_lifted_frame(self) -> str:
        """Get the name of the frame for the tool lifted pose."""
        return f'{self.slot_id}_tool_lifted'

    @property
    def tool_attached_frame(self) -> str:
        """Get the name of the frame for the tool attached pose."""
        return f'{self.slot_id}_tool_attached'

    @property
    def tool_slide_in_frame(self) -> str:
        """Get the name of the frame for the tool slide in pose."""
        return f'{self.slot_id}_tool_slide_in'