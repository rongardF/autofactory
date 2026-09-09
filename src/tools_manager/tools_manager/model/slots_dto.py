from pydantic import BaseModel, Field

from tools_manager.msg import SlotInfo, ToolInfo, Slots

from tools_manager.model.slot_info_dto import SlotInfoDto
from tools_manager.model.tool_info_dto import ToolInfoDto


class SlotsDto(BaseModel):
    """Information about all slots in the rack."""

    tools_expected: list[SlotInfoDto] = Field(description='List of tools expected in the rack.')
    tools_mounted: list[ToolInfoDto] = Field(description='List of tools currently mounted in the rack.')

    def __eq__(self, value: object) -> bool:
        return (
            isinstance(value, SlotsDto)
            and self.tools_expected == value.tools_expected
            and self.tools_mounted == value.tools_mounted
        )

    @staticmethod
    def to_slots_msg(slots_dto: 'SlotsDto') -> 'Slots':
        """Convert a SlotsDto to a Slots message."""

        slots_msg = Slots()
        slots_msg.tools_expected = [
            SlotInfo(slot_id=slot.slot_id, tool_sn=slot.tool_sn) for slot in slots_dto.tools_expected
        ]
        slots_msg.tools_mounted = [
            ToolInfo(
                slot_id=tool.slot_id,
                tool_sn=tool.tool_sn,
                tool_type=tool.tool_type.value,
                tool_part_number=tool.tool_part_number,
                tool_part_revision=tool.tool_part_revision,
                material_part_number=tool.material_part_number,
                material_part_revision=tool.material_part_revision,
            )
            for tool in slots_dto.tools_mounted
        ]
        return slots_msg

    def get_tool_info(self, tool_sn: str) -> ToolInfoDto|None:
        """Get the tool info for a given tool serial number."""
        for tool in self.tools_mounted:
            if tool.tool_sn == tool_sn:
                return tool
        return None

    def get_slot_info(self, tool_sn: str) -> SlotInfoDto|None:
        """Get the slot info for a given tool serial number."""
        for slot in self.tools_expected:
            if slot.tool_sn == tool_sn:
                return slot
        return None