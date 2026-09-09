from pydantic import BaseModel, Field


class SlotInfoDto(BaseModel):
    """Information about a single slot in the rack."""

    slot_id: str = Field(description='Unique identifier for the slot.')
    tool_sn: str = Field(description='Serial number of the tool assigned to the slot.')

    def __eq__(self, value: object) -> bool:
        return (
            isinstance(value, SlotInfoDto)
            and self.slot_id == value.slot_id
            and self.tool_sn == value.tool_sn
        )