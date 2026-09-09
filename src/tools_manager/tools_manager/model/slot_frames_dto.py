from pydantic import BaseModel, Field, ConfigDict


class SlotFramesDto(BaseModel):
    """Frame names for a single tool slot in the rack."""

    model_config = ConfigDict(extra='forbid')

    tool_slide_in_frame: str = Field(description='Name of the frame for the tool slide in pose.')
    tool_attached_frame: str = Field(description='Name of the frame for the tool attached pose.')
    tool_lifted_frame: str = Field(description='Name of the frame for the tool lifted pose.')