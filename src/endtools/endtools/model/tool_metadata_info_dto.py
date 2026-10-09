from pydantic import BaseModel, Field, ConfigDict


class ToolMetadataInfoDTO(BaseModel):
    """Pydantic v2 DTO for tool metadata information."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    tool_ref: str = Field(
        ...,
        description="The reference of the tool, e.g., 'volumetric_dispenser'.",
    )
    launch_file: str = Field(
        ...,
        description="The name of the launch file associated with the tool.",
    )
    model_path: str = Field(
        ...,
        description="The path to the model assets for the tool.",
    )