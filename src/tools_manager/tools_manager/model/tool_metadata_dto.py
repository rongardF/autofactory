from pydantic import BaseModel, Field, ConfigDict


class ToolMetadataDto(BaseModel):
    """Endtool metadata info (launch and model)."""
    model_config = ConfigDict(extra='forbid', frozen=True)

    launch_file: str = Field(description='Launch file name for the tool.')
    model: str = Field(description='Model name (root directory) for the tool.')