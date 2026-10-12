
from uuid import UUID, uuid4

from pydantic import BaseModel, Field, ConfigDict


class LaunchDto(BaseModel):
    """Frame names for a single tool slot in the rack."""

    model_config = ConfigDict(extra='forbid')

    launch_file: str = Field(
        description='Path to the launch file used to spawn the tool model.',
        frozen=True,
    )

    launch_id: UUID = Field(
        description='Unique identifier for the launch instance.',
        default_factory=uuid4,
        frozen=True,
    )
    launch_arguments: dict[str, str|float|bool|int|list] = Field(
        description='Arguments passed to the launch file.',
        default_factory=dict,
        frozen=True,
    )
    running: bool = Field(
        description='Indicates whether the launch is currently running.',
        default=True,
        frozen=False
    )