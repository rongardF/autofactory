from pydantic import Field, field_validator

from endtools.model.tool_config_base_dto import ToolConfigBaseDTO
from endtools.enumerator.tool_type_enum import ToolTypeEnum


class DispensingToolConfigDTO(ToolConfigBaseDTO):
    """Pydantic v2 DTO for dispensing tool parameters."""

    tool_type: ToolTypeEnum = ToolTypeEnum.VOLUMETRIC_DISPENSER
    flow_rate: float = Field(
        default=1.0,
        description='Volumetric flow rate (ml/s). Used by the mock now and by real hardware later. Must be >= 0.',
    )

    @field_validator('flow_rate')
    @classmethod
    def _validate_flow_rate(cls, value: float) -> float:
        """Ensure the flow rate is non-negative."""
        if value < 0.0:
            raise ValueError(f'flow_rate must be >= 0, got {value}')
        return value