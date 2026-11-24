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
    delete_material_on_deactivation: bool = Field(
        default=False,
        description='Simulation only: delete the dispensed material models owned by the tool when it is deactivated.',
    )
    station_workplace_node_name: str = Field(
        default='workplace_rfid_scanner',
        description='Simulation only: node name of the station workplace whose '
        '"simulated_product_metadata" topic names the product model and link that '
        'dispensed material is welded to.',
    )
    default_tcp_valid_period: float = Field(
        default=86400.0,
        description='Default validity period (seconds) for a cached TCP value. '
        'Used as the fallback when a set_tcp request does not provide its own period.',
    )

    @field_validator('flow_rate')
    @classmethod
    def _validate_flow_rate(cls, value: float) -> float:
        """Ensure the flow rate is non-negative."""
        if value < 0.0:
            raise ValueError(f'flow_rate must be >= 0, got {value}')
        return value

    @property
    def tcp_cache_key(self) -> str:
        """Return the station_cache key under which this tool's TCP is stored."""
        return f'dispenser_{self.tool_sn}_tcp'