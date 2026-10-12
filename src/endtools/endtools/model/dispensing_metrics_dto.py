from pydantic import BaseModel, Field, ConfigDict


class DispensingMetricsDTO(BaseModel):
    """Data Transfer Object (DTO) for dispensing metrics.
    """
    model_config = ConfigDict(frozen=True)

    dispensed_volume_cc: float = Field(..., description="The volume dispensed in cubic centimeters.")
    dispensing_duration_s: float = Field(..., description="The duration of the dispensing operation in seconds.")
    flowrate_cc: float = Field(..., description="The flow rate during the dispensing operation in cubic centimeters per second.")