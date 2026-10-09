from enum import Enum


class ToolTypeEnum(str, Enum):
    """Enumeration of valid tool types."""

    UNKNOWN = 'UNKNOWN'
    VOLUMETRIC_DISPENSER = 'VOL_DISPENSER'
