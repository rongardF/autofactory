from enum import Enum


class ActionStateEnum(Enum):
    """Enum for action states."""

    UNMOUNTING_TOOL = "unmounting_tool"
    MOUNTING_TOOL = "mounting_tool"