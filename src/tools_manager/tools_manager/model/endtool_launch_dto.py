
from pydantic import BaseModel, Field, ConfigDict

from tools_manager.model.launch_dto import LaunchDto

from tools_manager.services.node_state_manager import NodeStateManager


class EndtoolLaunchDto(BaseModel):
    """Frame names for a single tool slot in the rack."""

    model_config = ConfigDict(extra='forbid', frozen=True)

    launch: LaunchDto = Field(
        description='Reference to launch process where node is launched.',
    )
    endtool_node_manager: NodeStateManager = Field(
        description='Node state managers for the nodes in the collection.',
    )
