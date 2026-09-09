
from tools_manager.tools_manager.interface.tool_mount_controller import ToolMountController

from tools_manager.model.tool_info_dto import ToolInfoDto


class SimulatedToolMountController(ToolMountController):
    def __init__(self, node, config):
        self._node = node
        self._config = config

    def setup(self) -> None:
        raise NotImplementedError()

    def teardown(self) -> None:
        raise NotImplementedError()

    def get_mounted_tool_info(self) -> ToolInfoDto|None:
        raise NotImplementedError()

    def is_mounted(self) -> bool:
        raise NotImplementedError()

    def lock_closed(self, closed: bool) -> bool:
        raise NotImplementedError()