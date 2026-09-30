from rclpy.lifecycle import LifecycleNode

from tools_manager.interface.tool_mount_controller import ToolMountController
from tools_manager.model.tool_info_dto import ToolInfoDto


class HardwareToolMountController(ToolMountController):
    def __init__(self, node: LifecycleNode):
        self._node = node

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