from typing import Callable
from threading import RLock

from rclpy.lifecycle import LifecycleNode
from geometry_msgs.msg import TransformStamped

from tools_manager.model.tool_rack_node_config_dto import ToolRackNodeConfigDTO
from tools_manager.model.tool_info_dto import ToolInfoDto
from tools_manager.model.slots_dto import SlotsDto
from tools_manager.interface.rack_controller import RackController


class HardwareRackController(RackController):
    def __init__(
        self,
        node: LifecycleNode,
        config: ToolRackNodeConfigDTO,
        callback: Callable[[SlotsDto], None]
    ) -> None:
        self._node = node
        self._config = config
        self._callback = callback

        self._callback_lock = RLock()
        self._slot_info_map_lock = RLock()

    def setup(self) -> None:
        raise NotImplementedError()
    
    def teardown(self) -> None:
        raise NotImplementedError()
    
    def get_reserved_slot_id(self, tool_sn: str) -> str|None:
        raise NotImplementedError()

    def get_tool_info(self, slot_id: str) -> ToolInfoDto|None:
        raise NotImplementedError()

    def get_tool_lifted_transform(self, tool_sn: str) -> TransformStamped|None:
        raise NotImplementedError()

    def get_tool_attached_transform(self, tool_sn: str) -> TransformStamped|None:
        raise NotImplementedError()

    def get_tool_slide_in_transform(self, tool_sn: str) -> TransformStamped|None:
        raise NotImplementedError()