from threading import RLock

from rclpy.lifecycle import LifecycleNode

from tools_manager.model.tool_rack_node_config_dto import ToolRackNodeConfigDTO
from tools_manager.model.tool_info_dto import ToolInfoDto
from tools_manager.model.slots_dto import SlotsDto
from tools_manager.interface.rack_controller import RackController


class HardwareRackController(RackController):
    def __init__(
        self,
        node: LifecycleNode,
        config: ToolRackNodeConfigDTO,
    ) -> None:
        self._node = node
        self._config = config

        self._callback_lock = RLock()
        self._slot_info_map_lock = RLock()

    def setup(self) -> None:
        raise NotImplementedError()
    
    def teardown(self) -> None:
        raise NotImplementedError()
    
    def get_slot_index(self, tool_sn: str) -> int|None:
        raise NotImplementedError()

    def get_tool_info(self, index: int) -> ToolInfoDto|None:
        raise NotImplementedError()

    def get_slots_data(self) -> SlotsDto:
        raise NotImplementedError()