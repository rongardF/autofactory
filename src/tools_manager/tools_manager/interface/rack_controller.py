from abc import ABC, abstractmethod

from tools_manager.model.tool_info_dto import ToolInfoDto
from tools_manager.model.slots_dto import SlotsDto


class RackController(ABC):
    @abstractmethod
    def setup(self) -> None:
        raise NotImplementedError()

    @abstractmethod
    def teardown(self) -> None:
        raise NotImplementedError()

    @abstractmethod
    def get_slot_index(self, tool_sn: str) -> int|None:
        raise NotImplementedError()

    @abstractmethod
    def get_tool_info(self, index: int) -> ToolInfoDto|None:
        raise NotImplementedError()

    @abstractmethod
    def get_slots_data(self) -> SlotsDto:
        raise NotImplementedError()

    