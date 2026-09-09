from abc import ABC, abstractmethod

from tools_manager.model.tool_info_dto import ToolInfoDto


class ToolMountController(ABC):
    @abstractmethod
    def setup(self) -> None:
        raise NotImplementedError()

    @abstractmethod
    def teardown(self) -> None:
        raise NotImplementedError()

    @abstractmethod
    def get_mounted_tool_info(self) -> ToolInfoDto|None:
        raise NotImplementedError()

    @abstractmethod
    def is_mounted(self) -> bool:
        raise NotImplementedError()

    @abstractmethod
    def lock_closed(self, closed: bool) -> bool:
        raise NotImplementedError()

