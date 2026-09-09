from abc import ABC, abstractmethod

from geometry_msgs.msg import TransformStamped

from tools_manager.model.tool_info_dto import ToolInfoDto


class RackController(ABC):
    @abstractmethod
    def setup(self) -> None:
        raise NotImplementedError()

    @abstractmethod
    def teardown(self) -> None:
        raise NotImplementedError()

    @abstractmethod
    def get_reserved_slot_id(self, tool_sn: str) -> str|None:
        raise NotImplementedError()

    @abstractmethod
    def get_tool_info(self, slot_id: str) -> ToolInfoDto|None:
        raise NotImplementedError()

    @abstractmethod
    def get_tool_lifted_transform(self, tool_sn: str) -> TransformStamped|None:
        raise NotImplementedError()

    @abstractmethod
    def get_tool_attached_transform(self, tool_sn: str) -> TransformStamped|None:
        raise NotImplementedError()

    @abstractmethod
    def get_tool_slide_in_transform(self, tool_sn: str) -> TransformStamped|None:
        raise NotImplementedError()

    