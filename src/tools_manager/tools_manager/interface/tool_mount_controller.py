from abc import ABC, abstractmethod

from tools_manager.model.tool_info_dto import ToolInfoDto


class ToolMountController(ABC):
    @abstractmethod
    def setup(self) -> None:
        """Set up the tool mount controller, including any necessary subscriptions or services."""
        raise NotImplementedError()

    @abstractmethod
    def teardown(self) -> None:
        """Tear down the tool mount controller, including any necessary cleanup of subscriptions or services."""
        raise NotImplementedError()

    @abstractmethod
    def get_mounted_tool_info(self) -> ToolInfoDto|None:
        """
        Get information about the currently mounted tool, if any.
        
        :returns: ``ToolInfoDto`` if a tool is mounted, ``None`` otherwise.
        """
        raise NotImplementedError()

    @abstractmethod
    def is_mounted(self) -> bool:
        """
        Check if a tool is currently mounted.

        :returns: ``True`` if a tool is mounted, ``False`` otherwise.
        """
        raise NotImplementedError()

    @abstractmethod
    def lock_closed(self, closed: bool) -> bool:
        """
        Lock or unlock the tool mount.

        :param closed: ``True`` to lock the mount, ``False`` to unlock.
        :returns: ``True`` if the operation was successful, ``False`` otherwise.
        """
        raise NotImplementedError()

