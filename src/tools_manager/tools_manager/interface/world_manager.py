from abc import ABC, abstractmethod

from tools_manager.model.tool_info_dto import ToolInfoDto


class WorldManager(ABC):
    @abstractmethod
    def spawn_model(self, tool: ToolInfoDto, link_name: str):
        """
        Add a tool model into the world, attached to the link specified.

        raises ModelSpawnError if the model could not be spawned.

        :param tool: Tool information.
        :param link_name: Link name to attach the tool to.
        :returns: ``True`` on success, ``False`` otherwise.
        """
        pass

    @abstractmethod
    def delete_model(self, model_id: str):
        """
        Remove a tool model from the world.

        raises ModelDeleteError if the model could not be deleted.

        :param model_id: Unique model ID (equal to the tool serial number).
        :returns: ``True`` on success, ``False`` otherwise.
        """
        pass

    @abstractmethod
    def attach_to_link(self, model_id: str, link_name: str):
        """
        Attach a tool model to a link in the world. Only single link attachment is 
        supported, so if the model is already attached to a link, it will be detached first.

        raises ModelAttachError if the model could not be attached.

        :param model_id: Unique model ID (equal to the tool serial number).
        :param link_name: Link name to attach the tool to.
        :returns: ``True`` on success, ``False`` otherwise.
        """
        pass

