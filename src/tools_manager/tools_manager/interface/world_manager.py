from abc import ABC, abstractmethod


class WorldManager(ABC):
    @abstractmethod
    def spawn_model(self, model_id: str, model_path: str) -> bool:
        pass

    @abstractmethod
    def delete_model(self, model_id: str) -> bool:
        pass

    @abstractmethod
    def attach_to_tool_mount(self, model_id: str) -> bool:
        pass

    @abstractmethod
    def attach_to_tool_rack(self, model_id: str) -> bool:
        pass