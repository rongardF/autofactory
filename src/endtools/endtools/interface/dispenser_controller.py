from abc import ABC, abstractmethod

from endtools.model.dispensing_tool_config_dto import DispensingToolConfigDTO
from endtools.model.dispensing_metrics_dto import DispensingMetricsDTO


class DispenserController(ABC):
    """Abstract base class for dispenser controller."""

    @property
    @abstractmethod
    def is_dispensing(self) -> bool:
        """Return whether the controller is currently dispensing."""
        pass

    @abstractmethod
    def start_dispensing(self):
        """Start the controller."""
        pass

    @abstractmethod
    def stop_dispensing(self) -> DispensingMetricsDTO:
        """Stop the controller."""
        pass

    @abstractmethod
    def setup(self, config: DispensingToolConfigDTO) -> None:
        """Set up the controller with the given configuration."""
        pass

    @abstractmethod
    def teardown(self) -> None:
        """Tear down the controller."""
        pass