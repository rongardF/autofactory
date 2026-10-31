from abc import ABC, abstractmethod

from rclpy.time import Time

from endtools.model.dispensing_tool_config_dto import DispensingToolConfigDTO
from endtools.model.dispensing_metrics_dto import DispensingMetricsDTO


class DispenserController(ABC):
    """Abstract base class for dispenser controller."""

    @property
    @abstractmethod
    def is_dispensing(self) -> bool:
        """Return whether the controller is currently dispensing."""
        raise NotImplementedError()

    @abstractmethod
    def start_dispensing(self, pose_stamp: Time | None = None):
        """Start the controller.

        :param pose_stamp: Instant the TCP reached the pose that starts the
            dispensed element. Controllers that model the deposited material
            resolve the TCP pose in TF at this instant instead of sampling their
            own clock, which drifts by the caller's round-trip latency. ``None``
            means "use the node clock".
        """
        raise NotImplementedError()

    @abstractmethod
    def stop_dispensing(self, pose_stamp: Time | None = None) -> DispensingMetricsDTO:
        """Stop the controller.

        :param pose_stamp: Instant the TCP reached the pose that ends the
            dispensed element. ``None`` means "use the node clock".
        """
        raise NotImplementedError()

    @abstractmethod
    def setup(self, config: DispensingToolConfigDTO) -> None:
        """Set up the controller with the given configuration."""
        raise NotImplementedError()

    @abstractmethod
    def teardown(self) -> None:
        """Tear down the controller."""
        raise NotImplementedError()