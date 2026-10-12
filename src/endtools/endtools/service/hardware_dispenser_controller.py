from threading import RLock

from rclpy.lifecycle import LifecycleNode

from endtools.interface.dispenser_controller import DispenserController
from endtools.model.dispensing_metrics_dto import DispensingMetricsDTO
from endtools.model.dispensing_tool_config_dto import DispensingToolConfigDTO


class HardwareDispenserController(DispenserController):
    """Hardware controller for the volumetric dispensing tool."""

    def __init__(self, node: LifecycleNode):
        self._config: DispensingToolConfigDTO | None = None
        self._node = node

        self._is_dispensing = False
        self._lock = RLock()  # Thread-safe lock for concurrent access

    @property
    def is_dispensing(self) -> bool:
        with self._lock:
            return self._is_dispensing

    def start_dispensing(self):
        """Start the hardware dispensing operation."""
        with self._lock:
            if self._is_dispensing:
                raise RuntimeError("Dispensing is already in progress.")
            self._is_dispensing = True

        self._dispening_start_time = self._node.get_clock().now()

        # TODO: Add actual hardware control logic to start dispensing here.
        raise NotImplementedError("Hardware dispensing logic is not implemented yet.")

    def stop_dispensing(self) -> DispensingMetricsDTO:
        """Stop the hardware dispensing operation and return metrics."""
        with self._lock:
            if not self._is_dispensing:
                raise RuntimeError("Dispensing is not in progress.")
            self._is_dispensing = False

        if self._config is None:
            raise RuntimeError("Controller is not set up with a configuration.")

        dispensing_time = self._node.get_clock().now() - self._dispening_start_time

        # TODO: Add actual hardware control logic to stop dispensing here.
        raise NotImplementedError("Hardware dispensing logic is not implemented yet.")

    def setup(self, config: DispensingToolConfigDTO) -> None:
        """Set up the hardware controller with the given configuration."""
        self._config = config

        # TODO: Add actual hardware setup logic here, such as initializing GPIO pins or communication interfaces.

    def teardown(self) -> None:
        """Tear down the hardware controller."""
        self._config = None

        # TODO: Add actual hardware teardown logic here, such as releasing GPIO pins or closing communication interfaces.