from threading import RLock

from rclpy.lifecycle import LifecycleNode

from endtools.interface.dispenser_controller import DispenserController
from endtools.model.dispensing_metrics_dto import DispensingMetricsDTO
from endtools.model.dispensing_tool_config_dto import DispensingToolConfigDTO


class SimulatedDispenserController(DispenserController):
    """Simulated controller for the volumetric dispensing tool."""

    def __init__(self, node: LifecycleNode):
        self._config: DispensingToolConfigDTO | None = None
        self._node = node

        self._is_dispensing = False
        self._lock = RLock()  # Thread-safe lock for concurrent access

    @property
    def is_dispensing(self) -> bool:
        with self._lock:
            return self._is_dispensing

    @is_dispensing.setter
    def is_dispensing(self, value: bool):
        with self._lock:
            self._is_dispensing = value

    def start_dispensing(self):
        """Start the hardware dispensing operation."""
        with self._lock:
            if self.is_dispensing:
                raise RuntimeError("Dispensing is already in progress.")
            self.is_dispensing = True

        self._dispening_start_time = self._node.get_clock().now()

    def stop_dispensing(self) -> DispensingMetricsDTO:
        """Stop the hardware dispensing operation and return metrics."""
        with self._lock:
            if not self.is_dispensing:
                raise RuntimeError("Dispensing is not in progress.")
            self.is_dispensing = False

        if self._config is None:
            raise RuntimeError("Controller is not set up with a configuration.")

        dispensing_time = self._node.get_clock().now() - self._dispening_start_time
        dispensed_volume = self._config.flow_rate * dispensing_time.nanoseconds / 1e9  # Convert nanoseconds to seconds

        return DispensingMetricsDTO(
            dispensed_volume_cc=dispensed_volume,
            dispensing_duration_s=dispensing_time.nanoseconds / 1e9,
            flowrate_cc=self._config.flow_rate,
        )

    def setup(self, config: DispensingToolConfigDTO) -> None:
        """Set up the hardware controller with the given configuration."""
        self._config = config

    def teardown(self) -> None:
        """Tear down the hardware controller."""
        self._config = None