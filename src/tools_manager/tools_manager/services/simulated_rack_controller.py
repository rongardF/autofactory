from threading import RLock

from rclpy.lifecycle import LifecycleNode
from rclpy.subscription import Subscription
from ros_gz_interfaces.msg import Dataframe

from tools_manager.model.tool_rack_node_config_dto import ToolRackNodeConfigDTO
from tools_manager.model.slots_dto import SlotsDto
from tools_manager.model.tool_info_dto import ToolInfoDto
from tools_manager.interface.rack_controller import RackController


class SimulatedRackController(RackController):
    def __init__(
        self,
        node: LifecycleNode,
        config: ToolRackNodeConfigDTO,
    ) -> None:
        self._node = node
        self._config = config

        self._subscribers: dict[int, Subscription] = {}
        self._slot_info: dict[int, ToolInfoDto|None] = {}
        self._slot_info_lock = RLock()

    def _update_slot_info(self, index: int, data: Dataframe) -> None:
        with self._slot_info_lock:
            if len(data.data) == 0:
                # Empty payload: no tag in range, so the slot is empty.
                self._slot_info[index] = None
                return
            try:
                self._slot_info[index] = ToolInfoDto.from_tag_data(index, bytes(data.data))
            except Exception as e:
                self._node.get_logger().error(
                    f'Failed to parse RFID tag data for slot {index}: {e}'
                )
                self._slot_info[index] = None

    def _setup_sim_subscribers(self) -> None:
        for slot in self._config.slots:
            self._subscribers[slot.index] = self._node.create_subscription(
                Dataframe,
                f"/slot{slot.index}_rfid_scanner/tag_data",
                lambda msg, index=slot.index: self._update_slot_info(index, msg),  # FIXME: check type annotations
                10,
            )

    def _teardown_sim_subscribers(self) -> None:
        for sub in self._subscribers.values():
            self._node.destroy_subscription(sub)
        self._subscribers.clear()

    def setup(self) -> None:
        self._setup_sim_subscribers()
        if not self.tools_in_correct_slots():
            self._node.get_logger().warn(
                "Rack is not correctly configured. Some tools are not in their expected slots."
            )

    def teardown(self) -> None:
        self._teardown_sim_subscribers()

    def tools_in_correct_slots(self) -> bool:
        with self._slot_info_lock:
            for slot in self._config.slots:
                mounted = self._slot_info.get(slot.index)
                mounted_sn = mounted.tool_sn if mounted is not None else None
                if mounted_sn != slot.tool_sn:
                    return False
        return True

    def get_slot_index(self, tool_sn: str) -> int|None:
        for slot in self._config.slots:
            if slot.tool_sn == tool_sn:
                return slot.index
        return None

    def get_tool_info(self, index: int) -> ToolInfoDto|None:
        with self._slot_info_lock:
            return self._slot_info.get(index)

    def get_slots_data(self) -> SlotsDto:
        with self._slot_info_lock:
            slots_info = self._slot_info.copy()

        return SlotsDto(
            tools_expected=self._config.slots,
            tools_mounted=[
                tool_info for tool_info in slots_info.values() if tool_info is not None
            ],
        )