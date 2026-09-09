from typing import Callable
from threading import RLock

from rclpy.lifecycle import LifecycleNode
from geometry_msgs.msg import TransformStamped

from endtools.enumerators.tool_type_enum import ToolTypeEnum

from tools_manager.model.tool_rack_node_config_dto import ToolRackNodeConfigDTO
from tools_manager.model.rack_config import ToolSlotDTO
from tools_manager.model.slot_info_dto import SlotInfoDto
from tools_manager.model.slots_dto import SlotsDto
from tools_manager.model.tool_info_dto import ToolInfoDto
from tools_manager.interface.rack_controller import RackController
from tools_manager.utils.transformations import pose6_to_transform_stamped


class SimulatedRackController(RackController):
    def __init__(
        self,
        node: LifecycleNode,
        config: ToolRackNodeConfigDTO,
        callback: Callable[[SlotsDto], None]
    ) -> None:
        self._node = node
        self._config = config
        self._callback = callback

        self._slot_info_map: dict[str, ToolInfoDto|None] = self._generate_slot_info_map()
        self._slots_by_tool_sn: dict[str, ToolSlotDTO] = {
            slot.tool_sn: slot for slot in self._config.rack_config.slots
        }
        self._tools_expected: dict[str, SlotInfoDto] = {
            slot.tool_sn: SlotInfoDto(
                slot_id=slot.name,
                tool_sn=slot.tool_sn
            ) for slot in self._config.rack_config.slots
        }   

        self._callback_lock = RLock()
        self._slot_info_map_lock = RLock()

    def _generate_slot_info_map(self) -> dict[str, ToolInfoDto|None]:
        slot_info_map: dict[str, ToolInfoDto|None] = {}
        for slot_tool, sim_tool in self._config.rack_config.get_matched_tools():
            if slot_tool and sim_tool is None:
                slot_info_map[slot_tool.name] = None
            elif slot_tool and sim_tool:
                slot_info_map[slot_tool.name] = ToolInfoDto(
                    slot_id=slot_tool.name,
                    tool_sn=slot_tool.tool_sn,
                    tool_type=ToolTypeEnum(sim_tool.tool_type),
                    tool_part_number=sim_tool.tool_part_number,
                    tool_part_revision=sim_tool.tool_part_revision,
                    material_part_number=sim_tool.material_part_number,
                    material_part_revision=sim_tool.material_part_revision,
                )
            else:
                self._node.get_logger().warn(
                    f"Simulated tool has no corresponding slot in the rack configuration."
                )

        return slot_info_map

    def setup(self) -> None:
        if not self.tools_in_correct_slots():
            self._node.get_logger().warn(
                "Rack is not correctly configured. Some tools are not in their expected slots."
            )

    def teardown(self) -> None:
        pass

    def tools_in_correct_slots(self) -> bool:
        with self._slot_info_map_lock:
            slot_info_map_cache = self._slot_info_map.copy()

        for slot in self._config.rack_config.slots:
            slot_info = slot_info_map_cache[slot.name]
            if slot_info and slot.tool_sn != slot_info.tool_sn:
                return False
        return True

    def get_reserved_slot_id(self, tool_sn: str) -> str|None:
        slot_info = self._tools_expected.get(tool_sn, None)
        return slot_info.slot_id if slot_info else None

    def get_tool_info(self, slot_id: str) -> ToolInfoDto|None:
        with self._slot_info_map_lock:
            return self._slot_info_map.get(slot_id)

    def get_tool_info_by_sn(self, tool_sn: str) -> ToolInfoDto|None:
        slot = self._slots_by_tool_sn.get(tool_sn)
        if slot is None:
            self._node.get_logger().warn(
                f"No slot found for tool '{tool_sn}'; cannot get tool info."
            )
            return None

        return self.get_tool_info(slot.name)

    # FIXME: this mess below
    def get_tool_lifted_transform(self, tool_sn: str) -> TransformStamped|None:
        tool_info = self.get_tool_info_by_sn(tool_sn)
        return self._build_slot_transform(tool_info)

    def get_tool_attached_transform(self, tool_sn: str) -> TransformStamped|None:
        return self._build_slot_transform(tool_sn, 'tool_attached_pose')

    def get_tool_slide_in_transform(self, tool_sn: str) -> TransformStamped|None:
        return self._build_slot_transform(tool_sn, 'tool_slide_in_pose')

    def _build_slot_transform(self, tool_info: ToolInfoDto|None) -> TransformStamped|None:
        if tool_info is None:
            self._node.get_logger().warn(
                f"No tool info found; cannot build transform."
            )
            return None
        
        slot = self._slots_by_tool_sn.get(tool_info.slot_id)
        if slot is None:
            self._node.get_logger().warn(
                f"No slot found for tool '{tool_info.tool_sn}'; cannot build transform."
            )
            return None

        pose = getattr(slot, pose_attribute)
        return pose6_to_transform_stamped(
            parent_frame_id=self._config.parent_frame_id,
            child_frame_id=f'{slot.name}_{pose_attribute}',  # TODO: move to some DTO so it can be used in tool-mount as well
            pose=pose,
            stamp=self._node.get_clock().now().to_msg(),
        )

    def update_slot_info(self, slot_id: str, new_slot_info: ToolInfoDto|None) -> None:
        with self._slot_info_map_lock:
            old_slot_info = self._slot_info_map.get(slot_id)
            if old_slot_info == new_slot_info:
                return  # No change, do not notify callbacks
            self._slot_info_map[slot_id] = new_slot_info

        with self._callback_lock:
            self._callback(SlotsDto(
                tools_expected=list(self._tools_expected.values()),
                tools_mounted=[tool for tool in self._slot_info_map.values() if tool is not None]
            ))

    def get_current_slots_dto(self) -> SlotsDto:
        with self._slot_info_map_lock:
            return SlotsDto(
                tools_expected=list(self._tools_expected.values()),
                tools_mounted=[tool for tool in self._slot_info_map.values() if tool is not None]
            )