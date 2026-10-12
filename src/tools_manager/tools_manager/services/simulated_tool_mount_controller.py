from threading import RLock

from rclpy.lifecycle import LifecycleNode
from rclpy.subscription import Subscription
from ros_gz_interfaces.msg import Dataframe

from tools_manager.interface.tool_mount_controller import ToolMountController
from tools_manager.model.tool_info_dto import ToolInfoDto


class SimulatedToolMountController(ToolMountController):
    def __init__(self, node: LifecycleNode) -> None:
        self._node = node

        self._mounted_info: ToolInfoDto|None = None
        self._mounted_info_lock = RLock()

        self._tool_data_subscription: Subscription | None = None

    def _update_mounted_info(self, data: Dataframe) -> None:
        with self._mounted_info_lock:
            if len(data.data) == 0:
                # Empty payload: no tag in range, so the mount is empty.
                self._mounted_info = None
                return
            try:
                # The RFID reader plugin publishes the already hex-decoded tag
                # payload bytes, whereas from_tag_data expects the hex-text
                # representation (as stored in config). Re-encode to hex text.
                # the -1 index means that it is not on a rack
                self._mounted_info = ToolInfoDto.from_tag_data(-1, bytes(data.data).hex().encode())
            except Exception as e:
                self._node.get_logger().error(
                    f'Failed to parse RFID tag data for tool mount: {e}'
                )
                self._mounted_info = None

    def setup(self) -> None:
        self._tool_data_subscription = self._node.create_subscription(
            Dataframe,
            f"/tool_mount_rfid_scanner/tag_data",
            self._update_mounted_info,
            10,
        )

    def teardown(self) -> None:
        if self._tool_data_subscription is not None:
            self._node.destroy_subscription(self._tool_data_subscription)
            self._tool_data_subscription = None

    def get_mounted_tool_info(self) -> ToolInfoDto|None:
        with self._mounted_info_lock:
            return self._mounted_info

    def is_mounted(self) -> bool:
        with self._mounted_info_lock:
            return self._mounted_info is not None

    def lock_closed(self, closed: bool) -> bool:
        return True