# Copyright (c) 2026, Movement Controller Contributors
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from rclpy import init, shutdown
from pydantic import ValidationError
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.publisher import Publisher
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.service import Service

from tools_manager.msg import Slots
from tools_manager.srv import FillSlot, FreeSlot

from tools_manager.utils.config_reader import read_tool_rack_config_file
from tools_manager.model.slots_dto import SlotsDto
from tools_manager.model.tool_info_dto import ToolInfoDto
from tools_manager.model.tool_rack_node_config_dto import ToolRackNodeConfigDTO
from tools_manager.interface.rack_controller import RackController
from tools_manager.services.hardware_rack_controller import HardwareRackController
from tools_manager.services.simulated_rack_controller import SimulatedRackController

class ToolRack(LifecycleNode):

    def __init__(self, node_name: str = 'tool_rack') -> None:
        """Initialise the ToolRack lifecycle node.

        Declares all ROS 2 parameters with defaults and descriptions and
        creates the internal thread-safety primitives used to serialise
        concurrent goal and cancel callbacks.

        :param node_name: ROS 2 node name passed to :class:`rclpy.lifecycle.LifecycleNode`.
        :type node_name: str
        """
        super().__init__(node_name)

        self._config: ToolRackNodeConfigDTO | None = None
        self._rack_controller: RackController | None = None

        # region: parameters
        self.declare_parameter(
            'simulated',
            True,
            ParameterDescriptor(description='Whether the node is running in simulation mode'),
        )
        self.declare_parameter(
            'config_file',
            "tool_rack_config.yaml",
            ParameterDescriptor(description='Path to the tool rack configuration file'),
        )
        self.declare_parameter(
            'parent_frame_id',
            "station",
            ParameterDescriptor(description='Parent frame ID'),
        )

        self._slots_topic: Publisher | None = None
        self._fill_slot: Service | None = None
        self._free_slot: Service | None = None

        self._service_callback_group = ReentrantCallbackGroup()

    # region: lifecycle callbacks
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Configuring from state: {state.label}')
        if super().on_configure(state) != TransitionCallbackReturn.SUCCESS:
            return TransitionCallbackReturn.FAILURE

        # Read and validate constraint parameters
        try:
            rack_config = read_tool_rack_config_file(self.get_parameter('config_file').get_parameter_value().string_value)
            self._config = ToolRackNodeConfigDTO(
                rack_config=rack_config,
                simulated=self.get_parameter('simulated').get_parameter_value().bool_value,
                parent_frame_id=self.get_parameter('parent_frame_id').get_parameter_value().string_value,
            )

            self._slots_topic = self.create_lifecycle_publisher(
                Slots,
                '~/slots',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            )
            def _callback(slots: SlotsDto) -> None:
                self.get_logger().info(f"Slot info updated: {slots}")
                slots_msg = SlotsDto.to_slots_msg(slots)
                if self._slots_topic is not None:
                    self._slots_topic.publish(slots_msg)
                else:
                    self.get_logger().error("Slots topic publisher is not initialized.")

            if self._config.simulated:
                self._rack_controller = SimulatedRackController(self, self._config, _callback)
            else:
                self._rack_controller = HardwareRackController(self, self._config, _callback)
        except ValidationError as e:
            self.get_logger().error(f'Failed to generate tool rack config: {e}')
            return TransitionCallbackReturn.FAILURE
        except Exception as e:
            self.get_logger().error(f'Unknown error while configuring: {e}')
            self._rack_controller = None  # ensure rack controller is not used if config fails
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Activating from state: {state.label}')
        if super().on_activate(state) != TransitionCallbackReturn.SUCCESS:
            return TransitionCallbackReturn.FAILURE

        if self._config is None:
            self.get_logger().error('Cannot activate: configuration is not set')
            return TransitionCallbackReturn.FAILURE

        if self._slots_topic is None:
            self.get_logger().error("Slots topic publisher is not initialized.")
            return TransitionCallbackReturn.FAILURE

        # publish bootup rack state if in simulated mode; in real hardware mode
        # the rack controller will publish once established connection with the hardware
        if self._config.simulated:
            self.get_logger().info('Setting up simulated rack controller')
            if not isinstance(self._rack_controller, SimulatedRackController):
                self.get_logger().error('Rack controller is not a SimulatedRackController in simulated mode')
                return TransitionCallbackReturn.FAILURE

            self._rack_controller.setup()

            slots = self._rack_controller.get_current_slots_dto()
            slots_msg = SlotsDto.to_slots_msg(slots)
            self._slots_topic.publish(slots_msg)

            # setup services to mount and unmount tools in simulation mode
            self._fill_slot = self.create_service(
                FillSlot,
                '~/fill_slot',
                self._fill_slot_callback,
                callback_group=self._service_callback_group
            )
            self._free_slot = self.create_service(
                FreeSlot,
                '~/free_slot',
                self._free_slot_callback,
                callback_group=self._service_callback_group
            )
        else:
            self.get_logger().info('Setting up hardware rack controller')
            if not isinstance(self._rack_controller, HardwareRackController):
                self.get_logger().error('Rack controller is not a HardwareRackController in hardware mode')
                return TransitionCallbackReturn.FAILURE

            try:
                self._rack_controller.setup()
            except Exception as e:
                self.get_logger().error(f'Error during hardware rack controller setup: {e}')
                return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Deactivating from state: {state.label}')
        if super().on_deactivate(state) != TransitionCallbackReturn.SUCCESS:
            return TransitionCallbackReturn.FAILURE

        if self._rack_controller is not None:
            try:
                self._rack_controller.teardown()
            except Exception as e:
                self.get_logger().error(f'Error during rack controller teardown: {e}')
                return TransitionCallbackReturn.FAILURE

        # remove the simulation only services if they were created
        if self._config is not None and self._config.simulated:
            if self._fill_slot is not None:
                self.destroy_service(self._fill_slot)
                self._fill_slot = None
            if self._free_slot is not None:
                self.destroy_service(self._free_slot)
                self._free_slot = None

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Cleaning up from state: {state.label}')
        if super().on_cleanup(state) != TransitionCallbackReturn.SUCCESS:
            return TransitionCallbackReturn.FAILURE
        
        self._config = None
        self._rack_controller = None
        if self._slots_topic is not None:
            self.destroy_publisher(self._slots_topic)
            self._slots_topic = None

        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().error(f'Error occurred in state: {state.label}')
        return super().on_error(state)

    # endregion: lifecycle callbacks

    # region: callbacks
    def _fill_slot_callback(self, request: FillSlot.Request, response: FillSlot.Response) -> FillSlot.Response:
        if self._rack_controller is None:
            self.get_logger().error("Rack controller is not initialized.")
            response.success = False
            response.err_message = "Rack controller is not initialized."
            return response

        try:
            if not isinstance(self._rack_controller, SimulatedRackController):
                self.get_logger().error("FillSlot service is only available in simulated mode.")
                response.success = False
                response.err_message = "FillSlot service is only available in simulated mode."
                return response
            self._rack_controller.update_slot_info(request.tool.slot_id, ToolInfoDto.from_msg(request.tool))
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Error filling slot: {e}")
            response.success = False
            response.err_message = str(e)

        return response

    def _free_slot_callback(self, request: FreeSlot.Request, response: FreeSlot.Response) -> FreeSlot.Response:
        if self._rack_controller is None:
            self.get_logger().error("Rack controller is not initialized.")
            response.success = False
            response.err_message = "Rack controller is not initialized."
            return response

        try:
            if not isinstance(self._rack_controller, SimulatedRackController):
                self.get_logger().error("FreeSlot service is only available in simulated mode.")
                response.success = False
                response.err_message = "FreeSlot service is only available in simulated mode."
                return response
            self._rack_controller.update_slot_info(request.slot_id, None)
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Error freeing slot: {e}")
            response.success = False
            response.err_message = str(e)

        return response
    # endregion: callbacks


def main(args=None) -> None:
    """Entry point for the ``tool_rack`` executable.

    Initialises rclpy, creates a :class:`ToolRack` node, and spins
    it with a :class:`~rclpy.executors.MultiThreadedExecutor` (5 threads) to
    allow concurrent goal, feedback, and cancel callbacks.  Shuts down cleanly
    on exit or keyboard interrupt.

    :param args: Optional command-line arguments forwarded to :func:`rclpy.init`.
    :type args: list[str] | None
    """
    init(args=args)
    node = ToolRack()
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        shutdown()
