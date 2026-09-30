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
from rclpy.timer import Timer

from tools_manager.msg import Slots

from tools_manager.exception.deactivation_failed_exception import DeactivationFailedException
from tools_manager.exception.cleanup_failed_exception import CleanupFailedException
from tools_manager.model.slots_dto import SlotsDto
from tools_manager.model.tool_rack_node_config_dto import ToolRackNodeConfigDTO
from tools_manager.interface.rack_controller import RackController
from tools_manager.services.hardware_rack_controller import HardwareRackController
from tools_manager.services.simulated_rack_controller import SimulatedRackController

from tools_manager.utils.config_reader import read_config_file


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
            'tools_manager_config_file',
            "tools_manager_config.yaml",
            ParameterDescriptor(description='Path to the tool rack configuration file'),
        )
        self.declare_parameter(
            'parent_frame_id',
            "station",
            ParameterDescriptor(description='Parent frame ID'),
        )
        self.declare_parameter(
            'slots_update_rate',
            10.0,
            ParameterDescriptor(description='Rate at which to update slots (in Hz)'),
        )

        self._slots_topic: Publisher | None = None
        self._slots_update_timer: Timer | None = None

        self._service_callback_group = ReentrantCallbackGroup()

    # region: lifecycle callbacks
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Configuring from state: {state.label}')
        if super().on_configure(state) != TransitionCallbackReturn.SUCCESS:
            return TransitionCallbackReturn.FAILURE

        # Read and validate constraint parameters
        try:
            config = read_config_file(self.get_parameter('tools_manager_config_file').get_parameter_value().string_value)
            self._config = ToolRackNodeConfigDTO(
                slots=config.slots,
                simulated=self.get_parameter('simulated').get_parameter_value().bool_value,
                parent_frame_id=self.get_parameter('parent_frame_id').get_parameter_value().string_value,
                slots_update_rate=self.get_parameter('slots_update_rate').get_parameter_value().double_value,
            )

            self._slots_topic = self.create_lifecycle_publisher(
                Slots,
                '~/slots',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            )
            
            if self._config.simulated:
                self._rack_controller = SimulatedRackController(self, self._config)
            else:
                self._rack_controller = HardwareRackController(self, self._config)
        except ValidationError as e:
            self.get_logger().error(f'Failed to process/generate config: {e}')
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

        if self._rack_controller is None:
            self.get_logger().error("Rack controller is not initialized.")
            return TransitionCallbackReturn.FAILURE

        self._rack_controller.setup()

        self._slots_update_timer = self.create_timer(
            1.0 / self._config.slots_update_rate,
            self._update_slots
        )

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Deactivating from state: {state.label}')
        if super().on_deactivate(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Failed to deactivate node: super().on_deactivate() returned FAILURE')
            raise DeactivationFailedException('Failed to deactivate node: super().on_deactivate() returned FAILURE')

        if self._rack_controller is not None:
            self._rack_controller.teardown()

        # remove the simulation only services if they were created
        if self._slots_update_timer is not None:
            self.destroy_timer(self._slots_update_timer)
            self._slots_update_timer = None

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Cleaning up from state: {state.label}')
        if super().on_cleanup(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Failed to cleanup node: super().on_cleanup() returned FAILURE')
            raise CleanupFailedException('Failed to cleanup node: super().on_cleanup() returned FAILURE')
        
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
    def _update_slots(self) -> None:
        if self._rack_controller is None:
            self.get_logger().error("Rack controller is not initialized.")
            # TODO: node should transistion into error state
            return
        if self._slots_topic is None:
            self.get_logger().error("Slots topic publisher is not initialized.")
            # TODO: node should transistion into error state
            return

        slots = self._rack_controller.get_slots_data()
        slots_msg = SlotsDto.to_slots_msg(slots)
        self._slots_topic.publish(slots_msg)
    # endregion: callbacks


def main(args=None) -> None:
    """Entry point for the ``tool_rack`` executable.

    Initialises rclpy, creates a :class:`ToolRack` node, and spins
    it with a :class:`~rclpy.executors.MultiThreadedExecutor` (3 threads) to
    allow concurrent goal, feedback, and cancel callbacks.  Shuts down cleanly
    on exit or keyboard interrupt.

    :param args: Optional command-line arguments forwarded to :func:`rclpy.init`.
    :type args: list[str] | None
    """
    init(args=args)
    node = ToolRack()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        shutdown()
