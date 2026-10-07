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
from rclpy.timer import Timer
from rclpy.service import Service
from rclpy.callback_groups import ReentrantCallbackGroup

from tools_manager.msg import ToolInfo
from std_srvs.srv import SetBool

from tools_manager.exception.deactivation_failed_exception import DeactivationFailedException
from tools_manager.exception.cleanup_failed_exception import CleanupFailedException
from tools_manager.exception.tool_mount_exception import ToolMountControllerError
from tools_manager.model.tool_mount_node_config_dto import ToolMountNodeConfigDTO
from tools_manager.model.tool_info_dto import ToolInfoDto
from tools_manager.interface.tool_mount_controller import ToolMountController
from tools_manager.services.simulated_tool_mount_controller import SimulatedToolMountController
from tools_manager.services.hardware_tool_mount_controller import HardwareToolMountController


class ToolMount(LifecycleNode):

    def __init__(self, node_name: str = 'tool_mount') -> None:
        """Initialise the ToolMount lifecycle node.

        Declares all ROS 2 parameters with defaults and descriptions and
        creates the internal thread-safety primitives used to serialise
        concurrent goal and cancel callbacks.

        :param node_name: ROS 2 node name passed to :class:`rclpy.lifecycle.LifecycleNode`.
        :type node_name: str
        """
        super().__init__(node_name)

        
        self._tool_mount_controller: ToolMountController|None = None
        self._config: ToolMountNodeConfigDTO|None = None

        # region: parameters
        self.declare_parameter(
            'simulated',
            True,
            ParameterDescriptor(description='Whether the node is running in simulation mode'),
        )
        self.declare_parameter(
            'parent_frame_id',
            "tool0",
            ParameterDescriptor(description='Parent frame ID for the tool mount'),
        )
        self.declare_parameter(
            'mounted_publish_rate',
            10.0,
            ParameterDescriptor(description='Publish rate for the mounted tool information'),
        )

        # subscriptions and publishers
        self._tool_mounted_publisher: Publisher|None = None

        # services
        self._lock_service: Service|None = None

        # timers
        self._mounted_check_timer: Timer|None = None

        # callback groups
        self._service_callback_group = ReentrantCallbackGroup()
        self._publisher_callback_group = ReentrantCallbackGroup()

    # region: lifecycle callbacks
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Configuring from state: {state.label}')
        if super().on_configure(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Failed to configure base lifecycle node.')
            return TransitionCallbackReturn.FAILURE

        try:
            self._config = ToolMountNodeConfigDTO(
                simulated=self.get_parameter('simulated').get_parameter_value().bool_value,
                parent_frame_id=self.get_parameter('parent_frame_id').get_parameter_value().string_value,
                mounted_publish_rate=self.get_parameter('mounted_publish_rate').get_parameter_value().double_value,
            )

            if self._config.simulated:
                self._tool_mount_controller = SimulatedToolMountController(self)
            else:
                self._tool_mount_controller = HardwareToolMountController(self)

            self._tool_mounted_publisher = self.create_lifecycle_publisher(
                ToolInfo,
                '~/tool_mounted',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            )
        except ValidationError as e:
            self.get_logger().error(f'Failed to read tools manager config file: {e}')
            self._config = None
            self._tool_mount_controller = None
            if self._tool_mounted_publisher is not None:
                self.destroy_publisher(self._tool_mounted_publisher)
                self._tool_mounted_publisher = None
            return TransitionCallbackReturn.FAILURE
        except Exception as e:
            self.get_logger().error(f'Unknown error while configuring: {e}')
            self._config = None
            self._tool_mount_controller = None
            if self._tool_mounted_publisher is not None:
                self.destroy_publisher(self._tool_mounted_publisher)
                self._tool_mounted_publisher = None
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Activating from state: {state.label}')
        if super().on_activate(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Failed to activate base lifecycle node.')
            return TransitionCallbackReturn.FAILURE

        # sanity checks before activating the node
        if (
            self._tool_mount_controller is None or
            self._config is None or
            self._tool_mounted_publisher is None
        ):
            self.get_logger().error('Sanity check failed, something is un-initialized. Cannot activate node.')
            return TransitionCallbackReturn.FAILURE

        # initialize tool_mount controller
        try:
            self._tool_mount_controller.setup()
        except ToolMountControllerError as e:
            self.get_logger().error(f'Failed to setup tool mount controller: {e}')
            return TransitionCallbackReturn.FAILURE

        self._mounted_check_timer = self.create_timer(
            1.0 / self._config.mounted_publish_rate,
            self._check_mounted_tool,
        )

        # create services for locking/unlocking the tool mount
        self._lock_service = self.create_service(
            SetBool,
            '~/lock',
            self._lock_callback,
        )

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Deactivating from state: {state.label}')
        if super().on_deactivate(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Failed to deactivate base lifecycle node.')
            return TransitionCallbackReturn.FAILURE

        if self._tool_mount_controller is not None:
            self._tool_mount_controller.teardown()

        if self._lock_service is not None:
            self.destroy_service(self._lock_service)
            self._lock_service = None

        if self._mounted_check_timer is not None:
            self.destroy_timer(self._mounted_check_timer)
            self._mounted_check_timer = None

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Cleaning up from state: {state.label}')
        if super().on_cleanup(state) != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Failed to cleanup base lifecycle node.')
            raise CleanupFailedException('Failed to cleanup base lifecycle node.')

        if self._tool_mounted_publisher is not None:
            self.destroy_publisher(self._tool_mounted_publisher)
            self._tool_mounted_publisher = None

        self._tool_mount_controller = None
        self._config = None

        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().error(f'Error occurred in state: {state.label}')
        return super().on_error(state)

    # endregion: lifecycle callbacks

    # region: callbacks
    def _lock_callback(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        self.get_logger().info(f'Received lock update: {request}')
        
        if self._tool_mount_controller is None:
            self.get_logger().error('Tool mount controller is not initialized.')
            response.success = False
            response.message = "Tool mount controller is not initialized."
            return response
        
        if not self._tool_mount_controller.lock_closed(request.data):
            self.get_logger().error('Failed to close the lock.')
            response.success = False
            response.message = "Failed to close the lock."
            return response
        
        response.success = True
        response.message = "Lock update processed successfully."
        return response

    def _check_mounted_tool(self) -> None:
        if self._tool_mount_controller is None or self._tool_mounted_publisher is None:
            # TODO: node should transistion into error state actually - change thsi behavior
            self.get_logger().error('Tool mount controller or publisher is not initialized.')
            if self._tool_mounted_publisher is not None:
                self._tool_mounted_publisher.publish(ToolInfoDto.to_msg(ToolInfoDto()))  # publish empty ToolInfo
            return

        tool_info = self._tool_mount_controller.get_mounted_tool_info()
        if tool_info is not None:
            tool_info_msg = ToolInfoDto.to_msg(tool_info)
            self._tool_mounted_publisher.publish(tool_info_msg)
    # endregion: callbacks


def main(args=None) -> None:
    """Entry point for the ``tool_mount`` executable.

    Initialises rclpy, creates a :class:`ToolMount` node, and spins
    it with a :class:`~rclpy.executors.MultiThreadedExecutor` (3 threads) to
    allow concurrent goal, feedback, and cancel callbacks.  Shuts down cleanly
    on exit or keyboard interrupt.

    :param args: Optional command-line arguments forwarded to :func:`rclpy.init`.
    :type args: list[str] | None
    """
    init(args=args)
    node = ToolMount()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        shutdown()
