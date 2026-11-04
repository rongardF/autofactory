# Copyright (c) 2026, Autofactory
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

from threading import Lock
from typing import Callable

from pydantic import ValidationError
from rcl_interfaces.msg import ParameterDescriptor
from rclpy import init, ok, shutdown
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.publisher import Publisher
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.service import Service
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from product_carrier.interface import CarrierController
from product_carrier.model import CarrierActionResultDTO, ConveyorNodeConfigDTO
from product_carrier.services import HardwareConveyorController, SimulatedConveyorController


class Conveyor(LifecycleNode):
    """Lifecycle node controlling a conveyor line that feeds the robot workplace.

    Exposes the carrier agnostic interface:

    * ``~/inject_product`` (``std_srvs/Trigger``) - move a new product into the workplace.
    * ``~/eject_product`` (``std_srvs/Trigger``) - move the product out of the workplace.
    * ``~/product_in_place`` (``std_msgs/Bool``, latched) - product presence in the workplace.

    The node never triggers its own lifecycle transitions.
    """

    def __init__(self, node_name: str = 'conveyor') -> None:
        """Declare parameters; no ROS interface is created until activation.

        :param node_name: ROS 2 node name.
        """
        super().__init__(node_name)

        self.declare_parameter(
            'simulated',
            True,
            ParameterDescriptor(description='Use the simulated (Gazebo) backend instead of real hardware'),
        )

        self._config: ConveyorNodeConfigDTO | None = None
        self._controller: CarrierController | None = None
        self._controller_ready: bool = False

        self._product_in_place_pub: Publisher | None = None
        self._inject_srv: Service | None = None
        self._eject_srv: Service | None = None

        self._publish_lock: Lock = Lock()
        self._service_callback_group: ReentrantCallbackGroup = ReentrantCallbackGroup()

    # region: lifecycle callbacks
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Configuring from state: {state.label}')
        if super().on_configure(state) != TransitionCallbackReturn.SUCCESS:
            return TransitionCallbackReturn.FAILURE

        try:
            self._config = ConveyorNodeConfigDTO(
                simulated=self.get_parameter('simulated').get_parameter_value().bool_value,
            )
            if self._config.simulated:
                self._controller = SimulatedConveyorController(self, self._config)
            else:
                self._controller = HardwareConveyorController(self, self._config)
        except ValidationError as e:
            self.get_logger().error(f'Invalid configuration: {e}')
            self._release_all()
            return TransitionCallbackReturn.FAILURE
        except Exception as e:
            self.get_logger().error(f'Configuration failed: {e}')
            self._release_all()
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info(f'Configured (simulated={self._config.simulated})')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Activating from state: {state.label}')
        if self._config is None or self._controller is None:
            self.get_logger().error('Cannot activate: node is not configured')
            return TransitionCallbackReturn.FAILURE

        try:
            # Publisher first so no presence change can arrive before it exists.
            self._create_interface_publisher()
            self._controller.setup(self._on_presence_changed)
            self._controller_ready = True
            self._create_interface_services()
            if super().on_activate(state) != TransitionCallbackReturn.SUCCESS:
                raise RuntimeError('Base class activation failed')
        except Exception as e:
            self.get_logger().error(f'Activation failed: {e}')
            self._destroy_interface()
            self._teardown_controller()
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info('Activated')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Deactivating from state: {state.label}')
        self._destroy_interface()
        self._teardown_controller()
        result = super().on_deactivate(state)
        if result != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Deactivation failed: base class deactivation did not succeed')
        return result

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Cleaning up from state: {state.label}')
        self._config = None
        self._controller = None
        result = super().on_cleanup(state)
        if result != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error('Cleanup failed: base class cleanup did not succeed')
        return result

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Shutting down from state: {state.label}')
        self._release_all()
        super().on_shutdown(state)
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().error(f'Error occurred in state: {state.label}; releasing all resources')
        self._release_all()
        super().on_error(state)
        return TransitionCallbackReturn.SUCCESS
    # endregion: lifecycle callbacks

    # region: interface management
    def _create_interface_publisher(self) -> None:
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        with self._publish_lock:
            self._product_in_place_pub = self.create_publisher(Bool, '~/product_in_place', qos)
            self._product_in_place_pub.publish(Bool(data=False))

    def _create_interface_services(self) -> None:
        self._inject_srv = self.create_service(
            Trigger,
            '~/inject_product',
            self._inject_product_callback,
            callback_group=self._service_callback_group,
        )
        self._eject_srv = self.create_service(
            Trigger,
            '~/eject_product',
            self._eject_product_callback,
            callback_group=self._service_callback_group,
        )

    def _destroy_interface(self) -> None:
        if self._inject_srv is not None:
            self.destroy_service(self._inject_srv)
            self._inject_srv = None
        if self._eject_srv is not None:
            self.destroy_service(self._eject_srv)
            self._eject_srv = None
        with self._publish_lock:
            if self._product_in_place_pub is not None:
                self.destroy_publisher(self._product_in_place_pub)
                self._product_in_place_pub = None

    def _teardown_controller(self) -> None:
        if self._controller is not None and self._controller_ready:
            try:
                self._controller.teardown()
            except Exception as e:
                self.get_logger().error(f'Controller teardown failed: {e}')
        self._controller_ready = False

    def _release_all(self) -> None:
        self._destroy_interface()
        self._teardown_controller()
        self._controller = None
        self._config = None
    # endregion: interface management

    # region: callbacks
    def _on_presence_changed(self, in_place: bool) -> None:
        with self._publish_lock:
            if self._product_in_place_pub is None:
                return
            self._product_in_place_pub.publish(Bool(data=in_place))
        self.get_logger().info(f'Product in place: {in_place}')

    def _inject_product_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        controller = self._controller
        return self._run_action('inject_product', controller.inject_product if controller else None, response)

    def _eject_product_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        controller = self._controller
        return self._run_action('eject_product', controller.eject_product if controller else None, response)

    def _run_action(
        self,
        name: str,
        action: Callable[[], CarrierActionResultDTO] | None,
        response: Trigger.Response,
    ) -> Trigger.Response:
        try:
            if action is None:
                raise RuntimeError('Carrier controller is not available')
            result = action()
            response.success = result.success
            response.message = result.message
        except Exception as e:
            self.get_logger().error(f'{name} failed: {e}')
            response.success = False
            response.message = str(e)
        return response
    # endregion: callbacks


def main(args: list[str] | None = None) -> None:
    """Entry point for the ``conveyor`` executable.

    :param args: Optional command-line arguments forwarded to :func:`rclpy.init`.
    """
    init(args=args)
    node = Conveyor()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if ok():
            shutdown()
