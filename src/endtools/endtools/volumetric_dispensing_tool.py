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
from rclpy.service import Service
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import String
from std_srvs.srv import Trigger

from tf2_ros import StaticTransformBroadcaster

from endtools.srv import StopDispensing

from endtools.enumerator import ToolTypeEnum
from endtools.model import DispensingToolConfigDTO
from endtools.interface import DispenserController
from endtools.service import (
    SimulatedDispenserController,
    HardwareDispenserController
)
from endtools.utils import build_tcp_transform


class VolumetricDispensingTool(LifecycleNode):

    def __init__(self, node_name: str = 'volumetric_dispensing_tool') -> None:
        super().__init__(node_name)

        # internal attributes
        self._config: DispensingToolConfigDTO|None = None
        self._controller: DispenserController|None = None

        # region: parameters
        self.declare_parameter(
            'tool_type',
            "VOL_DISPENSER",
            ParameterDescriptor(description='Tool type', read_only=True),
        )
        self.declare_parameter(
            'simulated',
            True,
            ParameterDescriptor(description='Whether the node is running in simulation mode'),
        )
        
        self.declare_parameter(
            'tool_sn',
            "dispensing_ABC123",
            ParameterDescriptor(description='Tool serial number'),
        )
        self.declare_parameter(
            'tcp_frame_id',
            "tool0",
            ParameterDescriptor(description='Tool TCP frame ID'),
        )
        self.declare_parameter(
            'tcp',
            [0.0837, 0.0, -0.267, 1.570797, 0.0, 1.570797],
            ParameterDescriptor(description='Calibrated tool TCP pose against "tcp_frame_id" frame'),
        )
        self.declare_parameter(
            'tcp_uncalibrated',
            [0.0837, 0.0, -0.267, 1.570797, 0.0, 1.570797],
            ParameterDescriptor(description='Nominal (uncalibrated) tool TCP pose against "tcp_frame_id" frame'),
        )
        self.declare_parameter(
            'mounted',
            False,
            ParameterDescriptor(description='Whether the tool is mounted on tool-mount'),
        )
        self.declare_parameter(
            'flowrate',
            1.0,
            ParameterDescriptor(description='Volumetric flow rate (cc/s).'),
        )

        # services, publishers, and subscribers
        self._start_service: Service|None = None
        self._stop_service: Service|None = None
        self._is_mounted_publisher: Publisher|None = None
        self._static_tf_broadcaster: StaticTransformBroadcaster|None = None

        # callback groups
        self._service_callback_group = ReentrantCallbackGroup()
        self._publisher_callback_group = ReentrantCallbackGroup()
        
    # region: lifecycle callbacks
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Configuring from state: {state.label}')
        if super().on_configure(state) != TransitionCallbackReturn.SUCCESS:
            return TransitionCallbackReturn.FAILURE

        try:
            self._config = DispensingToolConfigDTO(
                tool_type=ToolTypeEnum(self.get_parameter('tool_type').get_parameter_value().string_value),
                tool_sn=self.get_parameter('tool_sn').get_parameter_value().string_value,
                tcp_frame_id=self.get_parameter('tcp_frame_id').get_parameter_value().string_value,
                tcp=list(self.get_parameter('tcp').get_parameter_value().double_array_value),
                tcp_uncalibrated=list(self.get_parameter('tcp_uncalibrated').get_parameter_value().double_array_value),
                mounted=self.get_parameter('mounted').get_parameter_value().bool_value,
                flow_rate=self.get_parameter('flowrate').get_parameter_value().double_value,
                simulated=self.get_parameter('simulated').get_parameter_value().bool_value,
            )

            # publish a latched mounted state topic for other nodes to subscribe to
            self._is_mounted_publisher = self.create_lifecycle_publisher(
                String, '~/mounted', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            )
            self._is_mounted_publisher.publish(String(data=str(self._config.mounted).lower()))

            # Broadcast the calibrated and uncalibrated TCP frames as static
            # transforms relative to tcp_frame_id
            # TODO: these should be broadcast only when tool is mounted!!
            self._static_tf_broadcaster = StaticTransformBroadcaster(self)
            stamp = self.get_clock().now().to_msg()
            self._static_tf_broadcaster.sendTransform([
                build_tcp_transform(
                    self._config.tcp_frame_id,
                    self._config.calibrated_tcp_frame_id,
                    self._config.tcp,
                    stamp,
                ),
                build_tcp_transform(
                    self._config.tcp_frame_id,
                    self._config.uncalibrated_tcp_frame_id,
                    self._config.tcp_uncalibrated,
                    stamp,
                ),
            ])

            if self._config.simulated:
                self.get_logger().info('Running in simulation mode; using SimulatedDispenserController.')
                self._controller = SimulatedDispenserController(self)
            else:
                self.get_logger().info('Running in hardware mode; using HardwareDispenserController.')
                self._controller = HardwareDispenserController(self)
        except ValidationError as e:
            self.get_logger().error(f'Failed to read config values: {e}')
            return TransitionCallbackReturn.FAILURE
        except Exception as e:
            self.get_logger().error(f'Unknown error while configuring: {e}')
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Activating from state: {state.label}')
        if super().on_activate(state) != TransitionCallbackReturn.SUCCESS:
            return TransitionCallbackReturn.FAILURE

        if self._config is None:
            self.get_logger().error('Cannot activate: configuration is not set.')
            return TransitionCallbackReturn.FAILURE
        elif self._config.mounted is False:
            self.get_logger().error('Tool is not mounted, cannot activate.')
            return TransitionCallbackReturn.FAILURE
        elif self._controller is None:
            self.get_logger().error('Controller is not initialized, cannot activate.')
            return TransitionCallbackReturn.FAILURE

        self._controller.setup(self._config)

        self._start_service = self.create_service(
            Trigger, '~/dispense_start', self._handle_dispense_start,
            callback_group=self._service_callback_group,
        )
        self._stop_service = self.create_service(
            StopDispensing, '~/dispense_stop', self._handle_dispense_stop,
            callback_group=self._service_callback_group,
        )

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Deactivating from state: {state.label}')
        if super().on_deactivate(state) != TransitionCallbackReturn.SUCCESS:
            return TransitionCallbackReturn.FAILURE

        self.destroy_service(self._start_service) if self._start_service else None
        self.destroy_service(self._stop_service) if self._stop_service else None
        self._controller.teardown() if self._controller else None

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Cleaning up from state: {state.label}')
        if super().on_cleanup(state) != TransitionCallbackReturn.SUCCESS:
            return TransitionCallbackReturn.FAILURE

        self._config = None
        self.destroy_publisher(self._is_mounted_publisher) if self._is_mounted_publisher else None
        if self._static_tf_broadcaster is not None:
            self.destroy_publisher(self._static_tf_broadcaster.pub_tf)
            self._static_tf_broadcaster = None
        self._controller = None

        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().error(f'Error occurred in state: {state.label}')
        return super().on_error(state)
    # endregion: lifecycle callbacks

    # region: callbacks
    def _handle_dispense_start(self, _: Trigger.Request, response: Trigger.Response):
        """Handle the dispense start service request."""
        if self._controller is None:
            response.success = False
            response.message = "Controller is not initialized."
            return response
        if self._controller.is_dispensing:
            response.success = False
            response.message = "Dispensing is already in progress."
            return response

        try:
            self._controller.start_dispensing()
            response.success = True
            response.message = "Dispensing started successfully."
        except RuntimeError as e:
            response.success = False
            response.message = str(e)
        except Exception as e:
            response.success = False
            response.message = f"Unexpected error: {e}"

        return response

    def _handle_dispense_stop(self, _: StopDispensing.Request, response: StopDispensing.Response):
        """Handle the dispense stop service request."""
        if self._controller is None:
            response.success = False
            response.error_message = "Controller is not initialized."
            return response

        try:
            metrics = self._controller.stop_dispensing()
            response.success = True
            response.error_message = ""
            response.dispensed_volume = metrics.dispensed_volume_cc
            response.duration_seconds = metrics.dispensing_duration_s
        except RuntimeError as e:
            response.success = False
            response.error_message = str(e)
        except Exception as e:
            response.success = False
            response.error_message = f"Unexpected error: {e}"

        return response

    # endregion: callbacks


def main(args=None) -> None:
    """Entry point for the ``volumetric_dispensing_tool`` executable.

    Initialises rclpy, creates a :class:`VolumetricDispensingTool` node, and spins
    it with a :class:`~rclpy.executors.MultiThreadedExecutor` (5 threads) to
    allow concurrent goal, feedback, and cancel callbacks.  Shuts down cleanly
    on exit or keyboard interrupt.

    :param args: Optional command-line arguments forwarded to :func:`rclpy.init`.
    :type args: list[str] | None
    """
    init(args=args)
    call_shutdown = True
    node = VolumetricDispensingTool()
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down.')
        call_shutdown = False
    finally:
        if call_shutdown:
            shutdown()
