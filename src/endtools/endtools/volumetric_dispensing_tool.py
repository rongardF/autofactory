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
from rclpy.client import Client
from rclpy.duration import Duration
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

from repositories.msg import NumberArrayCache
from repositories.srv import FetchNumberArray, StoreNumberArray

from endtools.srv import SetTcp, StopDispensing

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

        # The current calibrated TCP pose. ``None`` until it is either fetched
        # from the station cache or explicitly set via the ``set_tcp`` service.
        self._tool_tcp: list[float]|None = None

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
            'tcp_uncalibrated',
            [0.0837, 0.0, -0.267, 1.570797, 0.0, 1.570797],
            ParameterDescriptor(description='Nominal (uncalibrated) tool TCP pose against "tcp_frame_id" frame'),
        )
        self.declare_parameter(
            'station_cache_name',
            "station_cache",
            ParameterDescriptor(description='Node name of the station_cache node providing the TCP cache services'),
        )
        self.declare_parameter(
            'tcp_valid_period',
            86400.0,
            ParameterDescriptor(description='Default validity period (seconds) for a cached TCP value'),
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
        self._set_tcp_service: Service|None = None
        self._is_mounted_publisher: Publisher|None = None
        self._static_tf_broadcaster: StaticTransformBroadcaster|None = None

        # station_cache service clients (created on configure, destroyed on cleanup)
        self._store_tcp_client: Client|None = None
        self._fetch_tcp_client: Client|None = None

        # callback groups
        self._service_callback_group = ReentrantCallbackGroup()
        self._publisher_callback_group = ReentrantCallbackGroup()
        # Clients live in a separate group so synchronous calls made from within
        # a service callback do not deadlock the executor.
        self._cache_client_callback_group = ReentrantCallbackGroup()

    # region: tool_tcp property
    @property
    def tool_tcp(self) -> list[float]:
        """Current calibrated TCP pose as [x, y, z, roll, pitch, yaw] (m, rad).

        When the backing ``_tool_tcp`` is unset, the value is fetched from
        ``station_cache``. If the cache has no live entry (missing or expired),
        the uncalibrated TCP is returned and a warning is logged.
        """
        if self._tool_tcp is None:
            fetched = self._fetch_tcp_from_cache()
            if fetched is not None:
                self._tool_tcp = fetched
            else:
                key = self._config.tcp_cache_key if self._config else '<unconfigured>'
                self.get_logger().warning(
                    f'TCP not found or expired in station_cache for key '
                    f'{key!r}; falling back to uncalibrated TCP.'
                )
                return list(self._config.tcp_uncalibrated) if self._config else []
        return self._tool_tcp

    @tool_tcp.setter
    def tool_tcp(self, value: list[float]) -> None:
        """Set the calibrated TCP, persisting it in station_cache first.

        The value is stored in the cache before ``_tool_tcp`` is updated and the
        static TF is re-published. If the cache store fails, ``_tool_tcp`` and
        the TF are left unchanged and a :class:`RuntimeError` is raised.
        """
        if self._config is None:
            raise RuntimeError('Cannot set TCP before the node is configured')
        if not self._store_tcp_in_cache(value, self._config.tcp_valid_period):
            self.get_logger().warning(
                f'Failed to store TCP in station_cache for key {self._config.tcp_cache_key!r}.'
            )
            raise RuntimeError('Failed to store TCP in station_cache')
        self._tool_tcp = list(value)
        self._publish_tcp_transforms()
    # endregion: tool_tcp property

    # region: tcp helpers
    def _publish_tcp_transforms(self) -> None:
        """Broadcast the calibrated and uncalibrated TCP frames as static TF.

        The frames are only published while the tool is mounted; when the tool
        is not mounted no TCP frames are broadcast.
        """
        if self._static_tf_broadcaster is None or self._config is None:
            return
        if not self._config.mounted:
            return
        stamp = self.get_clock().now().to_msg()
        self._static_tf_broadcaster.sendTransform([
            build_tcp_transform(
                self._config.tcp_frame_id,
                self._config.calibrated_tcp_frame_id,
                self.tool_tcp,
                stamp,
            ),
            build_tcp_transform(
                self._config.tcp_frame_id,
                self._config.uncalibrated_tcp_frame_id,
                self._config.tcp_uncalibrated,
                stamp,
            ),
        ])

    def _store_tcp_in_cache(self, tcp: list[float], valid_period: float) -> bool:
        """Store ``tcp`` in station_cache under the tool's TCP key.

        :returns: True if the value was stored successfully, False otherwise.
        """
        if self._store_tcp_client is None or self._config is None:
            return False
        if not self._store_tcp_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning('station_cache store_number_array service unavailable.')
            return False

        entry = NumberArrayCache()
        entry.key = self._config.tcp_cache_key
        entry.value = list(tcp)
        entry.expire_at_valid_till = True
        entry.valid_till = (self.get_clock().now() + Duration(seconds=valid_period)).to_msg()

        request = StoreNumberArray.Request()
        request.data = entry
        try:
            response = self._store_tcp_client.call(request)
        except Exception as e:
            self.get_logger().warning(f'station_cache store call failed: {e}')
            return False
        return response is not None and bool(response.success)

    def _fetch_tcp_from_cache(self) -> list[float]|None:
        """Fetch the cached TCP for this tool from station_cache.

        :returns: The cached TCP, or None if missing, expired, or unavailable.
        """
        if self._fetch_tcp_client is None or self._config is None:
            return None
        if not self._fetch_tcp_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning('station_cache fetch_number_array service unavailable.')
            return None

        request = FetchNumberArray.Request()
        request.key = self._config.tcp_cache_key
        try:
            response = self._fetch_tcp_client.call(request)
        except Exception as e:
            self.get_logger().warning(f'station_cache fetch call failed: {e}')
            return None
        if response is not None and response.found:
            return list(response.data.value)
        return None
    # endregion: tcp helpers

    # region: lifecycle callbacks
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Configuring from state: {state.label}')
        if super().on_configure(state) != TransitionCallbackReturn.SUCCESS:
            return TransitionCallbackReturn.FAILURE

        try:
            tcp_valid_period = self.get_parameter('tcp_valid_period').get_parameter_value().double_value
            self._config = DispensingToolConfigDTO(
                tool_type=ToolTypeEnum(self.get_parameter('tool_type').get_parameter_value().string_value),
                tool_sn=self.get_parameter('tool_sn').get_parameter_value().string_value,
                tcp_frame_id=self.get_parameter('tcp_frame_id').get_parameter_value().string_value,
                tcp_uncalibrated=list(self.get_parameter('tcp_uncalibrated').get_parameter_value().double_array_value),
                mounted=self.get_parameter('mounted').get_parameter_value().bool_value,
                flow_rate=self.get_parameter('flowrate').get_parameter_value().double_value,
                simulated=self.get_parameter('simulated').get_parameter_value().bool_value,
                default_tcp_valid_period=tcp_valid_period,
                tcp_valid_period=tcp_valid_period,
            )

            # Resolve the station_cache service name. The TCP cache key and
            # validity periods now live on the frozen config DTO.
            station_cache_name = self.get_parameter('station_cache_name').get_parameter_value().string_value

            # Create the station_cache clients used by the tool_tcp property to
            # persist and retrieve the calibrated TCP.
            self._store_tcp_client = self.create_client(
                StoreNumberArray, f'/{station_cache_name}/store_number_array',
                callback_group=self._cache_client_callback_group,
            )
            self._fetch_tcp_client = self.create_client(
                FetchNumberArray, f'/{station_cache_name}/fetch_number_array',
                callback_group=self._cache_client_callback_group,
            )

            # publish a latched mounted state topic for other nodes to subscribe to
            self._is_mounted_publisher = self.create_lifecycle_publisher(
                String, '~/mounted', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            )
            self._is_mounted_publisher.publish(String(data=str(self._config.mounted).lower()))

            # Broadcast the calibrated and uncalibrated TCP frames as static
            # transforms relative to tcp_frame_id. The calibrated frame uses the
            # current tool_tcp (cached value or uncalibrated fallback). Frames
            # are only published while the tool is mounted.
            self._static_tf_broadcaster = StaticTransformBroadcaster(self)
            self._publish_tcp_transforms()

            # The set_tcp service is available in both the inactive (configured)
            # and active states; it is destroyed on cleanup.
            self._set_tcp_service = self.create_service(
                SetTcp, '~/set_tcp', self._handle_set_tcp,
                callback_group=self._service_callback_group,
            )

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
        self.destroy_service(self._set_tcp_service) if self._set_tcp_service else None
        self._set_tcp_service = None
        self.destroy_publisher(self._is_mounted_publisher) if self._is_mounted_publisher else None
        if self._static_tf_broadcaster is not None:
            self.destroy_publisher(self._static_tf_broadcaster.pub_tf)
            self._static_tf_broadcaster = None
        if self._store_tcp_client is not None:
            self.destroy_client(self._store_tcp_client)
            self._store_tcp_client = None
        if self._fetch_tcp_client is not None:
            self.destroy_client(self._fetch_tcp_client)
            self._fetch_tcp_client = None
        self._tool_tcp = None
        self._controller = None

        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().error(f'Error occurred in state: {state.label}')
        return super().on_error(state)
    # endregion: lifecycle callbacks

    # region: callbacks
    def _handle_set_tcp(self, request: SetTcp.Request, response: SetTcp.Response):
        """Handle the set_tcp service request.

        Validates the TCP length, resolves the cache validity period (request
        override or node default), and assigns the ``tool_tcp`` property, which
        persists the value in station_cache and re-publishes the static TF.
        """
        tcp = list(request.tcp)
        if len(tcp) != 6:
            response.success = False
            response.error_message = (
                f'tcp must have exactly 6 elements [x, y, z, roll, pitch, yaw], got {len(tcp)}'
            )
            return response

        if self._config is None:
            response.success = False
            response.error_message = 'Node is not configured.'
            return response

        period = request.tcp_valid_period if request.tcp_valid_period > 0.0 else self._config.default_tcp_valid_period
        try:
            self._config = self._config.model_copy(update={'tcp_valid_period': period})
            self.tool_tcp = tcp
            response.success = True
            response.error_message = ""
        except Exception as e:
            response.success = False
            response.error_message = str(e)

        return response

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
