#!/usr/bin/env python3
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
"""Lifecycle node for the Siemens SIMATIC RF240R RFID reader (RS-232 ASCII).

Exposes the common RFID scanner interface of this package:

* ``~/tag_detected`` (``std_msgs/Bool``, latched) - a usable tag is detected;
* ``~/read_data`` (``rfid_scanner/srv/ReadData``) - read tag memory;
* ``~/write_data`` (``rfid_scanner/srv/WriteData``) - write tag memory.

The node runs against real hardware (6GT2821-4AC40, RS-232 ASCII protocol) or
against the Gazebo RFID simulation, selected by the ``simulated`` parameter.
Fatal reader errors move the node from *active* to *unconfigured*.
"""

import traceback
from threading import Lock

import rclpy
from pydantic import ValidationError
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.publisher import Publisher
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.service import Service
from std_msgs.msg import Bool

from rfid_scanner.enums.parity_enum import ParityEnum
from rfid_scanner.exceptions.reader_fatal_error import ReaderFatalError
from rfid_scanner.exceptions.tag_access_error import TagAccessError
from rfid_scanner.interface.rfid_reader import RfidReader
from rfid_scanner.models.node_config_dto import NodeConfigDTO
from rfid_scanner.srv import ReadData, WriteData
from rfid_scanner.services.simulated_reader import SimulatedReader
from rfid_scanner.services.simatic_ascii_reader import SimaticAsciiReader


class SimaticRf240rAscii(LifecycleNode):
    """ROS 2 driver node for the SIMATIC RF240R reader."""

    _TAG_DETECTED_QOS = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )

    def __init__(self, node_name: str = 'simatic_rf240r_ascii') -> None:
        """Declare parameters; resources are created by lifecycle transitions."""
        super().__init__(node_name)

        defaults = NodeConfigDTO()
        self.declare_parameter('simulated', defaults.simulated, ParameterDescriptor(
            description='Use the Gazebo-simulated reader (true) or real hardware (false).'))
        self.declare_parameter('device_name', defaults.device_name, ParameterDescriptor(
            description='Device name; namespaces the Gazebo topics/service. Must equal '
            'the xacro device_name.'))
        self.declare_parameter('serial_port', defaults.serial_port, ParameterDescriptor(
            description='Serial device of the reader (real hardware only).'))
        self.declare_parameter('baud_rate', defaults.baud_rate, ParameterDescriptor(
            description='Serial baud rate: 19200, 57600 or 115200. Must match the reader.'))
        self.declare_parameter('parity', defaults.parity.value, ParameterDescriptor(
            description='Serial parity: odd, even or none. Must match the reader.'))
        self.declare_parameter('stop_bits', defaults.stop_bits, ParameterDescriptor(
            description='Serial stop bits: 1 or 2. Must match the reader.'))
        self.declare_parameter(
            'response_timeout_sec', defaults.response_timeout_sec, ParameterDescriptor(
                description='Maximum wait for a command acknowledgement, in seconds.'))
        self.declare_parameter(
            'heartbeat_period_sec', defaults.heartbeat_period_sec, ParameterDescriptor(
                description='SLG-STATUS heartbeat period, in seconds (real hardware only).'))

        self._config: NodeConfigDTO | None = None
        self._reader: RfidReader | None = None
        self._service_group = ReentrantCallbackGroup()
        self._tag_detected_pub: Publisher | None = None
        self._read_srv: Service | None = None
        self._write_srv: Service | None = None

        self._publish_lock: Lock = Lock()
        self._publishing_enabled: bool = False
        self._last_published: bool | None = None

        self._fatal_lock: Lock = Lock()
        self._fatal_pending: bool = False
        self._is_active: bool = False
        # Wakes the executor from any thread to run the deferred self-transition.
        self._fatal_guard = self.create_guard_condition(self._on_fatal_guard)

    # region lifecycle transitions
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Validate the parameters and construct the reader service."""
        try:
            self._config = NodeConfigDTO(
                simulated=self.get_parameter('simulated').get_parameter_value().bool_value,
                device_name=self.get_parameter('device_name').get_parameter_value().string_value,
                serial_port=self.get_parameter('serial_port').get_parameter_value().string_value,
                baud_rate=self.get_parameter('baud_rate').get_parameter_value().integer_value,  # type: ignore
                parity=ParityEnum(self.get_parameter('parity').get_parameter_value().string_value),
                stop_bits=self.get_parameter('stop_bits').get_parameter_value().integer_value,  # type: ignore
                response_timeout_sec=self.get_parameter('response_timeout_sec').get_parameter_value().double_value,
                heartbeat_period_sec=self.get_parameter('heartbeat_period_sec').get_parameter_value().double_value,
            )
        except ValidationError as e:
            for error in e.errors():
                location = '.'.join(str(part) for part in error['loc'])
                self.get_logger().error(f"Invalid parameter '{location}': {error['msg']}")
            self._config = None
            return TransitionCallbackReturn.FAILURE

        try:
            if self._config.simulated:
                self._reader = SimulatedReader(
                    self, self._config, self._on_detected_changed, self._on_fatal_error
                )
            else:
                self._reader = SimaticAsciiReader(
                    self, self._config, self._on_detected_changed, self._on_fatal_error
                )
        except Exception as e:
            self.get_logger().error(f'Failed to construct the reader: {e}')
            self._reader = None
            self._config = None
            return TransitionCallbackReturn.FAILURE

        mode = 'simulated' if self._config.simulated else f"serial '{self._config.serial_port}'"
        self.get_logger().info(
            f"Configured ({mode}, device_name '{self._config.device_name}')")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Create the interface, initialise the reader and publish the initial state."""
        with self._fatal_lock:
            self._fatal_pending = False
        with self._publish_lock:
            self._last_published = None
            self._publishing_enabled = False

        reader = self._reader
        if reader is None:
            self.get_logger().error('Activation failed: node is not configured')
            return TransitionCallbackReturn.FAILURE
        try:
            self._tag_detected_pub = self.create_lifecycle_publisher(
                Bool, '~/tag_detected', self._TAG_DETECTED_QOS)
            self._read_srv = self.create_service(
                ReadData, '~/read_data', self._handle_read,
                callback_group=self._service_group)
            self._write_srv = self.create_service(
                WriteData, '~/write_data', self._handle_write,
                callback_group=self._service_group)
            reader.initialize()
            # Enables the lifecycle publisher; must happen before any publish.
            result = super().on_activate(state)
            if result != TransitionCallbackReturn.SUCCESS:
                raise RuntimeError(f'base activation returned {result}')
        except Exception as e:
            self.get_logger().error(f'Activation failed: {e}')
            reader.deinitialize()
            self._destroy_interface()
            return TransitionCallbackReturn.FAILURE

        with self._publish_lock:
            self._publishing_enabled = True
        self._is_active = True
        self._publish_detected(reader.is_tag_detected, initial=True)
        self.get_logger().info('Activated')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Publish false, destroy the interface and de-initialise the reader."""
        self._is_active = False
        self._publish_false_and_disable()
        super().on_deactivate(state)
        self._destroy_interface()
        if self._reader is not None:
            self._reader.deinitialize()
        self.get_logger().info('Deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Drop the reader service and the configuration."""
        self._reader = None
        self._config = None
        self.get_logger().info('Cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Release whatever exists, from any state."""
        self._is_active = False
        self._publish_false_and_disable()
        if self._tag_detected_pub is not None:
            super().on_deactivate(state)
        self._destroy_interface()
        if self._reader is not None:
            self._reader.deinitialize()
        self._reader = None
        self._config = None
        return TransitionCallbackReturn.SUCCESS
    # endregion lifecycle transitions

    # region tag_detected publishing
    def _on_detected_changed(self, detected: bool) -> None:
        """Publish a detection change reported by the reader (any thread)."""
        self._publish_detected(detected)

    def _publish_detected(self, detected: bool, initial: bool = False) -> None:
        """Publish ``detected`` if publishing is enabled and the value changed."""
        with self._publish_lock:
            if not self._publishing_enabled or self._tag_detected_pub is None:
                return
            # The initial value is skipped if a change was already published.
            if initial and self._last_published is not None:
                return
            if detected == self._last_published:
                return
            self._tag_detected_pub.publish(Bool(data=detected))
            self._last_published = detected

    def _publish_false_and_disable(self) -> None:
        """Publish false (if needed) and stop publishing."""
        with self._publish_lock:
            if (
                self._publishing_enabled and
                self._tag_detected_pub is not None and 
                self._last_published is not False
            ):
                self._tag_detected_pub.publish(Bool(data=False))
                self._last_published = False
            self._publishing_enabled = False

    def _destroy_interface(self) -> None:
        """Destroy the publisher and services if they exist."""
        if self._read_srv is not None:
            self.destroy_service(self._read_srv)
            self._read_srv = None
        if self._write_srv is not None:
            self.destroy_service(self._write_srv)
            self._write_srv = None
        if self._tag_detected_pub is not None:
            self.destroy_lifecycle_publisher(self._tag_detected_pub)  # type: ignore ;the create service itself annotates a 'Publisher' not a 'LifecyclePublisher'
            self._tag_detected_pub = None
    # endregion tag_detected publishing

    # region fatal self-transition
    def _on_fatal_error(self, reason: str) -> None:
        """Schedule the deferred deactivate -> cleanup (any thread, idempotent)."""
        with self._fatal_lock:
            if self._fatal_pending:
                return
            self._fatal_pending = True
        self.get_logger().error(
            f'Fatal reader error, transitioning to unconfigured: {reason}')
        self._fatal_guard.trigger()

    def _on_fatal_guard(self) -> None:
        """Run the self-transition in the executor."""
        with self._fatal_lock:
            if not self._fatal_pending:
                return
        if not self._is_active:
            self.get_logger().info('Fatal error reported but node is no longer active')
            return
        result = self.trigger_deactivate()
        self.get_logger().info(f'Self-transition deactivate: {result}')
        result = self.trigger_cleanup()
        self.get_logger().info(f'Self-transition cleanup: {result}')
    # endregion fatal self-transition

    # region services
    def _handle_read(
        self, request: ReadData.Request, response: ReadData.Response,
    ) -> ReadData.Response:
        """Read tag memory."""
        reader = self._reader
        try:
            if reader is None:
                raise TagAccessError('node is not active')
            data = reader.read(request.start_address, request.length)
            response.success = True
            response.error_message = ''
            response.data = list(data)
        except Exception as e:
            self._fail_response(response, 'read', request.start_address, e)
            response.data = []
        return response

    def _handle_write(
        self, request: WriteData.Request, response: WriteData.Response,
    ) -> WriteData.Response:
        """Write tag memory."""
        reader = self._reader
        try:
            if reader is None:
                raise TagAccessError('node is not active')
            reader.write(request.start_address, bytes(request.data))
            response.success = True
            response.error_message = ''
        except Exception as e:
            self._fail_response(response, 'write', request.start_address, e)
        return response

    def _fail_response(
        self,
        response: ReadData.Response | WriteData.Response,
        operation: str,
        start_address: int,
        error: Exception,
    ) -> None:
        """Log the error and fill a failure response."""
        message = str(error) or type(error).__name__
        prefix = f'{operation} at address {start_address} failed'
        if isinstance(error, TagAccessError):
            self.get_logger().warning(f'{prefix}: {message}')
        elif isinstance(error, ReaderFatalError):
            self.get_logger().error(f'{prefix}: {message}')
        else:
            self.get_logger().error(f'{prefix}: {message}\n{traceback.format_exc()}')
        response.success = False
        response.error_message = message
    # endregion services


def main(args: list[str] | None = None) -> None:
    """Spin the SIMATIC RF240R node until shutdown."""
    rclpy.init(args=args)
    node = SimaticRf240rAscii()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
