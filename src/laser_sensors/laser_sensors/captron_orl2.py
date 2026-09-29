#!/usr/bin/env python3
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
"""Captron ORL2-40T laser cross (TCP measurement) sensor ROS2 node.

The physical ORL2-40T-2PS6 is a laser *cross* Tool-Center-Point measuring unit:
two perpendicular laser lines meet at an intersection inside a 40 mm window and
the device exposes ``2x PNP-NO`` digital outputs, one per beam. An output is
active whenever an object (typically a tool tip) interrupts that beam at
the intersection.

Device datasheet and models: https://www.captron.com/products/detail/orl2-40t-2ps6/
"""

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor
from rclpy.publisher import Publisher
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan

from laser_sensors.msg import BeamTriggered
from io_controllers.msg import IOStates


class CaptronORL2(LifecycleNode):
    """ROS2 driver node for the Captron ORL2-40T laser cross sensor."""

    def __init__(self, node_name: str = 'captron_orl2') -> None:
        """Declare parameters and set up publishers (and, in sim, subscribers)."""
        super().__init__(node_name)

        self.declare_parameter(
            'simulated',
            True,
            ParameterDescriptor(
                description='Run against a simulated Gazebo sensor (True) or '
                'real hardware (False). Defaults to True.',
            ),
        )
        self.declare_parameter(
            'sensor_diameter',
            0.04,
            ParameterDescriptor(
                description='The diameter of the sensor window in meters. Defaults to 0.04 m (40 mm).',
            ),
        )
        self.declare_parameter(
            'frame_id',
            "captron_orl2_intersection",
            ParameterDescriptor(
                description='The frame ID to use for the laser cross intersection. Defaults to "captron_orl2_intersection".',
            ),
        )
        self.declare_parameter(
            'x_axis_beam_input',
            "cross_laser_x_triggered",
            ParameterDescriptor(
                description='The GPIO input for the X-axis beam. Defaults to "cross_laser_x_triggered".',
            ),
        )
        self.declare_parameter(
            'y_axis_beam_input',
            "cross_laser_y_triggered",
            ParameterDescriptor(
                description='The GPIO input for the Y-axis beam. Defaults to "cross_laser_y_triggered".',
            ),
        )
        self.declare_parameter(
            'io_states_topic',
            "/gpio_controller/io_states",
            ParameterDescriptor(
                description='The GPIO controller IOStates topic. Defaults to "/gpio_controller/io_states".',
            ),
        )

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        ""
        self._simulated = self.get_parameter('simulated').get_parameter_value().bool_value
        self._io_states_topic = self.get_parameter('io_states_topic').get_parameter_value().string_value
        self._x_axis_beam_input = self.get_parameter('x_axis_beam_input').get_parameter_value().string_value
        self._y_axis_beam_input = self.get_parameter('y_axis_beam_input').get_parameter_value().string_value
        self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self._sensor_diameter = self.get_parameter('sensor_diameter').get_parameter_value().double_value
        self.get_logger().info(
            f"Captron ORL2 configured. Frame ID: '{self._frame_id}', "
            f"X-axis beam input: '{self._x_axis_beam_input}', "
            f"Y-axis beam input: '{self._y_axis_beam_input}', "
            f"IOStates topic: '{self._io_states_topic}'"
        )
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Set up publishers and subscribers based on the launch configuration."""
        # Publish each PNP-NO output as its own stamped boolean so consumers see
        # an identical topic layout whether the sensor is real or simulated and
        # can rely on the header timestamp for each triggered state.
        self._output_1_pub = self.create_lifecycle_publisher(BeamTriggered, '~/x_axis_triggered', 10)
        self._output_2_pub = self.create_lifecycle_publisher(BeamTriggered, '~/y_axis_triggered', 10)

        try:
            if self._simulated:
                self._setup_simulated()
            else:
                self._setup_real_hardware()
        except Exception as e:
            self.get_logger().error(f"Error during activation: {e}")
            return TransitionCallbackReturn.FAILURE

        # Let the base class activate the managed lifecycle publishers. Without
        # this, LifecyclePublisher.publish() silently drops every message
        # because the publisher stays disabled, so nothing is published on the
        # beam topics even though the callbacks run.
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Clean up publishers and subscribers."""
        # Deactivate the managed publishers before destroying them.
        super().on_deactivate(state)
        self.destroy_publisher(self._output_1_pub)
        self.destroy_publisher(self._output_2_pub)
        return TransitionCallbackReturn.SUCCESS

    def _setup_real_hardware(self) -> None:
        """Set up GPIO inputs for the real hardware sensor (not implemented)."""
        self.create_subscription(
            IOStates,
            self._io_states_topic,
            self._parse_io_states_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Real Captron ORL2 ready. Listening for GPIO controller IOStates on "
            f"'{self._io_states_topic}'"
        )

    def _parse_io_states_callback(self, io_states: IOStates) -> None:
        """Parse the GPIO controller IOStates message and publish beam outputs."""
        x_axis_input = self._x_axis_beam_input
        y_axis_input = self._y_axis_beam_input

        x_axis_triggered = any(
            io.io_name == x_axis_input and io.state for io in io_states.digital_io
        )
        y_axis_triggered = any(
            io.io_name == y_axis_input and io.state for io in io_states.digital_io
        )

        self._publish_output(self._output_1_pub, x_axis_triggered)
        self._publish_output(self._output_2_pub, y_axis_triggered)

    # region: simulated subscription and callback
    def _setup_simulated(self) -> None:
        """Subscribe to the Gazebo-bridged single-ray beam sensors."""
        self.create_subscription(
            LaserScan,
            '/simulated_laser_cross/beam_x_axis',
            self._beam_x_simulated_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            '/simulated_laser_cross/beam_y_axis',
            self._beam_y_simulated_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Simulated Captron ORL2 ready. Listening for Gazebo laser scans on "
            f"'simulated_laser_cross/beam_x_axis' -> '{self.get_name()}/x_axis_triggered', "
            f"'simulated_laser_cross/beam_y_axis' -> '{self.get_name()}/y_axis_triggered'"
        )

    def _beam_x_simulated_callback(self, scan: LaserScan) -> None:
        """Publish output 1 from the first simulated beam scan."""
        interrupted = any(
            scan.range_min <= distance <= self._sensor_diameter
            for distance in scan.ranges
        )
        self._publish_output(self._output_1_pub, interrupted)

    def _beam_y_simulated_callback(self, scan: LaserScan) -> None:
        """Publish output 2 from the second simulated beam scan."""
        interrupted = any(
            scan.range_min <= distance <= self._sensor_diameter
            for distance in scan.ranges
        )
        self._publish_output(self._output_2_pub, interrupted)
    # endregion: simulated subscription and callback

    def _publish_output(self, publisher: Publisher, interrupted: bool) -> None:
        """Convert a single-ray scan into a beam-interrupted boolean output."""
        msg = BeamTriggered()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.triggered = interrupted
        publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    """Spin the Captron ORL2 sensor node until shutdown."""
    rclpy.init(args=args)
    node = CaptronORL2()
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