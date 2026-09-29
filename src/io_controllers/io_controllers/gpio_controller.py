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
"""Device-agnostic GPIO controller lifecycle node.

``gpio_controller`` owns the normalized public ROS interface surface and
delegates every hardware concern to an :class:`AbstractIODriver` selected by
config (or the mock, when ``simulated=true``). It implements the managed
lifecycle, the fixed-rate ``io_state`` publisher, the best-effort ``set_io``
service, and the change-driven ``monitor_io`` action; the ``set_mock_behavior``
service (simulated mode) is added in a later step.
"""

import queue
import threading

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy

from io_controllers.action import MonitorIO
from io_controllers.drivers.driver_factory import create_driver
from io_controllers.enums.io_direction_enum import IODirectionEnum
from io_controllers.interfaces.io_driver import AbstractIODriver
from io_controllers.models.io_config_dto import IOControllerConfigDTO
from io_controllers.models.io_signal_dto import AnalogSignalDTO, DigitalSignalDTO
from io_controllers.msg import IOStates
from io_controllers.srv import SetIO, SetMockBehavior
from io_controllers.utils.config_loader import ConfigError, load_config
from io_controllers.utils.monitor_entry import MonitorEntry

# Metadata for one mapped signal, used to route/validate set_io requests.
# (is_digital, direction) keyed by io_name; built once from the driver mapping.
_SignalMeta = tuple[bool, IODirectionEnum]


class GpioController(LifecycleNode):
    """Device-agnostic GPIO controller as a managed lifecycle node."""

    def __init__(self, node_name: str = 'gpio_controller') -> None:
        """Declare parameters; defer all resource creation to lifecycle transitions.

        Args:
            node_name: The ROS node name. Defaults to ``'gpio_controller'``.
        """
        super().__init__(node_name)

        self.declare_parameter(
            'config_file',
            '',
            ParameterDescriptor(
                description='Path to the YAML mapping/config file (required). '
                'Absolute, or resolved against share/io_controllers/config.',
            ),
        )
        self.declare_parameter(
            'publish_rate_hz',
            10.0,
            ParameterDescriptor(
                description='Overrides publish_rate_hz from the config file when '
                '> 0; the fixed rate at which io_state snapshots publish.',
            ),
        )
        self.declare_parameter(
            'simulated',
            False,
            ParameterDescriptor(
                description='Use the device-agnostic MockIODriver instead of the '
                'real device driver, and advertise set_mock_behavior.',
            ),
        )

        self._config: IOControllerConfigDTO | None = None
        self._driver: AbstractIODriver | None = None
        self._simulated: bool = False
        self._publish_rate_hz: float = 10.0
        self._signal_index: dict[str, _SignalMeta] = {}
        self._io_state_pub = None
        self._set_io_srv = None
        self._publish_timer = None

        # set_mock_behavior state (simulated mode only). Scheduled input changes
        # run on node-clock one-shot timers; the lock guards the handle list
        # since both the service handler and the timer callbacks touch it.
        self._set_mock_behavior_srv = None
        self._mock_lock = threading.Lock()
        self._mock_timers: list = []

        # monitor_io state. Goals are only accepted while active; each active
        # goal registers an entry keyed by io_name so the change callback can
        # route changes to it. All rclpy calls stay on executor threads.
        self._active: bool = False
        self._monitor_group: ReentrantCallbackGroup | None = None
        self._monitor_action_server: ActionServer | None = None
        self._monitor_lock = threading.Lock()
        self._watchers: dict[str, set[MonitorEntry]] = {}

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Load config, construct the driver, and configure the static mapping.

        Args:
            state: The previous lifecycle state (unused).

        Returns:
            TransitionCallbackReturn.SUCCESS on success, FAILURE on any error
            (logged), leaving the node unconfigured.
        """
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self._simulated = self.get_parameter('simulated').get_parameter_value().bool_value

        try:
            self._config = load_config(config_file)
        except ConfigError as exc:
            self.get_logger().error(f'Failed to load config: {exc}')
            return TransitionCallbackReturn.FAILURE

        try:
            self._driver = create_driver(self._config, self, simulated=self._simulated)
            self._driver.configure(self._config)
        except Exception as exc:  # noqa: BLE001 - convert to lifecycle FAILURE
            self.get_logger().error(f'Failed to configure driver: {exc}')
            self._driver = None
            return TransitionCallbackReturn.FAILURE

        # Effective publish rate: the node parameter overrides the config value
        # only when explicitly set > 0.
        rate_param = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self._publish_rate_hz = rate_param if rate_param > 0.0 else self._config.publish_rate_hz

        # Build the static signal index (io_name -> (is_digital, direction))
        # from the driver's mapping so set_io can route and validate targets.
        snapshot = self._driver.snapshot()
        self._signal_index = {
            **{s.io_name: (True, s.direction) for s in snapshot.digital},
            **{s.io_name: (False, s.direction) for s in snapshot.analog},
        }

        # Create the io_state publisher and set_io service (inactive until
        # on_activate starts the timer). QoS: reliable, latest-wins snapshot.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._io_state_pub = self.create_lifecycle_publisher(IOStates, '~/io_states', qos)
        self._set_io_srv = self.create_service(SetIO, '~/set_io', self._on_set_io)

        # Simulated mode only: advertise input scripting (SPEC 7.6). Real
        # drivers get their input values from hardware, so this stays absent.
        if self._simulated:
            self._set_mock_behavior_srv = self.create_service(
                SetMockBehavior, '~/set_mock_behavior', self._on_set_mock_behavior
            )

        # Advertise the change-driven monitor_io action. A ReentrantCallbackGroup
        # under the MultiThreadedExecutor lets each goal's watch loop run on its
        # own executor thread concurrently with the change-routing callbacks.
        self._monitor_group = ReentrantCallbackGroup()
        self._monitor_action_server = ActionServer(
            self,
            MonitorIO,
            '~/monitor_io',
            execute_callback=self._execute_monitor,
            goal_callback=self._on_monitor_goal,
            cancel_callback=self._on_monitor_cancel,
            callback_group=self._monitor_group,
        )
        # Route genuine driver value changes to watching goals (enqueue-only).
        self._driver.set_change_callback(self._on_driver_change)

        mode = 'simulated' if self._simulated else self._config.device_type.value
        self.get_logger().info(f'Configured gpio_controller ({mode} driver).')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Establish live hardware links via the driver.

        Args:
            state: The previous lifecycle state (unused).

        Returns:
            TransitionCallbackReturn.SUCCESS on success, FAILURE on error.
        """
        if self._driver is None:
            self.get_logger().error('Cannot activate: driver is not configured.')
            return TransitionCallbackReturn.FAILURE
        try:
            self._driver.activate()
        except Exception as exc:  # noqa: BLE001 - convert to lifecycle FAILURE
            self.get_logger().error(f'Failed to activate driver: {exc}')
            return TransitionCallbackReturn.FAILURE

        # Activate managed entities (lifecycle publishers) before publishing.
        super().on_activate(state)

        # Start the fixed-rate snapshot publish timer.
        period = 1.0 / self._publish_rate_hz
        self._publish_timer = self.create_timer(period, self._publish_snapshot)

        # Begin accepting monitor_io goals; the change callback now has live
        # watchers to route to.
        self._active = True

        self.get_logger().info(
            f'Activated gpio_controller (publishing io_state at '
            f'{self._publish_rate_hz:.3f} Hz).'
        )
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Drop live hardware links while keeping the configured mapping.

        Args:
            state: The previous lifecycle state (unused).

        Returns:
            TransitionCallbackReturn.SUCCESS on success, FAILURE on error.
        """
        # Stop accepting new goals and signal active watch loops to wind down;
        # each aborts itself with message="deactivated" (SPEC 7.2).
        self._active = False

        # Stop publishing before dropping hardware links.
        if self._publish_timer is not None:
            self._publish_timer.cancel()
            self.destroy_timer(self._publish_timer)
            self._publish_timer = None

        # Cancel any pending mock-behavior input timers (SPEC 7.2).
        self._cancel_mock_timers()

        # Deactivate managed entities (lifecycle publishers).
        super().on_deactivate(state)

        if self._driver is not None:
            try:
                self._driver.deactivate()
            except Exception as exc:  # noqa: BLE001 - convert to lifecycle FAILURE
                self.get_logger().error(f'Failed to deactivate driver: {exc}')
                return TransitionCallbackReturn.FAILURE

        self.get_logger().info('Deactivated gpio_controller.')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Drop the driver and config, returning to the unconfigured state.

        Args:
            state: The previous lifecycle state (unused).

        Returns:
            TransitionCallbackReturn.SUCCESS.
        """
        self._teardown()
        self.get_logger().info('Cleaned up gpio_controller.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Tear down all resources on shutdown.

        Args:
            state: The previous lifecycle state (unused).

        Returns:
            TransitionCallbackReturn.SUCCESS.
        """
        self._teardown()
        self.get_logger().info('Shut down gpio_controller.')
        return TransitionCallbackReturn.SUCCESS

    def _publish_snapshot(self) -> None:
        """Timer callback: publish one full normalized snapshot of all signals."""
        if self._driver is None or self._io_state_pub is None:
            return
        msg = self._driver.snapshot().to_msg()
        msg.header.stamp = self.get_clock().now().to_msg()
        self._io_state_pub.publish(msg)

    # -- monitor_io action ---------------------------------------------------

    def _on_monitor_goal(self, goal_request: MonitorIO.Goal) -> GoalResponse:
        """Accept a monitor goal only while active and for a known io_name.

        Args:
            goal_request: The incoming goal carrying ``io_name``/``emit_initial``.

        Returns:
            GoalResponse.ACCEPT for a mapped signal while active, else REJECT.
        """
        if not self._active or self._driver is None:
            self.get_logger().warning('monitor_io goal rejected: node not active.')
            return GoalResponse.REJECT
        if goal_request.io_name not in self._signal_index:
            self.get_logger().warning(
                f"monitor_io goal rejected: unknown io_name '{goal_request.io_name}'."
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_monitor_cancel(self, goal_handle) -> CancelResponse:
        """Always accept cancellation of a monitor goal.

        Args:
            goal_handle: The goal handle requesting cancellation.

        Returns:
            CancelResponse.ACCEPT.
        """
        return CancelResponse.ACCEPT

    def _execute_monitor(self, goal_handle) -> MonitorIO.Result:
        """Watch one signal, streaming a feedback per change until the goal ends.

        Runs on an executor thread (ReentrantCallbackGroup). It registers a
        per-goal queue that the driver change callback feeds (enqueue-only), then
        drains that queue here and publishes feedback from this thread, so no
        raw/non-executor thread ever calls into ``rclpy`` (SPEC 7.5).

        Args:
            goal_handle: The accepted goal handle.

        Returns:
            MonitorIO.Result: Populated on cancel / deactivate / error.
        """
        request = goal_handle.request
        io_name = request.io_name
        is_digital, _ = self._signal_index[io_name]
        entry = MonitorEntry(goal_handle, io_name, is_digital)
        self._register_watcher(entry)
        try:
            if request.emit_initial:
                signal = self._current_signal(io_name, is_digital)
                if signal is not None:
                    stamp = self.get_clock().now().to_msg()
                    goal_handle.publish_feedback(
                        self._make_monitor_feedback(signal, is_digital, stamp)
                    )
                    entry.change_count += 1

            while True:
                if not self._active:
                    return self._finish_monitor(
                        goal_handle, entry, 'deactivated', outcome='aborted'
                    )
                if goal_handle.is_cancel_requested:
                    return self._finish_monitor(
                        goal_handle, entry, 'cancelled', outcome='canceled'
                    )
                try:
                    signal, stamp = entry.pending.get(timeout=0.1)
                except queue.Empty:
                    continue
                goal_handle.publish_feedback(
                    self._make_monitor_feedback(signal, is_digital, stamp)
                )
                entry.change_count += 1
        except Exception as exc:  # noqa: BLE001 - convert to aborted result
            self.get_logger().error(f"monitor_io failed for '{io_name}': {exc}")
            return self._finish_monitor(
                goal_handle, entry, f'error: {exc}', outcome='aborted'
            )
        finally:
            self._unregister_watcher(entry)

    def _on_driver_change(
        self, io_name: str, signal: DigitalSignalDTO | AnalogSignalDTO
    ) -> None:
        """Driver change callback: route one change to every watching goal.

        Enqueue-only and non-blocking per the driver contract (SPEC 7.5): it
        stamps the observation time and pushes into each interested goal's queue
        without touching any other ``rclpy`` API.

        Args:
            io_name: The signal whose cached value changed.
            signal: The new immutable signal snapshot.
        """
        stamp = self.get_clock().now().to_msg()
        with self._monitor_lock:
            watchers = self._watchers.get(io_name)
            if not watchers:
                return
            entries = tuple(watchers)
        for entry in entries:
            entry.pending.put((signal, stamp))

    def _register_watcher(self, entry: MonitorEntry) -> None:
        """Register a per-goal watch entry under its io_name."""
        with self._monitor_lock:
            self._watchers.setdefault(entry.io_name, set()).add(entry)

    def _unregister_watcher(self, entry: MonitorEntry) -> None:
        """Remove a per-goal watch entry, dropping the io_name key when empty."""
        with self._monitor_lock:
            watchers = self._watchers.get(entry.io_name)
            if watchers is not None:
                watchers.discard(entry)
                if not watchers:
                    del self._watchers[entry.io_name]

    def _current_signal(
        self, io_name: str, is_digital: bool
    ) -> DigitalSignalDTO | AnalogSignalDTO | None:
        """Return the current signal DTO for ``io_name`` from the driver snapshot."""
        snapshot = self._driver.snapshot()
        signals = snapshot.digital if is_digital else snapshot.analog
        for signal in signals:
            if signal.io_name == io_name:
                return signal
        return None

    def _make_monitor_feedback(
        self,
        signal: DigitalSignalDTO | AnalogSignalDTO,
        is_digital: bool,
        stamp,
    ) -> MonitorIO.Feedback:
        """Build one feedback message for a single observed change."""
        feedback = MonitorIO.Feedback()
        feedback.is_digital = is_digital
        feedback.stamp = stamp
        if is_digital:
            feedback.digital_io = signal.to_msg()
        else:
            feedback.analog_io = signal.to_msg()
        return feedback

    def _finish_monitor(
        self,
        goal_handle,
        entry: MonitorEntry,
        message: str,
        *,
        outcome: str,
    ) -> MonitorIO.Result:
        """Populate the result and move the goal to its terminal state.

        Args:
            goal_handle: The goal handle to finalize.
            entry: The per-goal watch state (source of ``change_count``).
            message: Human-readable termination reason.
            outcome: One of ``'canceled'`` (clean client cancel) or ``'aborted'``
                (node deactivate / error).

        Returns:
            MonitorIO.Result: The populated result.
        """
        result = MonitorIO.Result()
        result.change_count = entry.change_count
        result.message = message
        if outcome == 'canceled':
            result.success = True
            goal_handle.canceled()
        else:
            result.success = False
            goal_handle.abort()
        return result

    def _on_set_io(self, request: SetIO.Request, response: SetIO.Response) -> SetIO.Response:
        """Best-effort ``set_io`` handler.

        Every valid output entry is attempted; a failure on one does not abort
        the others. Inputs and unknown names are recorded as failures and never
        written. ``success`` is true only when nothing failed.

        Args:
            request: The set request carrying digital and analog entries.
            response: The response object to populate.

        Returns:
            SetIO.Response: The populated best-effort result.
        """
        failed: list[str] = []
        reasons: list[str] = []

        if self._driver is None:
            response.success = False
            response.message = 'node is not configured; no driver available'
            response.failed_io_names = []
            self.get_logger().error(response.message)
            return response

        for entry in request.command.digital_io:
            self._apply_set_entry(
                entry.io_name, bool(entry.state), expect_digital=True,
                failed=failed, reasons=reasons,
            )
        for entry in request.command.analog_io:
            self._apply_set_entry(
                entry.io_name, float(entry.state), expect_digital=False,
                failed=failed, reasons=reasons,
            )

        response.failed_io_names = failed
        response.success = not failed
        if failed:
            response.message = (
                f'{len(failed)} of '
                f'{len(request.command.digital_io) + len(request.command.analog_io)} '
                f'entries failed: ' + '; '.join(reasons)
            )
        else:
            response.message = ''
        return response

    def _apply_set_entry(
        self,
        io_name: str,
        value: bool | float,
        *,
        expect_digital: bool,
        failed: list[str],
        reasons: list[str],
    ) -> None:
        """Resolve, validate, and write one set_io entry, recording failures."""
        meta = self._signal_index.get(io_name)
        if meta is None:
            self._record_failure(io_name, 'unknown io_name', failed, reasons)
            return
        is_digital, direction = meta
        if direction is IODirectionEnum.IN:
            self._record_failure(io_name, 'is an input (not writable)', failed, reasons)
            return
        if is_digital != expect_digital:
            kind = 'digital' if is_digital else 'analog'
            self._record_failure(
                io_name, f'is {kind}, wrong request category', failed, reasons
            )
            return
        try:
            if is_digital:
                self._driver.set_digital(io_name, bool(value))
            else:
                self._driver.set_analog(io_name, float(value))
        except Exception as exc:  # noqa: BLE001 - convert to failure entry
            self._record_failure(io_name, str(exc), failed, reasons)

    def _record_failure(
        self, io_name: str, reason: str, failed: list[str], reasons: list[str]
    ) -> None:
        """Log and record a single set_io failure entry."""
        self.get_logger().error(f"set_io failed for '{io_name}': {reason}")
        failed.append(io_name)
        reasons.append(f"{io_name} ({reason})")

    # -- set_mock_behavior service (simulated mode only) ---------------------

    def _on_set_mock_behavior(
        self, request: SetMockBehavior.Request, response: SetMockBehavior.Response
    ) -> SetMockBehavior.Response:
        """Schedule one future value change for a simulated INPUT signal.

        Validates the target (must be simulated, a known input, non-negative
        delay), then appends a node-clock one-shot timer that applies the change
        via ``driver.apply_mock_change`` (SPEC 7.6). Append semantics: existing
        pending changes are never replaced or cancelled.

        Args:
            request: The scheduling request.
            response: The response object to populate.

        Returns:
            SetMockBehavior.Response: ``success=false`` with a reason on any
            rejection, else ``success=true``.
        """
        if not self._simulated or self._driver is None:
            return self._reject_mock(response, 'node is not in simulated mode')
        meta = self._signal_index.get(request.io_name)
        if meta is None:
            return self._reject_mock(response, f"unknown io_name '{request.io_name}'")
        is_digital, direction = meta
        if direction is not IODirectionEnum.IN:
            return self._reject_mock(
                response,
                f"io_name '{request.io_name}' is an output; only inputs are scriptable",
            )
        if request.delay_seconds < 0.0:
            return self._reject_mock(
                response,
                f'delay_seconds must be >= 0 (got {request.delay_seconds})',
            )

        value = bool(request.digital_state) if is_digital else float(request.analog_state)
        self._schedule_mock_change(request.io_name, value, float(request.delay_seconds))
        response.success = True
        response.message = ''
        return response

    def _reject_mock(
        self, response: SetMockBehavior.Response, message: str
    ) -> SetMockBehavior.Response:
        """Populate and log a rejected set_mock_behavior response."""
        self.get_logger().error(f'set_mock_behavior rejected: {message}')
        response.success = False
        response.message = message
        return response

    def _schedule_mock_change(
        self, io_name: str, value: bool | float, delay_seconds: float
    ) -> bool:
        """Append a node-clock one-shot timer that applies one input change.

        Returns:
            bool: True if the change was scheduled; False if the node is not
            active (in which case no timer is created).
        """
        timer = None

        def _fire() -> None:
            nonlocal timer
            if timer is not None:
                timer.cancel()  # one-shot: never fire again
            if self._driver is None:
                return
            try:
                self._driver.apply_mock_change(io_name, value)
            except Exception as exc:  # noqa: BLE001 - log and drop
                self.get_logger().error(
                    f"set_mock_behavior failed to apply '{io_name}': {exc}"
                )

        # Hold the lock across the active-check + create + append so a schedule
        # can never slip a timer in after _cancel_mock_timers has run: that
        # method swaps the list under the same lock, and on_deactivate/_teardown
        # set _active=False before calling it.
        with self._mock_lock:
            if not self._active:
                return False
            timer = self.create_timer(max(delay_seconds, 0.0), _fire)
            self._mock_timers.append(timer)
        return True

    def _cancel_mock_timers(self) -> None:
        """Cancel and destroy all pending/fired mock-behavior timers."""
        with self._mock_lock:
            timers = self._mock_timers
            self._mock_timers = []
        for timer in timers:
            timer.cancel()
            self.destroy_timer(timer)

    def _teardown(self) -> None:
        """Release ROS entities, the driver, and config references."""
        self._active = False
        self._cancel_mock_timers()
        if self._publish_timer is not None:
            self._publish_timer.cancel()
            self.destroy_timer(self._publish_timer)
            self._publish_timer = None
        if self._monitor_action_server is not None:
            self._monitor_action_server.destroy()
            self._monitor_action_server = None
        self._monitor_group = None
        with self._monitor_lock:
            self._watchers = {}
        if self._set_mock_behavior_srv is not None:
            self.destroy_service(self._set_mock_behavior_srv)
            self._set_mock_behavior_srv = None
        if self._set_io_srv is not None:
            self.destroy_service(self._set_io_srv)
            self._set_io_srv = None
        if self._io_state_pub is not None:
            self.destroy_publisher(self._io_state_pub)
            self._io_state_pub = None
        self._signal_index = {}
        self._driver = None
        self._config = None


def main(args: list[str] | None = None) -> None:
    """Spin the GPIO controller lifecycle node until shutdown.

    Args:
        args: Optional command-line arguments forwarded to ``rclpy.init``.

    Returns:
        None.
    """
    rclpy.init(args=args)
    node = GpioController()
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
