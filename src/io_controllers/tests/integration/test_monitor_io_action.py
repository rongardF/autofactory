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
"""Integration tests for the ``monitor_io`` action (SPEC section 7.5).

With a fake ``IODriver`` injected in place of the real/mock driver, these tests
exercise the change-driven ``monitor_io`` action server: goal acceptance and
``emit_initial`` feedback, one feedback per genuine driver change, cancellation
result semantics, rejection of unknown ``io_name``s, and abort-on-deactivate.
"""

import threading
import time

import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.parameter import Parameter

from io_controllers import gpio_controller as gpio_module
from io_controllers.action import MonitorIO
from io_controllers.enums.io_direction_enum import IODirectionEnum
from io_controllers.enums.io_domain_enum import IODomainEnum
from io_controllers.gpio_controller import GpioController
from io_controllers.interfaces.io_driver import AbstractIODriver, ChangeCallback
from io_controllers.models.io_config_dto import IOControllerConfigDTO
from io_controllers.models.io_signal_dto import (
    AnalogSignalDTO,
    DigitalSignalDTO,
    IOStatesSnapshot,
)


class _FakeDriver(AbstractIODriver):
    """A driver whose value changes can be triggered on demand for the node."""

    def __init__(self, snapshot: IOStatesSnapshot | None = None) -> None:
        self.configured_with: IOControllerConfigDTO | None = None
        self.activated = False
        self.deactivated = False
        self.writes: list[tuple[str, object]] = []
        self.fail_on: set[str] = set()
        self._snapshot = snapshot or IOStatesSnapshot()
        self._change_callback: ChangeCallback | None = None

    def configure(self, config: IOControllerConfigDTO) -> None:
        self.configured_with = config

    def activate(self) -> None:
        self.activated = True

    def deactivate(self) -> None:
        self.deactivated = True

    def snapshot(self) -> IOStatesSnapshot:
        return self._snapshot

    def set_change_callback(self, callback: ChangeCallback | None) -> None:
        self._change_callback = callback

    def emit_change(
        self, io_name: str, signal: DigitalSignalDTO | AnalogSignalDTO
    ) -> None:
        """Simulate a genuine driver value change routed to the node."""
        if self._change_callback is not None:
            self._change_callback(io_name, signal)

    def set_digital(self, io_name: str, state: bool) -> None:
        if io_name in self.fail_on:
            raise RuntimeError('simulated hardware failure')
        self.writes.append((io_name, state))

    def set_analog(self, io_name: str, state: float) -> None:
        if io_name in self.fail_on:
            raise RuntimeError('simulated hardware failure')
        self.writes.append((io_name, state))


def _mapping_snapshot() -> IOStatesSnapshot:
    return IOStatesSnapshot(
        digital=[
            DigitalSignalDTO(io_name='part_present', direction=IODirectionEnum.IN),
            DigitalSignalDTO(io_name='gripper_close', direction=IODirectionEnum.OUT),
        ],
        analog=[
            AnalogSignalDTO(
                io_name='spindle_ref',
                direction=IODirectionEnum.OUT,
                domain=IODomainEnum.CURRENT,
            ),
        ],
    )


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def config_file(tmp_path):
    path = tmp_path / 'cfg.yaml'
    path.write_text('device_type: ur\ndevice:\n  mapping: {}\n')
    return str(path)


def _make_node(monkeypatch, config_file, fake, *, simulated=True):
    monkeypatch.setattr(
        gpio_module,
        'create_driver',
        lambda config, node, *, simulated: fake,
    )
    node = GpioController()
    node.set_parameters([
        Parameter('config_file', value=config_file),
        Parameter('simulated', value=simulated),
    ])
    return node


def _wait_future(future, timeout: float = 5.0) -> bool:
    """Poll a future to completion while an executor spins in the background."""
    end = time.time() + timeout
    while not future.done() and time.time() < end:
        time.sleep(0.02)
    return future.done()


def _monitor_setup(monkeypatch, config_file, fake):
    """Configure+activate the node under a background MultiThreadedExecutor.

    Returns (node, client_node, executor, client) with the action server up.
    """
    node = _make_node(monkeypatch, config_file, fake)
    assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
    assert node.trigger_activate() == TransitionCallbackReturn.SUCCESS

    client_node = rclpy.create_node('monitor_client')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    client = ActionClient(client_node, MonitorIO, '/gpio_controller/monitor_io')
    assert client.wait_for_server(timeout_sec=5.0), 'monitor_io server not available'
    return node, client_node, executor, client


def test_monitor_io_rejects_unknown_io_name(monkeypatch, config_file, ros):
    fake = _FakeDriver(_mapping_snapshot())
    node, client_node, executor, client = _monitor_setup(monkeypatch, config_file, fake)
    try:
        goal = MonitorIO.Goal()
        goal.io_name = 'does_not_exist'
        goal.emit_initial = False
        send_future = client.send_goal_async(goal)
        assert _wait_future(send_future)
        assert send_future.result().accepted is False
    finally:
        executor.shutdown()
        client_node.destroy_node()
        node.destroy_node()


def test_monitor_io_streams_changes_and_cancels(monkeypatch, config_file, ros):
    fake = _FakeDriver(_mapping_snapshot())
    node, client_node, executor, client = _monitor_setup(monkeypatch, config_file, fake)
    feedbacks = []
    try:
        goal = MonitorIO.Goal()
        goal.io_name = 'part_present'
        goal.emit_initial = True
        send_future = client.send_goal_async(
            goal, feedback_callback=lambda fb: feedbacks.append(fb.feedback)
        )
        assert _wait_future(send_future)
        goal_handle = send_future.result()
        assert goal_handle.accepted is True

        # emit_initial baseline feedback.
        end = time.time() + 5.0
        while not feedbacks and time.time() < end:
            time.sleep(0.02)
        assert feedbacks, 'expected emit_initial feedback'
        assert feedbacks[0].is_digital is True
        assert feedbacks[0].digital_io.io_name == 'part_present'

        # A genuine driver change produces one more feedback.
        fake.emit_change(
            'part_present',
            DigitalSignalDTO(
                io_name='part_present', direction=IODirectionEnum.IN, state=True
            ),
        )
        end = time.time() + 5.0
        while len(feedbacks) < 2 and time.time() < end:
            time.sleep(0.02)
        assert len(feedbacks) >= 2

        # Cancel -> clean termination with a populated result.
        cancel_future = goal_handle.cancel_goal_async()
        assert _wait_future(cancel_future)
        result_future = goal_handle.get_result_async()
        assert _wait_future(result_future)
        result = result_future.result().result
        assert result.success is True
        assert result.message == 'cancelled'
        assert result.change_count >= 2
    finally:
        executor.shutdown()
        client_node.destroy_node()
        node.destroy_node()


def test_monitor_io_aborts_on_deactivate(monkeypatch, config_file, ros):
    fake = _FakeDriver(_mapping_snapshot())
    node, client_node, executor, client = _monitor_setup(monkeypatch, config_file, fake)
    try:
        goal = MonitorIO.Goal()
        goal.io_name = 'gripper_close'
        goal.emit_initial = False
        send_future = client.send_goal_async(goal)
        assert _wait_future(send_future)
        goal_handle = send_future.result()
        assert goal_handle.accepted is True

        # Deactivating the node must terminate the active goal.
        assert node.trigger_deactivate() == TransitionCallbackReturn.SUCCESS

        result_future = goal_handle.get_result_async()
        assert _wait_future(result_future)
        result = result_future.result().result
        assert result.success is False
        assert result.message == 'deactivated'
    finally:
        executor.shutdown()
        client_node.destroy_node()
        node.destroy_node()
