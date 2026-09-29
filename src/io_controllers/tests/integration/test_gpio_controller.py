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
"""Integration tests for the GPIO controller lifecycle (SPEC section 7.2).

These tests drive the managed lifecycle with a fake driver injected in place of
the real/mock driver, asserting each transition succeeds and delegates to the
driver. Publishing/service behavior is covered in later steps.
"""

import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.parameter import Parameter

from io_controllers import gpio_controller as gpio_module
from io_controllers.gpio_controller import GpioController
from io_controllers.enums.io_direction_enum import IODirectionEnum
from io_controllers.enums.io_domain_enum import IODomainEnum
from io_controllers.interfaces.io_driver import AbstractIODriver, ChangeCallback
from io_controllers.models.io_config_dto import IOControllerConfigDTO
from io_controllers.models.io_signal_dto import (
    AnalogSignalDTO,
    DigitalSignalDTO,
    IOStatesSnapshot,
)
from io_controllers.msg import AnalogIO, DigitalIO, IOStates
from io_controllers.srv import SetIO


class _FakeDriver(AbstractIODriver):
    """Records lifecycle calls and writes so node delegation can be asserted."""

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


def test_full_lifecycle_delegates_to_driver(monkeypatch, config_file, ros):
    fake = _FakeDriver()
    node = _make_node(monkeypatch, config_file, fake)
    try:
        assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
        assert isinstance(fake.configured_with, IOControllerConfigDTO)

        assert node.trigger_activate() == TransitionCallbackReturn.SUCCESS
        assert fake.activated is True

        assert node.trigger_deactivate() == TransitionCallbackReturn.SUCCESS
        assert fake.deactivated is True

        assert node.trigger_cleanup() == TransitionCallbackReturn.SUCCESS
    finally:
        node.destroy_node()


def test_configure_fails_on_missing_config(monkeypatch, ros):
    fake = _FakeDriver()
    # Empty config_file -> load_config raises -> configure FAILURE.
    node = _make_node(monkeypatch, '', fake)
    try:
        assert node.trigger_configure() == TransitionCallbackReturn.FAILURE
        assert fake.configured_with is None
    finally:
        node.destroy_node()


def _set_io_request(digital=(), analog=()):
    request = SetIO.Request()
    for io_name, state in digital:
        entry = DigitalIO()
        entry.io_name = io_name
        entry.state = state
        request.command.digital_io.append(entry)
    for io_name, state in analog:
        entry = AnalogIO()
        entry.io_name = io_name
        entry.state = float(state)
        request.command.analog_io.append(entry)
    return request


def test_publish_emits_full_snapshot(monkeypatch, config_file, ros):
    fake = _FakeDriver(_mapping_snapshot())
    node = _make_node(monkeypatch, config_file, fake)
    sub_node = rclpy.create_node('io_state_listener')
    received: list[IOStates] = []
    executor = SingleThreadedExecutor()
    try:
        assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
        assert node.trigger_activate() == TransitionCallbackReturn.SUCCESS

        sub_node.create_subscription(
            IOStates, '/gpio_controller/io_state', received.append, 1
        )
        executor.add_node(node)
        executor.add_node(sub_node)

        end = time.time() + 5.0
        while not received and time.time() < end:
            executor.spin_once(timeout_sec=0.1)

        assert received, 'expected at least one io_state snapshot'
        msg = received[-1]
        assert {d.io_name for d in msg.digital_io} == {'part_present', 'gripper_close'}
        assert {a.io_name for a in msg.analog_io} == {'spindle_ref'}
    finally:
        executor.shutdown()
        sub_node.destroy_node()
        node.destroy_node()


def test_set_io_best_effort_mixed_success_and_failure(monkeypatch, config_file, ros):
    fake = _FakeDriver(_mapping_snapshot())
    node = _make_node(monkeypatch, config_file, fake)
    try:
        assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
        assert node.trigger_activate() == TransitionCallbackReturn.SUCCESS

        request = _set_io_request(
            digital=[
                ('gripper_close', True),   # OK output
                ('part_present', True),    # input -> failure
                ('nope', True),            # unknown -> failure
            ],
            analog=[('spindle_ref', 2.5)],  # OK output
        )
        response = node._on_set_io(request, SetIO.Response())

        assert response.success is False
        assert set(response.failed_io_names) == {'part_present', 'nope'}
        assert ('gripper_close', True) in fake.writes
        assert ('spindle_ref', 2.5) in fake.writes
    finally:
        node.destroy_node()


def test_set_io_all_success(monkeypatch, config_file, ros):
    fake = _FakeDriver(_mapping_snapshot())
    node = _make_node(monkeypatch, config_file, fake)
    try:
        assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
        assert node.trigger_activate() == TransitionCallbackReturn.SUCCESS

        request = _set_io_request(digital=[('gripper_close', True)])
        response = node._on_set_io(request, SetIO.Response())

        assert response.success is True
        assert response.failed_io_names == []
        assert response.message == ''
    finally:
        node.destroy_node()


def test_set_io_driver_failure_recorded(monkeypatch, config_file, ros):
    fake = _FakeDriver(_mapping_snapshot())
    fake.fail_on = {'gripper_close'}
    node = _make_node(monkeypatch, config_file, fake)
    try:
        assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
        assert node.trigger_activate() == TransitionCallbackReturn.SUCCESS

        request = _set_io_request(digital=[('gripper_close', True)])
        response = node._on_set_io(request, SetIO.Response())

        assert response.success is False
        assert response.failed_io_names == ['gripper_close']
    finally:
        node.destroy_node()
