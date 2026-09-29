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
"""Integration tests for the simulated-mode ``set_mock_behavior`` service.

These drive the real :class:`MockIODriver` (``simulated=true``) so the whole
input-scripting path is exercised: schedule -> node-clock timer ->
``driver.apply_mock_change`` -> cache change (SPEC section 7.6). Rejection cases
call the handler directly; timing/apply cases go through a real service client
under a background executor so timers are created on the executor thread.
"""

import threading
import time

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.parameter import Parameter

from io_controllers import gpio_controller as gpio_module
from io_controllers.drivers.mock_io_driver import MockIODriver
from io_controllers.gpio_controller import GpioController
from io_controllers.srv import SetMockBehavior

_CONFIG_YAML = """\
device_type: ur
publish_rate_hz: 20.0
device:
  mapping:
    digital_in_states:
      - {io_name: part_present, default: false}
    digital_out_states:
      - {io_name: gripper_close, default: false}
    analog_in_states:
      - {io_name: temperature, domain: voltage, default: 0.0}
"""


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def config_file(tmp_path):
    path = tmp_path / 'cfg.yaml'
    path.write_text(_CONFIG_YAML)
    return str(path)


def _make_node(config_file, *, simulated=True):
    node = GpioController()
    node.set_parameters([
        Parameter('config_file', value=config_file),
        Parameter('simulated', value=simulated),
    ])
    return node


def _input_value(node, io_name, *, is_digital):
    snapshot = node._driver.snapshot()
    signals = snapshot.digital if is_digital else snapshot.analog
    for signal in signals:
        if signal.io_name == io_name:
            return signal.state
    raise AssertionError(f'{io_name} not found in snapshot')


def _wait_value(node, io_name, expected, *, is_digital, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end:
        if _input_value(node, io_name, is_digital=is_digital) == expected:
            return True
        time.sleep(0.02)
    return _input_value(node, io_name, is_digital=is_digital) == expected


def _call_mock(client, io_name, *, digital=False, analog=0.0, delay=0.0):
    request = SetMockBehavior.Request()
    request.io_name = io_name
    request.digital_state = digital
    request.analog_state = float(analog)
    request.delay_seconds = float(delay)
    future = client.call_async(request)
    end = time.time() + 5.0
    while not future.done() and time.time() < end:
        time.sleep(0.02)
    assert future.done(), 'set_mock_behavior call did not return'
    return future.result()


def _mock_setup(config_file):
    """Configure+activate a simulated node under a background executor."""
    node = _make_node(config_file, simulated=True)
    assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
    assert node.trigger_activate() == TransitionCallbackReturn.SUCCESS

    client_node = rclpy.create_node('mock_client')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    client = client_node.create_client(SetMockBehavior, '/gpio_controller/set_mock_behavior')
    assert client.wait_for_service(timeout_sec=5.0), 'set_mock_behavior not available'
    return node, client_node, executor, client


# -- service availability ----------------------------------------------------


def test_service_absent_when_not_simulated(monkeypatch, config_file, ros):
    # The real UR driver is not implemented yet, and the service-absent
    # behavior depends only on the `simulated` flag, so use a mock driver here.
    monkeypatch.setattr(
        gpio_module,
        'create_driver',
        lambda config, node, *, simulated: MockIODriver(),
    )
    node = _make_node(config_file, simulated=False)
    try:
        assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
        assert node._set_mock_behavior_srv is None
    finally:
        node.destroy_node()


def test_service_present_when_simulated(config_file, ros):
    node = _make_node(config_file, simulated=True)
    try:
        assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
        assert node._set_mock_behavior_srv is not None
    finally:
        node.destroy_node()


# -- rejections (direct handler call; no timers created) ---------------------


def test_rejects_output_target(config_file, ros):
    node = _make_node(config_file, simulated=True)
    try:
        assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
        request = SetMockBehavior.Request()
        request.io_name = 'gripper_close'  # output
        request.digital_state = True
        response = node._on_set_mock_behavior(request, SetMockBehavior.Response())
        assert response.success is False
        assert 'output' in response.message
    finally:
        node.destroy_node()


def test_rejects_unknown_io_name(config_file, ros):
    node = _make_node(config_file, simulated=True)
    try:
        assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
        request = SetMockBehavior.Request()
        request.io_name = 'does_not_exist'
        response = node._on_set_mock_behavior(request, SetMockBehavior.Response())
        assert response.success is False
        assert 'unknown' in response.message
    finally:
        node.destroy_node()


def test_rejects_negative_delay(config_file, ros):
    node = _make_node(config_file, simulated=True)
    try:
        assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
        request = SetMockBehavior.Request()
        request.io_name = 'part_present'
        request.digital_state = True
        request.delay_seconds = -1.0
        response = node._on_set_mock_behavior(request, SetMockBehavior.Response())
        assert response.success is False
        assert 'delay_seconds' in response.message
    finally:
        node.destroy_node()


# -- apply / timing (through the real service client) ------------------------


def test_immediate_apply_changes_input(config_file, ros):
    node, client_node, executor, client = _mock_setup(config_file)
    try:
        assert _input_value(node, 'part_present', is_digital=True) is False
        response = _call_mock(client, 'part_present', digital=True, delay=0.0)
        assert response.success is True
        assert _wait_value(node, 'part_present', True, is_digital=True)
    finally:
        executor.shutdown()
        client_node.destroy_node()
        node.destroy_node()


def test_delayed_apply_waits_for_timer(config_file, ros):
    node, client_node, executor, client = _mock_setup(config_file)
    try:
        response = _call_mock(client, 'part_present', digital=True, delay=0.4)
        assert response.success is True
        # Not applied immediately.
        assert _input_value(node, 'part_present', is_digital=True) is False
        # Applied after the delay elapses.
        assert _wait_value(node, 'part_present', True, is_digital=True, timeout=3.0)
    finally:
        executor.shutdown()
        client_node.destroy_node()
        node.destroy_node()


def test_append_preserves_time_order(config_file, ros):
    node, client_node, executor, client = _mock_setup(config_file)
    try:
        # Two changes on the same analog input, appended (never replaced).
        assert _call_mock(client, 'temperature', analog=1.0, delay=0.2).success is True
        assert _call_mock(client, 'temperature', analog=2.0, delay=0.5).success is True

        assert _wait_value(node, 'temperature', 1.0, is_digital=False, timeout=3.0)
        assert _wait_value(node, 'temperature', 2.0, is_digital=False, timeout=3.0)
    finally:
        executor.shutdown()
        client_node.destroy_node()
        node.destroy_node()
