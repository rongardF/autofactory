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
"""Unit tests for the Universal Robots I/O driver.

The driver is exercised with a fake node (no live ROS graph): the read path is
driven by feeding real ``ur_msgs/msg/IOStates`` messages to ``_on_io_states``,
and the write path is verified by inspecting the requests captured by fake
service clients.
"""

import pytest
from ur_msgs.msg import Analog, Digital, IOStates
from ur_msgs.srv import SetAnalogOutput, SetIO

from io_controllers.drivers.ur_io_driver import URIODriver
from io_controllers.enums.device_type_enum import DeviceTypeEnum
from io_controllers.models.io_config_dto import IOControllerConfigDTO

_NS = '/io'
_SET_IO = f'{_NS}/set_io'
_SET_ANALOG = f'{_NS}/set_analog_output'


# --------------------------------------------------------------------------- #
# Fakes
# --------------------------------------------------------------------------- #
class _FakeFuture:
    def add_done_callback(self, callback) -> None:
        self._callback = callback


class _FakeClient:
    def __init__(self, available: bool = True) -> None:
        self.available = available
        self.requests = []

    def wait_for_service(self, timeout_sec=None) -> bool:
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        return _FakeFuture()


class _FakeLogger:
    def error(self, *_args, **_kwargs) -> None:
        pass


class _FakeNode:
    def __init__(self, service_available: bool = True) -> None:
        self._service_available = service_available
        self.clients: dict[str, _FakeClient] = {}
        self.subscription = None
        self.sub_callback = None
        self.destroyed_subscriptions = []
        self.destroyed_clients = []

    def create_subscription(self, _msg_type, _topic, callback, _qos):
        self.subscription = object()
        self.sub_callback = callback
        return self.subscription

    def create_client(self, _srv_type, name):
        client = _FakeClient(available=self._service_available)
        self.clients[name] = client
        return client

    def destroy_subscription(self, sub) -> None:
        self.destroyed_subscriptions.append(sub)

    def destroy_client(self, client) -> None:
        self.destroyed_clients.append(client)

    def get_logger(self) -> _FakeLogger:
        return _FakeLogger()


# --------------------------------------------------------------------------- #
# Config / driver fixtures
# --------------------------------------------------------------------------- #
def _config() -> IOControllerConfigDTO:
    return IOControllerConfigDTO(
        device_type=DeviceTypeEnum.UR,
        device={
            'controller_namespace': _NS,
            'mapping': {
                'digital_in_states': [
                    {'pin': 0, 'io_name': 'part_present', 'default': False},
                ],
                'digital_out_states': [
                    {'pin': 1, 'io_name': 'gripper_close'},
                ],
                'flag_states': [
                    {'pin': 2, 'io_name': 'cycle_flag'},
                ],
                'analog_in_states': [
                    {'pin': 0, 'io_name': 'line_pressure', 'domain': 'voltage',
                     'default': 1.5},
                ],
                'analog_out_states': [
                    {'pin': 1, 'io_name': 'spindle_ref', 'domain': 'current'},
                ],
                'tool_voltage': [
                    {'io_name': 'tool_v'},
                ],
            },
        },
    )


def _configured_driver() -> URIODriver:
    driver = URIODriver(_FakeNode())
    driver.configure(_config())
    return driver


def _activated_driver(service_available: bool = True) -> tuple[URIODriver, _FakeNode]:
    node = _FakeNode(service_available=service_available)
    driver = URIODriver(node)
    driver.configure(_config())
    driver.activate()
    return driver, node


def _snapshot_by_name(driver: URIODriver) -> dict:
    snap = driver.snapshot()
    return {s.io_name: s for s in [*snap.digital, *snap.analog]}


def _digital(pin: int, state: bool) -> Digital:
    entry = Digital()
    entry.pin = pin
    entry.state = state
    return entry


def _analog(pin: int, state: float) -> Analog:
    entry = Analog()
    entry.pin = pin
    entry.state = state
    return entry


# --------------------------------------------------------------------------- #
# configure / snapshot
# --------------------------------------------------------------------------- #
def test_cache_seeds_from_defaults():
    by_name = _snapshot_by_name(_configured_driver())
    assert by_name['part_present'].state is False
    assert by_name['line_pressure'].state == 1.5
    assert by_name['spindle_ref'].state == 0.0
    assert by_name['tool_v'].state == 0.0


def test_snapshot_splits_digital_and_analog():
    snap = _configured_driver().snapshot()
    assert {s.io_name for s in snap.digital} == {
        'part_present', 'gripper_close', 'cycle_flag'
    }
    assert {s.io_name for s in snap.analog} == {
        'line_pressure', 'spindle_ref', 'tool_v'
    }


def test_configure_rejects_duplicate_io_names():
    config = IOControllerConfigDTO(
        device_type=DeviceTypeEnum.UR,
        device={
            'controller_namespace': _NS,
            'mapping': {
                'digital_in_states': [{'pin': 0, 'io_name': 'dup'}],
                'digital_out_states': [{'pin': 0, 'io_name': 'dup'}],
            },
        },
    )
    with pytest.raises(ValueError):
        URIODriver(_FakeNode()).configure(config)


# --------------------------------------------------------------------------- #
# read path (io_states)
# --------------------------------------------------------------------------- #
def test_io_states_updates_snapshot():
    driver = _configured_driver()
    msg = IOStates()
    msg.digital_in_states = [_digital(0, True)]
    msg.analog_in_states = [_analog(0, 2.7)]

    driver._on_io_states(msg)

    by_name = _snapshot_by_name(driver)
    assert by_name['part_present'].state is True
    assert by_name['line_pressure'].state == 2.7


def test_io_states_change_callback_fires_once_per_change():
    driver = _configured_driver()
    events = []
    driver.set_change_callback(lambda name, sig: events.append((name, sig.state)))

    msg = IOStates()
    msg.digital_in_states = [_digital(0, True)]
    driver._on_io_states(msg)
    driver._on_io_states(msg)  # identical republish -> no change

    assert events == [('part_present', True)]


def test_io_states_ignores_unmapped_pins():
    driver = _configured_driver()
    events = []
    driver.set_change_callback(lambda name, sig: events.append(name))

    msg = IOStates()
    msg.digital_in_states = [_digital(99, True)]  # no such pin
    driver._on_io_states(msg)

    assert events == []


def test_configured_domain_is_authoritative_on_read():
    driver = _configured_driver()
    msg = IOStates()
    # Analog message carries its own domain; the driver keeps the configured one.
    entry = _analog(0, 4.2)
    entry.domain = Analog.CURRENT
    msg.analog_in_states = [entry]

    driver._on_io_states(msg)

    assert _snapshot_by_name(driver)['line_pressure'].domain.value == 'voltage'


# --------------------------------------------------------------------------- #
# write path
# --------------------------------------------------------------------------- #
def test_set_digital_out_builds_set_io_request():
    driver, node = _activated_driver()
    driver.set_digital('gripper_close', True)

    request = node.clients[_SET_IO].requests[-1]
    assert request.fun == SetIO.Request.FUN_SET_DIGITAL_OUT
    assert request.pin == 1
    assert request.state == float(SetIO.Request.STATE_ON)


def test_set_flag_builds_set_io_request():
    driver, node = _activated_driver()
    driver.set_digital('cycle_flag', False)

    request = node.clients[_SET_IO].requests[-1]
    assert request.fun == SetIO.Request.FUN_SET_FLAG
    assert request.pin == 2
    assert request.state == float(SetIO.Request.STATE_OFF)


def test_set_analog_out_builds_set_analog_output_request():
    driver, node = _activated_driver()
    driver.set_analog('spindle_ref', 3.3)

    request = node.clients[_SET_ANALOG].requests[-1]
    assert isinstance(request, SetAnalogOutput.Request)
    assert request.data.pin == 1
    assert request.data.domain == Analog.CURRENT
    assert request.data.state == pytest.approx(3.3)


def test_set_tool_voltage_builds_set_io_and_updates_cache():
    driver, node = _activated_driver()
    events = []
    driver.set_change_callback(lambda name, sig: events.append((name, sig.state)))

    driver.set_analog('tool_v', 24.0)

    request = node.clients[_SET_IO].requests[-1]
    assert request.fun == SetIO.Request.FUN_SET_TOOL_VOLTAGE
    assert request.pin == 0
    assert request.state == pytest.approx(24.0)
    # Command-only: cache reflects the last commanded value and fires a change.
    assert _snapshot_by_name(driver)['tool_v'].state == pytest.approx(24.0)
    assert events == [('tool_v', pytest.approx(24.0))]


def test_set_digital_on_input_rejected():
    driver, _ = _activated_driver()
    with pytest.raises(ValueError):
        driver.set_digital('part_present', True)


def test_set_analog_on_digital_output_rejected():
    driver, _ = _activated_driver()
    with pytest.raises(ValueError):
        driver.set_analog('gripper_close', 1.0)


def test_set_unknown_name_rejected():
    driver, _ = _activated_driver()
    with pytest.raises(KeyError):
        driver.set_digital('nope', True)


def test_write_before_activate_raises():
    driver = _configured_driver()
    with pytest.raises(RuntimeError):
        driver.set_digital('gripper_close', True)


# --------------------------------------------------------------------------- #
# lifecycle
# --------------------------------------------------------------------------- #
def test_activate_creates_subscription_and_clients():
    _, node = _activated_driver()
    assert node.subscription is not None
    assert set(node.clients) == {_SET_IO, _SET_ANALOG}


def test_activate_raises_when_service_unavailable():
    node = _FakeNode(service_available=False)
    driver = URIODriver(node)
    driver.configure(_config())
    with pytest.raises(RuntimeError):
        driver.activate()


def test_deactivate_destroys_entities():
    driver, node = _activated_driver()
    driver.deactivate()
    assert len(node.destroyed_subscriptions) == 1
    assert len(node.destroyed_clients) == 2
