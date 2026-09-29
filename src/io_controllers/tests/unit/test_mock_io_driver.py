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
"""Unit tests for the device-agnostic mock driver (SPEC sections 8.3, 9.4)."""

import pytest

from io_controllers.drivers.mock_io_driver import MockIODriver
from io_controllers.enums.device_type_enum import DeviceTypeEnum
from io_controllers.enums.io_direction_enum import IODirectionEnum
from io_controllers.enums.io_domain_enum import IODomainEnum
from io_controllers.models.io_config_dto import IOControllerConfigDTO
from io_controllers.models.io_signal_dto import AnalogSignalDTO, DigitalSignalDTO


def _config(mapping: dict) -> IOControllerConfigDTO:
    return IOControllerConfigDTO(
        device_type=DeviceTypeEnum.UR,
        device={'controller_namespace': '/io', 'mapping': mapping},
    )


def _standard_config() -> IOControllerConfigDTO:
    return _config(
        {
            'digital_in_states': [
                {'pin': 0, 'io_name': 'part_present', 'default': False},
                {'pin': 1, 'io_name': 'door_closed', 'default': True},
            ],
            'digital_out_states': [
                {'pin': 0, 'io_name': 'gripper_close'},
            ],
            'analog_in_states': [
                {'pin': 0, 'io_name': 'line_pressure', 'domain': 'voltage',
                 'default': 1.5},
            ],
            'analog_out_states': [
                {'pin': 0, 'io_name': 'spindle_ref', 'domain': 'current'},
            ],
        }
    )


def _configured_driver() -> MockIODriver:
    driver = MockIODriver()
    driver.configure(_standard_config())
    return driver


def _snapshot_by_name(driver: MockIODriver) -> dict:
    snap = driver.snapshot()
    return {s.io_name: s for s in [*snap.digital, *snap.analog]}


def test_cache_seeds_from_defaults():
    driver = _configured_driver()
    by_name = _snapshot_by_name(driver)
    assert by_name['part_present'].state is False
    assert by_name['door_closed'].state is True
    assert by_name['line_pressure'].state == 1.5
    assert by_name['spindle_ref'].state == 0.0


def test_snapshot_covers_all_signals_split_by_type():
    driver = _configured_driver()
    snap = driver.snapshot()
    assert {s.io_name for s in snap.digital} == {
        'part_present', 'door_closed', 'gripper_close'
    }
    assert {s.io_name for s in snap.analog} == {'line_pressure', 'spindle_ref'}
    assert all(isinstance(s, DigitalSignalDTO) for s in snap.digital)
    assert all(isinstance(s, AnalogSignalDTO) for s in snap.analog)


def test_set_digital_output_fires_callback_on_change():
    driver = _configured_driver()
    events = []
    driver.set_change_callback(lambda name, sig: events.append((name, sig.state)))

    driver.set_digital('gripper_close', True)

    assert events == [('gripper_close', True)]
    assert _snapshot_by_name(driver)['gripper_close'].state is True


def test_set_digital_output_no_callback_when_unchanged():
    driver = _configured_driver()
    events = []
    driver.set_change_callback(lambda name, sig: events.append(name))

    driver.set_digital('gripper_close', False)  # already False (default)

    assert events == []


def test_set_analog_output_fires_callback_on_change():
    driver = _configured_driver()
    events = []
    driver.set_change_callback(lambda name, sig: events.append((name, sig.state)))

    driver.set_analog('spindle_ref', 3.3)

    assert events == [('spindle_ref', 3.3)]
    assert _snapshot_by_name(driver)['spindle_ref'].state == 3.3


def test_apply_mock_change_on_input_fires_callback():
    driver = _configured_driver()
    events = []
    driver.set_change_callback(lambda name, sig: events.append((name, sig.state)))

    driver.apply_mock_change('part_present', True)

    assert events == [('part_present', True)]
    assert _snapshot_by_name(driver)['part_present'].state is True


def test_apply_mock_change_analog_input():
    driver = _configured_driver()
    driver.apply_mock_change('line_pressure', 2.75)
    assert _snapshot_by_name(driver)['line_pressure'].state == 2.75


def test_set_io_refuses_input():
    driver = _configured_driver()
    with pytest.raises(ValueError, match='input'):
        driver.set_digital('part_present', True)


def test_apply_mock_change_refuses_output():
    driver = _configured_driver()
    with pytest.raises(ValueError, match='output'):
        driver.apply_mock_change('gripper_close', True)


def test_set_digital_unknown_name_raises():
    driver = _configured_driver()
    with pytest.raises(KeyError):
        driver.set_digital('nope', True)


def test_set_digital_on_analog_signal_raises():
    driver = _configured_driver()
    with pytest.raises(ValueError, match='digital'):
        driver.set_digital('spindle_ref', True)


def test_duplicate_io_name_rejected_at_configure():
    config = _config(
        {
            'digital_in_states': [{'pin': 0, 'io_name': 'dup'}],
            'digital_out_states': [{'pin': 0, 'io_name': 'dup'}],
        }
    )
    driver = MockIODriver()
    with pytest.raises(ValueError, match='duplicate io_name'):
        driver.configure(config)


def test_callback_can_be_cleared():
    driver = _configured_driver()
    events = []
    driver.set_change_callback(lambda name, sig: events.append(name))
    driver.set_change_callback(None)

    driver.set_digital('gripper_close', True)

    assert events == []
