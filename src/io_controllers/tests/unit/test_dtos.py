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
"""Unit tests for the enum and DTO model layer (SPEC section 5)."""

import pytest
from pydantic import BaseModel, ValidationError

from io_controllers.enums import DeviceTypeEnum, IODirectionEnum, IODomainEnum
from io_controllers.models import (
    AnalogSignalDTO,
    DigitalSignalDTO,
    IOControllerConfigDTO,
    IOStatesSnapshot,
)


def test_enum_values_match_wire_strings():
    # Enum values must equal the string message constants exactly (SPEC 4/5.1).
    assert IODirectionEnum.IN.value == 'in'
    assert IODirectionEnum.OUT.value == 'out'
    assert IODomainEnum.VOLTAGE.value == 'voltage'
    assert IODomainEnum.CURRENT.value == 'current'
    assert DeviceTypeEnum.UR.value == 'ur'


def test_enums_are_str_subclasses():
    # str, Enum inheritance keeps them JSON/serialization friendly.
    assert isinstance(IODirectionEnum.IN, str)
    assert isinstance(IODomainEnum.VOLTAGE, str)
    assert isinstance(DeviceTypeEnum.UR, str)


def test_digital_signal_defaults():
    sig = DigitalSignalDTO(io_name='part_present', direction=IODirectionEnum.IN)
    assert sig.default is False
    assert sig.state is False


def test_analog_signal_requires_domain():
    with pytest.raises(ValidationError):
        AnalogSignalDTO(io_name='pressure', direction=IODirectionEnum.IN)


def test_analog_signal_full_construction():
    sig = AnalogSignalDTO(
        io_name='line_pressure',
        direction=IODirectionEnum.IN,
        domain=IODomainEnum.VOLTAGE,
        default=1.5,
        state=2.0,
    )
    assert sig.domain is IODomainEnum.VOLTAGE
    assert sig.default == pytest.approx(1.5)
    assert sig.state == pytest.approx(2.0)


def test_signal_dtos_are_frozen():
    digital = DigitalSignalDTO(io_name='d', direction=IODirectionEnum.OUT)
    analog = AnalogSignalDTO(
        io_name='a', direction=IODirectionEnum.OUT, domain=IODomainEnum.CURRENT
    )
    with pytest.raises(ValidationError):
        digital.state = True
    with pytest.raises(ValidationError):
        analog.state = 3.0


def test_snapshot_is_not_frozen_and_defaults_empty():
    snap = IOStatesSnapshot()
    assert snap.digital == []
    assert snap.analog == []
    # Mutable: cache updates replace the lists in place.
    snap.digital = [DigitalSignalDTO(io_name='d', direction=IODirectionEnum.OUT)]
    assert len(snap.digital) == 1


def test_config_envelope_defaults_and_validation():
    cfg = IOControllerConfigDTO(device_type=DeviceTypeEnum.UR, device={'a': 1})
    assert cfg.publish_rate_hz == pytest.approx(10.0)
    assert cfg.device == {'a': 1}


def test_config_rejects_non_positive_rate():
    with pytest.raises(ValidationError):
        IOControllerConfigDTO(
            device_type=DeviceTypeEnum.UR, publish_rate_hz=0.0, device={}
        )


def test_config_rejects_unknown_device_type():
    with pytest.raises(ValidationError):
        IOControllerConfigDTO(device_type='fanuc', device={})


def test_config_is_frozen():
    cfg = IOControllerConfigDTO(device_type=DeviceTypeEnum.UR, device={})
    with pytest.raises(ValidationError):
        cfg.publish_rate_hz = 20.0


@pytest.mark.parametrize(
    'model',
    [DigitalSignalDTO, AnalogSignalDTO, IOControllerConfigDTO, IOStatesSnapshot],
)
def test_every_field_has_description(model: type[BaseModel]):
    for name, field in model.model_fields.items():
        assert field.description, f'{model.__name__}.{name} missing description'
