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
"""Unit tests for the driver factory (SPEC section 8.4)."""

import pytest

from io_controllers.drivers.driver_factory import create_driver
from io_controllers.drivers.mock_io_driver import MockIODriver
from io_controllers.drivers.ur_io_driver import URIODriver
from io_controllers.enums.device_type_enum import DeviceTypeEnum
from io_controllers.interfaces.io_driver import AbstractIODriver
from io_controllers.models.io_config_dto import IOControllerConfigDTO


def _ur_config() -> IOControllerConfigDTO:
    return IOControllerConfigDTO(
        device_type=DeviceTypeEnum.UR,
        device={'mapping': {}},
    )


def test_simulated_returns_mock_driver():
    driver = create_driver(_ur_config(), node=object(), simulated=True)
    assert isinstance(driver, MockIODriver)
    assert isinstance(driver, AbstractIODriver)


def test_real_ur_returns_ur_driver():
    node = object()
    driver = create_driver(_ur_config(), node=node, simulated=False)
    assert isinstance(driver, URIODriver)
    assert isinstance(driver, AbstractIODriver)


def test_simulated_ignores_device_type_and_never_touches_real_driver():
    # Even with a real device_type, simulated must yield the mock.
    driver = create_driver(_ur_config(), node=None, simulated=True)
    assert isinstance(driver, MockIODriver)


def test_unsupported_real_device_type_raises(monkeypatch):
    # Emulate a config whose device_type has no registered real driver.
    from io_controllers.drivers import driver_factory

    monkeypatch.setattr(driver_factory, '_REAL_DRIVERS', {})
    with pytest.raises(ValueError, match='unsupported device_type'):
        create_driver(_ur_config(), node=object(), simulated=False)
