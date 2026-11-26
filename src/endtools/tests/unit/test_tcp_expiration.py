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
"""TCP expiration publishing must not resolve an unused TCP after unmount."""

from __future__ import annotations

from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time
from rclpy.time import Time as RclpyTime

from endtools.model.dispensing_tool_config_dto import DispensingToolConfigDTO
from endtools.srv import SetTcp
from endtools.volumetric_dispensing_tool import VolumetricDispensingTool
from repositories.models.material_repository import FluidMaterial

_MATERIAL = FluidMaterial(
    product_number='material-123',
    product_revision='a1',
    material_type='fluid',
    is_serialized=False,
    density_kg_m3=3150.0,
    viscosity_pa_s=20.0,
    color_rgba=[1.0, 0.631, 0.0, 1.0],
)


@pytest.fixture
def tool(monkeypatch: pytest.MonkeyPatch) -> VolumetricDispensingTool:
    """Provide real TCP resolution and publishing with mocked ROS dependencies."""
    node = object.__new__(VolumetricDispensingTool)
    node._config = DispensingToolConfigDTO(tool_sn='abc123', mounted=False, material=_MATERIAL)
    node._tool_tcp = None
    node._tcp_expiration = None
    node._tcp_expiration_publisher = Mock()
    monkeypatch.setattr(node, '_fetch_tcp_from_cache', Mock(return_value=None))
    monkeypatch.setattr(node, 'get_logger', Mock())
    clock = Mock()
    clock.now.return_value = RclpyTime(seconds=10)
    monkeypatch.setattr(node, 'get_clock', Mock(return_value=clock))
    return node


def test_set_tcp_while_unmounted_preserves_expiration(
    tool: VolumetricDispensingTool,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Setting calibration on a racked tool still stores and publishes its expiry."""
    monkeypatch.setattr(tool, '_store_tcp_in_cache', Mock(return_value=True))
    tool._static_tf_publisher = Mock()
    request = SetTcp.Request(tcp=[0.0] * 6, tcp_valid_period=90.0)

    response = tool._handle_set_tcp(request, SetTcp.Response())

    assert response.success is True
    tool._store_tcp_in_cache.assert_called_once_with([0.0] * 6, 90.0)
    tool._fetch_tcp_from_cache.assert_not_called()
    tool._static_tf_publisher.publish.assert_not_called()
    tool._tcp_expiration_publisher.publish.assert_called_once_with(Time(sec=100))
    tool.get_logger().warning.assert_not_called()


@pytest.mark.parametrize('cached_expiration', [None, Time(sec=100)])
def test_unmounted_expiration_does_not_fetch_tcp(
    tool: VolumetricDispensingTool,
    cached_expiration: Time | None,
) -> None:
    """An unmounted tool preserves known expiry without cache access or warnings."""
    tool._tcp_expiration = cached_expiration

    tool._publish_tcp_expiration()

    tool._fetch_tcp_from_cache.assert_not_called()
    tool.get_logger().warning.assert_not_called()
    tool._tcp_expiration_publisher.publish.assert_called_once_with(
        cached_expiration or Time(sec=10),
    )


def test_mounted_expiration_uses_cached_tcp(tool: VolumetricDispensingTool) -> None:
    """A mounted tool resolves the cache and publishes the calibrated TCP expiry."""
    tool._config = tool._config.model_copy(update={'mounted': True})
    tool._fetch_tcp_from_cache.return_value = ([0.0] * 6, Time(sec=100))

    tool._publish_tcp_expiration()

    tool._fetch_tcp_from_cache.assert_called_once_with()
    tool.get_logger().warning.assert_not_called()
    tool._tcp_expiration_publisher.publish.assert_called_once_with(Time(sec=100))


def test_mounted_missing_tcp_still_warns(tool: VolumetricDispensingTool) -> None:
    """Missing calibration on a mounted tool still warns and publishes expired state."""
    tool._config = tool._config.model_copy(update={'mounted': True})

    tool._publish_tcp_expiration()

    tool._fetch_tcp_from_cache.assert_called_once_with()
    tool.get_logger().warning.assert_called_once()
    assert 'falling back to uncalibrated TCP' in (
        tool.get_logger().warning.call_args.args[0]
    )
    tool._tcp_expiration_publisher.publish.assert_called_once_with(Time(sec=10))
