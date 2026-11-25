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
"""Simulated material must be welded to the product announced by the station workplace."""

from __future__ import annotations

from unittest.mock import Mock

import pytest
from rclpy.time import Time

from endtools.model.dispensing_tool_config_dto import DispensingToolConfigDTO
from endtools.service.simulated_dispenser_controller import SimulatedDispenserController
from repositories.models.material_repository import FluidMaterial
from station_workplace.msg import SimulatedProductMetadata

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
def controller() -> SimulatedDispenserController:
    """Provide a set-up controller whose TF, SDF and Gazebo dependencies are mocked."""
    controller = SimulatedDispenserController(Mock())
    controller._config = DispensingToolConfigDTO(tool_sn='abc123', material=_MATERIAL)
    controller._owner = 'endtools:abc123'
    controller._world_client = Mock()
    controller._lookup_tcp_position = Mock(side_effect=[(0.0, 0.0, 0.0), (0.01, 0.0, 0.0)])
    controller._generate_sdf_content = Mock(return_value=('stripe_x', '<sdf/>'))
    return controller


def _spawn(controller: SimulatedDispenserController) -> str | None:
    return controller._spawn_dispensed_material(
        volume_cc=0.1, start_time=Time(seconds=1), end_time=Time(seconds=2))


def test_material_is_welded_to_announced_product(controller: SimulatedDispenserController) -> None:
    """The announced model and link are passed to the material SDF."""
    controller._on_product_metadata(SimulatedProductMetadata(
        spawned=True, product_model_name='heatsink_model', product_link='heatsink'))

    assert _spawn(controller) == 'stripe_x'

    args = controller._generate_sdf_content.call_args.args
    assert args[:2] == ('heatsink_model', 'heatsink')
    controller._world_client.spawn.assert_called_once()


@pytest.mark.parametrize('messages', [
    [],
    [
        SimulatedProductMetadata(
            spawned=True, product_model_name='heatsink_model', product_link='heatsink'),
        SimulatedProductMetadata(spawned=False),
    ],
])
def test_material_is_not_spawned_without_product(
    controller: SimulatedDispenserController,
    messages: list[SimulatedProductMetadata],
) -> None:
    """Nothing is spawned before any announcement or after the product was removed."""
    for msg in messages:
        controller._on_product_metadata(msg)

    assert _spawn(controller) is None

    controller._world_client.spawn.assert_not_called()
    controller._node.get_logger().error.assert_called_once()
