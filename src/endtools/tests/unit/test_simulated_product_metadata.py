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
from world_manager.exceptions import WorldManagerError

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


def _announce(controller: SimulatedDispenserController, model: str = 'heatsink_model') -> None:
    controller._on_product_metadata(SimulatedProductMetadata(
        spawned=True, product_model_name=model, product_link='heatsink'))


def _prepare_spawns(controller: SimulatedDispenserController, names: list[str]) -> None:
    controller._lookup_tcp_position = Mock(return_value=(0.0, 0.0, 0.0))
    controller._generate_sdf_content = Mock(side_effect=[(name, '<sdf/>') for name in names])


def test_material_is_chained_onto_the_previous_material(controller: SimulatedDispenserController) -> None:
    """Only the first material is welded to the product; the rest to their predecessor."""
    _announce(controller)
    _prepare_spawns(controller, ['stripe_1', 'dot_2', 'stripe_3'])

    for _ in range(3):
        _spawn(controller)

    anchors = [call.args[:2] for call in controller._generate_sdf_content.call_args_list]
    assert anchors == [
        ('heatsink_model', 'heatsink'),
        ('stripe_1', SimulatedDispenserController._MATERIAL_LINK),
        ('dot_2', SimulatedDispenserController._MATERIAL_LINK),
    ]


def test_chain_restarts_at_a_new_product(controller: SimulatedDispenserController) -> None:
    """A new product has none of the previous product's material to chain onto."""
    _announce(controller)
    _prepare_spawns(controller, ['stripe_1', 'stripe_2'])
    _spawn(controller)

    _announce(controller, model='other_heatsink_model')
    _spawn(controller)

    assert controller._generate_sdf_content.call_args.args[:2] == ('other_heatsink_model', 'heatsink')


def test_chain_is_kept_when_the_same_product_is_announced_again(
    controller: SimulatedDispenserController,
) -> None:
    """Re-delivery of the latched announcement must not break the chain."""
    _announce(controller)
    _prepare_spawns(controller, ['stripe_1', 'stripe_2'])
    _spawn(controller)

    _announce(controller)
    _spawn(controller)

    assert controller._generate_sdf_content.call_args.args[:2] == (
        'stripe_1', SimulatedDispenserController._MATERIAL_LINK)


def test_chain_restarts_at_the_product_after_a_failed_spawn(
    controller: SimulatedDispenserController,
) -> None:
    """A failed spawn may mean the chain tail is gone, so the next material welds to the product."""
    _announce(controller)
    _prepare_spawns(controller, ['stripe_1', 'stripe_2', 'stripe_3'])
    _spawn(controller)
    controller._world_client.spawn.side_effect = WorldManagerError('boom')
    assert _spawn(controller) is None
    controller._world_client.spawn.side_effect = None

    _spawn(controller)

    assert controller._generate_sdf_content.call_args.args[:2] == ('heatsink_model', 'heatsink')
